// SPDX-License-Identifier: GPL-2.0
/*
 * ak7375 vcm driver
 *
 * Copyright (C) 2018 Intel Corporation
 *
 * Ported onto the Rockchip rk_cam_vcm framework (logical-position model,
 * RK_VIDIOC_* ioctls, move-timing bookkeeping) so rkaiq can drive continuous
 * autofocus. The slew-rate tables, advanced/DLC/SAC modes, separate avdd
 * regulator and xsd-gpio from the dw9714 template are not applicable to the
 * AK7375 (a simple 12-bit DAC powered from the shared camera rail) and were
 * dropped.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/rk-camera-module.h>
#include <linux/version.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <linux/rk_vcm_head.h>
#include <linux/compat.h>

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x0)
#define AK7375_NAME			"ak7375"

/* AK7375 is a 12-bit DAC; the position occupies bits [15:4] of the 16-bit
 * write to AK7375_REG_POSITION (i.e. value = dac << 4). */
#define AK7375_MAX_REG			4095U
#define AK7375_GRADUAL_MOVELENS_STEPS	64
#define AK7375_CTRL_DELAY_US		1000

/* The framework maps logical positions onto a "current" range. The AK7375 has
 * no real current rating, so the values are used purely as DAC proxies: the
 * defaults below spread logical 0..max across the full DAC range. */
#define AK7375_MAX_CURRENT		100U
#define AK7375_DEFAULT_START_CURRENT	0
#define AK7375_DEFAULT_RATED_CURRENT	100
#define AK7375_DEFAULT_STEP_MODE	0

#define AK7375_REG_POSITION		0x0
#define AK7375_REG_CONT			0x2
#define AK7375_MODE_ACTIVE		0x0
#define AK7375_MODE_STANDBY		0x40

/* ak7375 device structure */
struct ak7375_device {
	struct v4l2_ctrl_handler ctrls_vcm;
	struct v4l2_ctrl *focus;
	struct v4l2_subdev sd;
	struct v4l2_device vdev;

	unsigned short current_related_pos;
	unsigned short current_lens_pos;
	unsigned int max_current;
	unsigned int start_current;
	unsigned int rated_current;
	unsigned int step_mode;
	unsigned int vcm_movefull_t;
	unsigned int max_logicalpos;

	struct __kernel_old_timeval start_move_tv;
	struct __kernel_old_timeval end_move_tv;
	unsigned long move_ms;

	u32 module_index;
	const char *module_facing;
	struct rk_cam_vcm_cfg vcm_cfg;

	struct i2c_client *client;
	/* active or standby mode */
	bool active;
};

static inline struct ak7375_device *to_ak7375_vcm(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct ak7375_device, ctrls_vcm);
}

static inline struct ak7375_device *sd_to_ak7375_vcm(struct v4l2_subdev *subdev)
{
	return container_of(subdev, struct ak7375_device, sd);
}

static int ak7375_i2c_write(struct ak7375_device *ak7375,
	u8 addr, u16 data, u8 size)
{
	struct i2c_client *client = v4l2_get_subdevdata(&ak7375->sd);
	u8 buf[3];
	int ret;

	if (size != 1 && size != 2)
		return -EINVAL;
	buf[0] = addr;
	buf[size] = data & 0xff;
	if (size == 2)
		buf[1] = (data >> 8) & 0xff;
	ret = i2c_master_send(client, (const char *)buf, size + 1);
	if (ret < 0)
		return ret;
	if (ret != size + 1)
		return -EIO;

	return 0;
}

/*
 * Estimate the time to move the lens by move_dac DAC counts. The AK7375 has no
 * programmable slew control, so we model the gradual ramp used on open/close:
 * AK7375_GRADUAL_MOVELENS_STEPS counts per AK7375_CTRL_DELAY_US. This is a
 * conservative (over-)estimate of the settle time, which keeps rkaiq from
 * sampling sharpness mid-move during a CAF search.
 */
static unsigned int ak7375_move_time(struct ak7375_device *dev_vcm,
	unsigned int move_dac)
{
	unsigned int steps;

	steps = (move_dac + AK7375_GRADUAL_MOVELENS_STEPS - 1) /
		AK7375_GRADUAL_MOVELENS_STEPS;

	return (steps * AK7375_CTRL_DELAY_US + 999) / 1000;
}

static int ak7375_set_dac(struct ak7375_device *dev_vcm, unsigned int dac)
{
	if (dac > AK7375_MAX_REG)
		dac = AK7375_MAX_REG;

	return ak7375_i2c_write(dev_vcm, AK7375_REG_POSITION, dac << 4, 2);
}

/*
 * The AK7375 does not provide a reliable position readback, so the framework's
 * get_pos consumes the cached DAC value instead of reading the device.
 */
static int ak7375_get_dac(struct ak7375_device *dev_vcm, unsigned int *cur_dac)
{
	*cur_dac = dev_vcm->current_lens_pos;

	return 0;
}

static int ak7375_get_pos(struct ak7375_device *dev_vcm,
	unsigned int *cur_pos)
{
	struct i2c_client *client = v4l2_get_subdevdata(&dev_vcm->sd);
	unsigned int dac, position, range;
	int ret;

	range = dev_vcm->rated_current - dev_vcm->start_current;
	ret = ak7375_get_dac(dev_vcm, &dac);
	if (!ret) {
		if (dac <= dev_vcm->start_current) {
			position = dev_vcm->max_logicalpos;
		} else if ((dac > dev_vcm->start_current) &&
			 (dac <= dev_vcm->rated_current)) {
			position = (dac - dev_vcm->start_current) *
				   dev_vcm->max_logicalpos / range;
			position = dev_vcm->max_logicalpos - position;
		} else {
			position = 0;
		}

		*cur_pos = position;

		dev_dbg(&client->dev, "%s: get position %d, dac %d\n",
			__func__, *cur_pos, dac);
		return 0;
	}

	dev_err(&client->dev,
		"%s: failed with error %d\n", __func__, ret);
	return ret;
}

static int ak7375_set_pos(struct ak7375_device *dev_vcm,
	unsigned int dest_pos)
{
	struct i2c_client *client = v4l2_get_subdevdata(&dev_vcm->sd);
	unsigned int position;
	unsigned int range;
	int ret;

	range = dev_vcm->rated_current - dev_vcm->start_current;
	if (dest_pos >= dev_vcm->max_logicalpos)
		position = dev_vcm->start_current;
	else
		position = dev_vcm->start_current +
			   (range * (dev_vcm->max_logicalpos - dest_pos) /
			    dev_vcm->max_logicalpos);

	if (position > AK7375_MAX_REG)
		position = AK7375_MAX_REG;

	dev_vcm->current_lens_pos = position;
	dev_vcm->current_related_pos = dest_pos;

	ret = ak7375_set_dac(dev_vcm, position);
	dev_dbg(&client->dev, "%s: set position %d, dac %d\n",
		__func__, dest_pos, position);

	return ret;
}

static int ak7375_get_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ak7375_device *dev_vcm = to_ak7375_vcm(ctrl);

	if (ctrl->id == V4L2_CID_FOCUS_ABSOLUTE)
		return ak7375_get_pos(dev_vcm, &ctrl->val);

	return -EINVAL;
}

static int ak7375_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ak7375_device *dev_vcm = to_ak7375_vcm(ctrl);
	struct i2c_client *client = v4l2_get_subdevdata(&dev_vcm->sd);
	unsigned int dest_pos = ctrl->val;
	int move_pos;
	long mv_us;
	int ret = 0;

	if (ctrl->id == V4L2_CID_FOCUS_ABSOLUTE) {
		if (dest_pos > dev_vcm->max_logicalpos) {
			dev_err(&client->dev,
				"%s dest_pos is error. %d > %d\n",
				__func__, dest_pos, dev_vcm->max_logicalpos);
			return -EINVAL;
		}
		/* calculate move time */
		move_pos = dev_vcm->current_related_pos - dest_pos;
		if (move_pos < 0)
			move_pos = -move_pos;

		ret = ak7375_set_pos(dev_vcm, dest_pos);

		dev_vcm->move_ms =
			((dev_vcm->vcm_movefull_t * (uint32_t)move_pos) /
			dev_vcm->max_logicalpos);

		dev_dbg(&client->dev,
			"dest_pos %d, dac %d, move_ms %ld\n",
			dest_pos, dev_vcm->current_lens_pos, dev_vcm->move_ms);

		dev_vcm->start_move_tv = ns_to_kernel_old_timeval(ktime_get_ns());
		mv_us = dev_vcm->start_move_tv.tv_usec +
				dev_vcm->move_ms * 1000;
		if (mv_us >= 1000000) {
			dev_vcm->end_move_tv.tv_sec =
				dev_vcm->start_move_tv.tv_sec + 1;
			dev_vcm->end_move_tv.tv_usec = mv_us - 1000000;
		} else {
			dev_vcm->end_move_tv.tv_sec =
					dev_vcm->start_move_tv.tv_sec;
			dev_vcm->end_move_tv.tv_usec = mv_us;
		}
	}

	return ret;
}

static const struct v4l2_ctrl_ops ak7375_vcm_ctrl_ops = {
	.g_volatile_ctrl = ak7375_get_ctrl,
	.s_ctrl = ak7375_set_ctrl,
};

static int ak7375_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	int ret;

	ret = pm_runtime_get_sync(sd->dev);
	if (ret < 0) {
		pm_runtime_put_noidle(sd->dev);
		return ret;
	}

	return 0;
}

static int ak7375_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	pm_runtime_put(sd->dev);

	return 0;
}

static const struct v4l2_subdev_internal_ops ak7375_int_ops = {
	.open = ak7375_open,
	.close = ak7375_close,
};

static void ak7375_update_vcm_cfg(struct ak7375_device *dev_vcm)
{
	struct i2c_client *client = v4l2_get_subdevdata(&dev_vcm->sd);

	if (dev_vcm->max_current == 0) {
		dev_err(&client->dev, "max current is zero");
		return;
	}

	dev_vcm->start_current = dev_vcm->vcm_cfg.start_ma *
				 AK7375_MAX_REG / dev_vcm->max_current;
	dev_vcm->rated_current = dev_vcm->vcm_cfg.rated_ma *
				 AK7375_MAX_REG / dev_vcm->max_current;
	dev_vcm->step_mode = dev_vcm->vcm_cfg.step_mode;

	dev_dbg(&client->dev,
		"vcm_cfg: %d, %d, %d, max_current %d\n",
		dev_vcm->vcm_cfg.start_ma,
		dev_vcm->vcm_cfg.rated_ma,
		dev_vcm->vcm_cfg.step_mode,
		dev_vcm->max_current);
}

static long ak7375_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct ak7375_device *dev_vcm = sd_to_ak7375_vcm(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct rk_cam_vcm_tim *vcm_tim;
	struct rk_cam_vcm_cfg *vcm_cfg;
	unsigned int max_logicalpos;
	int ret = 0;

	if (cmd == RK_VIDIOC_VCM_TIMEINFO) {
		vcm_tim = (struct rk_cam_vcm_tim *)arg;

		vcm_tim->vcm_start_t.tv_sec = dev_vcm->start_move_tv.tv_sec;
		vcm_tim->vcm_start_t.tv_usec =
				dev_vcm->start_move_tv.tv_usec;
		vcm_tim->vcm_end_t.tv_sec = dev_vcm->end_move_tv.tv_sec;
		vcm_tim->vcm_end_t.tv_usec = dev_vcm->end_move_tv.tv_usec;

		dev_dbg(&client->dev, "ak7375_get_move_res 0x%lx, 0x%lx, 0x%lx, 0x%lx\n",
			vcm_tim->vcm_start_t.tv_sec,
			vcm_tim->vcm_start_t.tv_usec,
			vcm_tim->vcm_end_t.tv_sec,
			vcm_tim->vcm_end_t.tv_usec);
	} else if (cmd == RK_VIDIOC_GET_VCM_CFG) {
		vcm_cfg = (struct rk_cam_vcm_cfg *)arg;

		vcm_cfg->start_ma = dev_vcm->vcm_cfg.start_ma;
		vcm_cfg->rated_ma = dev_vcm->vcm_cfg.rated_ma;
		vcm_cfg->step_mode = dev_vcm->vcm_cfg.step_mode;
	} else if (cmd == RK_VIDIOC_SET_VCM_CFG) {
		vcm_cfg = (struct rk_cam_vcm_cfg *)arg;

		dev_vcm->vcm_cfg.start_ma = vcm_cfg->start_ma;
		dev_vcm->vcm_cfg.rated_ma = vcm_cfg->rated_ma;
		dev_vcm->vcm_cfg.step_mode = vcm_cfg->step_mode;
		ak7375_update_vcm_cfg(dev_vcm);
	} else if (cmd == RK_VIDIOC_SET_VCM_MAX_LOGICALPOS) {
		max_logicalpos = *(unsigned int *)arg;

		if (max_logicalpos > 0) {
			dev_vcm->max_logicalpos = max_logicalpos;
			__v4l2_ctrl_modify_range(dev_vcm->focus,
				0, dev_vcm->max_logicalpos, 1, dev_vcm->max_logicalpos);
		}
		dev_dbg(&client->dev,
			"max_logicalpos %d\n", max_logicalpos);
	} else {
		dev_err(&client->dev,
			"cmd 0x%x not supported\n", cmd);
		return -EINVAL;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long ak7375_compat_ioctl32(struct v4l2_subdev *sd,
	unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	void __user *up = compat_ptr(arg);
	struct rk_cam_compat_vcm_tim compat_vcm_tim;
	struct rk_cam_vcm_tim vcm_tim;
	struct rk_cam_vcm_cfg vcm_cfg;
	unsigned int max_logicalpos;
	long ret;

	if (cmd == RK_VIDIOC_COMPAT_VCM_TIMEINFO) {
		struct rk_cam_compat_vcm_tim __user *p32 = up;

		ret = ak7375_ioctl(sd, RK_VIDIOC_VCM_TIMEINFO, &vcm_tim);
		compat_vcm_tim.vcm_start_t.tv_sec = vcm_tim.vcm_start_t.tv_sec;
		compat_vcm_tim.vcm_start_t.tv_usec = vcm_tim.vcm_start_t.tv_usec;
		compat_vcm_tim.vcm_end_t.tv_sec = vcm_tim.vcm_end_t.tv_sec;
		compat_vcm_tim.vcm_end_t.tv_usec = vcm_tim.vcm_end_t.tv_usec;

		put_user(compat_vcm_tim.vcm_start_t.tv_sec,
			&p32->vcm_start_t.tv_sec);
		put_user(compat_vcm_tim.vcm_start_t.tv_usec,
			&p32->vcm_start_t.tv_usec);
		put_user(compat_vcm_tim.vcm_end_t.tv_sec,
			&p32->vcm_end_t.tv_sec);
		put_user(compat_vcm_tim.vcm_end_t.tv_usec,
			&p32->vcm_end_t.tv_usec);
	} else if (cmd == RK_VIDIOC_GET_VCM_CFG) {
		ret = ak7375_ioctl(sd, RK_VIDIOC_GET_VCM_CFG, &vcm_cfg);
		if (!ret) {
			ret = copy_to_user(up, &vcm_cfg, sizeof(vcm_cfg));
			if (ret)
				ret = -EFAULT;
		}
	} else if (cmd == RK_VIDIOC_SET_VCM_CFG) {
		ret = copy_from_user(&vcm_cfg, up, sizeof(vcm_cfg));
		if (!ret)
			ret = ak7375_ioctl(sd, cmd, &vcm_cfg);
		else
			ret = -EFAULT;
	} else if (cmd == RK_VIDIOC_SET_VCM_MAX_LOGICALPOS) {
		ret = copy_from_user(&max_logicalpos, up, sizeof(max_logicalpos));
		if (!ret)
			ret = ak7375_ioctl(sd, cmd, &max_logicalpos);
		else
			ret = -EFAULT;
	} else {
		dev_err(&client->dev,
			"cmd 0x%x not supported\n", cmd);
		return -EINVAL;
	}

	return ret;
}
#endif

static const struct v4l2_subdev_core_ops ak7375_core_ops = {
	.ioctl = ak7375_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = ak7375_compat_ioctl32
#endif
};

static const struct v4l2_subdev_ops ak7375_ops = {
	.core = &ak7375_core_ops,
};

static void ak7375_subdev_cleanup(struct ak7375_device *ak7375_dev)
{
	v4l2_device_unregister_subdev(&ak7375_dev->sd);
	v4l2_device_unregister(&ak7375_dev->vdev);
	v4l2_ctrl_handler_free(&ak7375_dev->ctrls_vcm);
	media_entity_cleanup(&ak7375_dev->sd.entity);
}

static int ak7375_init_controls(struct ak7375_device *dev_vcm)
{
	struct v4l2_ctrl_handler *hdl = &dev_vcm->ctrls_vcm;
	const struct v4l2_ctrl_ops *ops = &ak7375_vcm_ctrl_ops;

	v4l2_ctrl_handler_init(hdl, 1);

	dev_vcm->focus = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_FOCUS_ABSOLUTE,
				0, dev_vcm->max_logicalpos, 1, dev_vcm->max_logicalpos);

	if (hdl->error)
		dev_err(dev_vcm->sd.dev, "%s fail error: 0x%x\n",
			__func__, hdl->error);
	dev_vcm->sd.ctrl_handler = hdl;
	return hdl->error;
}

#define USED_SYS_DEBUG
#ifdef USED_SYS_DEBUG
static ssize_t set_dacval(struct device *dev,
	struct device_attribute *attr,
	const char *buf,
	size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ak7375_device *dev_vcm = sd_to_ak7375_vcm(sd);
	int val = 0;
	int ret = 0;

	ret = kstrtoint(buf, 0, &val);
	if (!ret)
		ak7375_set_dac(dev_vcm, val);

	return count;
}

static ssize_t get_dacval(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ak7375_device *dev_vcm = sd_to_ak7375_vcm(sd);
	unsigned int dac = 0;

	ak7375_get_dac(dev_vcm, &dac);
	return sprintf(buf, "%u\n", dac);
}

static struct device_attribute attributes[] = {
	__ATTR(dacval, 0600, get_dacval, set_dacval),
};

static int add_sysfs_interfaces(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto undo;
	return 0;
undo:
	for (i--; i >= 0 ; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s: failed to create sysfs interface\n", __func__);
	return -ENODEV;
}

static int remove_sysfs_interfaces(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
	return 0;
}
#else
static inline int add_sysfs_interfaces(struct device *dev)
{
	return 0;
}

static inline int remove_sysfs_interfaces(struct device *dev)
{
	return 0;
}
#endif

static int ak7375_parse_dt_property(struct i2c_client *client,
				    struct ak7375_device *dev_vcm)
{
	struct device_node *np = of_node_get(client->dev.of_node);
	int ret;

	if (of_property_read_u32(np,
		OF_CAMERA_VCMDRV_MAX_CURRENT,
		(unsigned int *)&dev_vcm->max_current)) {
		dev_vcm->max_current = AK7375_MAX_CURRENT;
		dev_info(&client->dev,
			"could not get module %s from dts!\n",
			OF_CAMERA_VCMDRV_MAX_CURRENT);
	}
	if (dev_vcm->max_current == 0)
		dev_vcm->max_current = AK7375_MAX_CURRENT;

	if (of_property_read_u32(np,
		OF_CAMERA_VCMDRV_START_CURRENT,
		(unsigned int *)&dev_vcm->vcm_cfg.start_ma)) {
		dev_vcm->vcm_cfg.start_ma = AK7375_DEFAULT_START_CURRENT;
		dev_info(&client->dev,
			"could not get module %s from dts!\n",
			OF_CAMERA_VCMDRV_START_CURRENT);
	}
	if (of_property_read_u32(np,
		OF_CAMERA_VCMDRV_RATED_CURRENT,
		(unsigned int *)&dev_vcm->vcm_cfg.rated_ma)) {
		dev_vcm->vcm_cfg.rated_ma = AK7375_DEFAULT_RATED_CURRENT;
		dev_info(&client->dev,
			"could not get module %s from dts!\n",
			OF_CAMERA_VCMDRV_RATED_CURRENT);
	}
	if (of_property_read_u32(np,
		OF_CAMERA_VCMDRV_STEP_MODE,
		(unsigned int *)&dev_vcm->vcm_cfg.step_mode)) {
		dev_vcm->vcm_cfg.step_mode = AK7375_DEFAULT_STEP_MODE;
		dev_info(&client->dev,
			"could not get module %s from dts!\n",
			OF_CAMERA_VCMDRV_STEP_MODE);
	}

	ret = of_property_read_u32(np, RKMODULE_CAMERA_MODULE_INDEX,
				   &dev_vcm->module_index);
	ret |= of_property_read_string(np, RKMODULE_CAMERA_MODULE_FACING,
				       &dev_vcm->module_facing);
	if (ret) {
		dev_err(&client->dev,
			"could not get module information!\n");
		return -EINVAL;
	}

	dev_vcm->client = client;

	dev_dbg(&client->dev, "current: %d, %d, %d",
		dev_vcm->max_current,
		dev_vcm->start_current,
		dev_vcm->rated_current);

	return 0;
}

static int ak7375_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct ak7375_device *ak7375_dev;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(&client->dev, "probing...\n");
	ak7375_dev = devm_kzalloc(&client->dev, sizeof(*ak7375_dev),
				  GFP_KERNEL);
	if (ak7375_dev == NULL)
		return -ENOMEM;

	ret = ak7375_parse_dt_property(client, ak7375_dev);
	if (ret)
		return ret;
	v4l2_i2c_subdev_init(&ak7375_dev->sd, client, &ak7375_ops);
	ak7375_dev->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	ak7375_dev->sd.internal_ops = &ak7375_int_ops;

	ak7375_dev->max_logicalpos = VCMDRV_MAX_LOG;
	ret = ak7375_init_controls(ak7375_dev);
	if (ret)
		goto err_cleanup;

	ret = media_entity_pads_init(&ak7375_dev->sd.entity, 0, NULL);
	if (ret < 0)
		goto err_cleanup;

	sd = &ak7375_dev->sd;
	sd->entity.function = MEDIA_ENT_F_LENS;

	memset(facing, 0, sizeof(facing));
	if (strcmp(ak7375_dev->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 ak7375_dev->module_index, facing,
		 AK7375_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev(sd);
	if (ret)
		dev_err(&client->dev, "v4l2 async register subdev failed\n");

	ak7375_update_vcm_cfg(ak7375_dev);
	ak7375_dev->move_ms = 0;
	ak7375_dev->current_related_pos = ak7375_dev->max_logicalpos;
	ak7375_dev->current_lens_pos = ak7375_dev->start_current;
	ak7375_dev->start_move_tv = ns_to_kernel_old_timeval(ktime_get_ns());
	ak7375_dev->end_move_tv = ns_to_kernel_old_timeval(ktime_get_ns());
	ak7375_dev->vcm_movefull_t =
		ak7375_move_time(ak7375_dev, AK7375_MAX_REG);

	pm_runtime_set_active(&client->dev);
	pm_runtime_enable(&client->dev);
	pm_runtime_idle(&client->dev);

	add_sysfs_interfaces(&client->dev);
	dev_info(&client->dev, "probing successful\n");

	return 0;

err_cleanup:
	v4l2_ctrl_handler_free(&ak7375_dev->ctrls_vcm);
	media_entity_cleanup(&ak7375_dev->sd.entity);
	dev_err(&client->dev, "Probe failed: %d\n", ret);
	return ret;
}

static int ak7375_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ak7375_device *ak7375_dev = sd_to_ak7375_vcm(sd);

	remove_sysfs_interfaces(&client->dev);
	pm_runtime_disable(&client->dev);
	ak7375_subdev_cleanup(ak7375_dev);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

/*
 * Park the lens (gradually, to keep movement smooth) and put the AK7375 into
 * standby so it draws minimal current while the device is suspended.
 */
static int __maybe_unused ak7375_vcm_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ak7375_device *ak7375_dev = sd_to_ak7375_vcm(sd);
	int ret, val;

	if (!ak7375_dev->active)
		return 0;

	for (val = ak7375_dev->current_lens_pos &
		   ~(AK7375_GRADUAL_MOVELENS_STEPS - 1);
	     val >= 0; val -= AK7375_GRADUAL_MOVELENS_STEPS) {
		ret = ak7375_set_dac(ak7375_dev, val);
		if (ret)
			dev_err_once(dev, "%s I2C failure: %d\n",
				     __func__, ret);
		usleep_range(AK7375_CTRL_DELAY_US, AK7375_CTRL_DELAY_US + 10);
	}

	ret = ak7375_i2c_write(ak7375_dev, AK7375_REG_CONT,
			       AK7375_MODE_STANDBY, 1);
	if (ret)
		dev_err(dev, "%s I2C failure: %d\n", __func__, ret);

	ak7375_dev->active = false;

	return 0;
}

/*
 * Bring the AK7375 back to active mode and gradually restore the lens to the
 * last commanded position.
 */
static int __maybe_unused ak7375_vcm_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ak7375_device *ak7375_dev = sd_to_ak7375_vcm(sd);
	int ret, val;

	if (ak7375_dev->active)
		return 0;

	ret = ak7375_i2c_write(ak7375_dev, AK7375_REG_CONT,
		AK7375_MODE_ACTIVE, 1);
	if (ret) {
		dev_err(dev, "%s I2C failure: %d\n", __func__, ret);
		return ret;
	}

	for (val = ak7375_dev->current_lens_pos %
		   AK7375_GRADUAL_MOVELENS_STEPS;
	     val <= ak7375_dev->current_lens_pos;
	     val += AK7375_GRADUAL_MOVELENS_STEPS) {
		ret = ak7375_set_dac(ak7375_dev, val);
		if (ret)
			dev_err_ratelimited(dev, "%s I2C failure: %d\n",
						__func__, ret);
		usleep_range(AK7375_CTRL_DELAY_US, AK7375_CTRL_DELAY_US + 10);
	}

	ak7375_dev->active = true;

	return 0;
}

static const struct i2c_device_id ak7375_id_table[] = {
	{ AK7375_NAME, 0 },
	{ { 0 } }
};
MODULE_DEVICE_TABLE(i2c, ak7375_id_table);

static const struct of_device_id ak7375_of_table[] = {
	{ .compatible = "asahi-kasei,ak7375" },
	{ { 0 } }
};
MODULE_DEVICE_TABLE(of, ak7375_of_table);

static const struct dev_pm_ops ak7375_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ak7375_vcm_suspend, ak7375_vcm_resume)
	SET_RUNTIME_PM_OPS(ak7375_vcm_suspend, ak7375_vcm_resume, NULL)
};

static struct i2c_driver ak7375_i2c_driver = {
	.driver = {
		.name = AK7375_NAME,
		.pm = &ak7375_pm_ops,
		.of_match_table = ak7375_of_table,
	},
	.probe = &ak7375_probe,
	.remove = &ak7375_remove,
	.id_table = ak7375_id_table,
};
module_i2c_driver(ak7375_i2c_driver);

MODULE_AUTHOR("Tianshu Qiu <tian.shu.qiu@intel.com>");
MODULE_AUTHOR("Bingbu Cao <bingbu.cao@intel.com>");
MODULE_DESCRIPTION("AK7375 VCM driver");
MODULE_LICENSE("GPL v2");
