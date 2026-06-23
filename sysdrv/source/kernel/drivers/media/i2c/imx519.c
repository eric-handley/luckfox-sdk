// SPDX-License-Identifier: GPL-2.0
/*
 * imx519 driver
 *
 * Rockchip ISP camera-module skeleton (based on imx415.c) combined with the
 * Sony IMX519 register tables from the Arducam IMX519 driver
 * (Copyright (C) 2021 Arducam Technology co., Ltd.).
 *
 * Bring-up scope: single 2-lane SRGGB10 1920x1080 mode, no HDR, no autofocus.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/rk-camera-module.h>
#include <linux/pinctrl/consumer.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>

#define DRIVER_VERSION                      KERNEL_VERSION(0, 0x01, 0x00)

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN               V4L2_CID_GAIN
#endif

#define IMX519_LINK_FREQ_493M               493500000

#define IMX519_2LANES                       2

#define IMX519_XVCLK_FREQ_24M               24000000

/* Chip ID (16-bit at 0x0016) */
#define IMX519_REG_CHIP_ID                  0x0016
#define IMX519_CHIP_ID                      0x0519

#define IMX519_REG_CTRL_MODE                0x0100
#define IMX519_MODE_SW_STANDBY              0x00
#define IMX519_MODE_STREAMING               0x01

#define IMX519_REG_EXPOSURE                 0x0202
#define IMX519_EXPOSURE_OFFSET              32
#define IMX519_EXPOSURE_MIN                 20
#define IMX519_EXPOSURE_STEP                1
#define IMX519_EXPOSURE_DEFAULT             0x3e8

#define IMX519_REG_ANALOG_GAIN              0x0204
#define IMX519_ANA_GAIN_MIN                 0
#define IMX519_ANA_GAIN_MAX                 960
#define IMX519_ANA_GAIN_STEP                1
#define IMX519_ANA_GAIN_DEFAULT             0

#define IMX519_REG_DIGITAL_GAIN             0x020e
#define IMX519_DGTL_GAIN_MIN                0x0100
#define IMX519_DGTL_GAIN_MAX                0xffff
#define IMX519_DGTL_GAIN_STEP               1
#define IMX519_DGTL_GAIN_DEFAULT            0x0100

#define IMX519_REG_VTS                      0x0340
#define IMX519_VTS_MAX                      0xffdc

#define IMX519_REG_ORIENTATION              0x0101
#define IMX519_MIRROR_BIT_MASK              BIT(0)
#define IMX519_FLIP_BIT_MASK                BIT(1)

#define IMX519_REG_TEST_PATTERN             0x0600
#define IMX519_TEST_PATTERN_DISABLE         0
#define IMX519_TEST_PATTERN_SOLID_COLOR     1
#define IMX519_TEST_PATTERN_COLOR_BARS      2
#define IMX519_TEST_PATTERN_GREY_COLOR      3
#define IMX519_TEST_PATTERN_PN9             4

#define REG_NULL                            0xFFFF
#define REG_DELAY                           0xFFFE

#define IMX519_REG_VALUE_08BIT              1
#define IMX519_REG_VALUE_16BIT              2

#define OF_CAMERA_PINCTRL_STATE_DEFAULT    "rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP      "rockchip,camera_sleep"

#define IMX519_NAME                        "imx519"

struct regval {
    u16 addr;
    u8 val;
};

struct imx519_mode {
    u32 bus_fmt;
    u32 width;
    u32 height;
    struct v4l2_fract max_fps;
    u32 hts_def;
    u32 vts_def;
    u32 exp_def;
    u32 mipi_freq_idx;
    u32 bpp;
    const struct regval *global_reg_list;
    const struct regval *reg_list;
    u32 hdr_mode;
    u32 vc[PAD_MAX];
    u32 xvclk;
};

struct imx519 {
    struct i2c_client    *client;
    struct clk        *xvclk;
    struct gpio_desc    *reset_gpio;
    struct gpio_desc    *power_gpio;

    struct pinctrl        *pinctrl;
    struct pinctrl_state    *pins_default;
    struct pinctrl_state    *pins_sleep;

    struct v4l2_subdev    subdev;
    struct media_pad    pad;
    struct v4l2_ctrl_handler ctrl_handler;
    struct v4l2_ctrl    *exposure;
    struct v4l2_ctrl    *anal_a_gain;
    struct v4l2_ctrl    *digi_gain;
    struct v4l2_ctrl    *hblank;
    struct v4l2_ctrl    *vblank;
    struct v4l2_ctrl    *pixel_rate;
    struct v4l2_ctrl    *link_freq;
    struct v4l2_ctrl    *test_pattern;
    struct mutex        mutex;
    bool            streaming;
    bool            power_on;
    const struct imx519_mode *supported_modes;
    const struct imx519_mode *cur_mode;
    u32            cfg_num;
    u32            module_index;
    const char        *module_facing;
    const char        *module_name;
    const char        *len_name;
    u32            cur_vts;
    struct v4l2_fwnode_endpoint bus_cfg;
};

#define to_imx519(sd) container_of(sd, struct imx519, subdev)

static const struct regval imx519_mode_common_regs[] = {
    {0x0136, 0x18},
    {0x0137, 0x00},
    {0x3c7e, 0x01},
    {0x3c7f, 0x07},
    {0x3020, 0x00},
    {0x3e35, 0x01},
    {0x3f7f, 0x01},
    {0x5609, 0x57},
    {0x5613, 0x51},
    {0x561f, 0x5e},
    {0x5623, 0xd2},
    {0x5637, 0x11},
    {0x5657, 0x11},
    {0x5659, 0x12},
    {0x5733, 0x60},
    {0x5905, 0x57},
    {0x590f, 0x51},
    {0x591b, 0x5e},
    {0x591f, 0xd2},
    {0x5933, 0x11},
    {0x5953, 0x11},
    {0x5955, 0x12},
    {0x5a2f, 0x60},
    {0x5a85, 0x57},
    {0x5a8f, 0x51},
    {0x5a9b, 0x5e},
    {0x5a9f, 0xd2},
    {0x5ab3, 0x11},
    {0x5ad3, 0x11},
    {0x5ad5, 0x12},
    {0x5baf, 0x60},
    {0x5c15, 0x2a},
    {0x5c17, 0x80},
    {0x5c19, 0x31},
    {0x5c1b, 0x87},
    {0x5c25, 0x25},
    {0x5c27, 0x7b},
    {0x5c29, 0x2a},
    {0x5c2b, 0x80},
    {0x5c2d, 0x31},
    {0x5c2f, 0x87},
    {0x5c35, 0x2b},
    {0x5c37, 0x81},
    {0x5c39, 0x31},
    {0x5c3b, 0x87},
    {0x5c45, 0x25},
    {0x5c47, 0x7b},
    {0x5c49, 0x2a},
    {0x5c4b, 0x80},
    {0x5c4d, 0x31},
    {0x5c4f, 0x87},
    {0x5c55, 0x2d},
    {0x5c57, 0x83},
    {0x5c59, 0x32},
    {0x5c5b, 0x88},
    {0x5c65, 0x29},
    {0x5c67, 0x7f},
    {0x5c69, 0x2e},
    {0x5c6b, 0x84},
    {0x5c6d, 0x32},
    {0x5c6f, 0x88},
    {0x5e69, 0x04},
    {0x5e9d, 0x00},
    {0x5f18, 0x10},
    {0x5f1a, 0x0e},
    {0x5f20, 0x12},
    {0x5f22, 0x10},
    {0x5f24, 0x0e},
    {0x5f28, 0x10},
    {0x5f2a, 0x0e},
    {0x5f30, 0x12},
    {0x5f32, 0x10},
    {0x5f34, 0x0e},
    {0x5f38, 0x0f},
    {0x5f39, 0x0d},
    {0x5f3c, 0x11},
    {0x5f3d, 0x0f},
    {0x5f3e, 0x0d},
    {0x5f61, 0x07},
    {0x5f64, 0x05},
    {0x5f67, 0x03},
    {0x5f6a, 0x03},
    {0x5f6d, 0x07},
    {0x5f70, 0x07},
    {0x5f73, 0x05},
    {0x5f76, 0x02},
    {0x5f79, 0x07},
    {0x5f7c, 0x07},
    {0x5f7f, 0x07},
    {0x5f82, 0x07},
    {0x5f85, 0x03},
    {0x5f88, 0x02},
    {0x5f8b, 0x01},
    {0x5f8e, 0x01},
    {0x5f91, 0x04},
    {0x5f94, 0x05},
    {0x5f97, 0x02},
    {0x5f9d, 0x07},
    {0x5fa0, 0x07},
    {0x5fa3, 0x07},
    {0x5fa6, 0x07},
    {0x5fa9, 0x03},
    {0x5fac, 0x01},
    {0x5faf, 0x01},
    {0x5fb5, 0x03},
    {0x5fb8, 0x02},
    {0x5fbb, 0x01},
    {0x5fc1, 0x07},
    {0x5fc4, 0x07},
    {0x5fc7, 0x07},
    {0x5fd1, 0x00},
    {0x6302, 0x79},
    {0x6305, 0x78},
    {0x6306, 0xa5},
    {0x6308, 0x03},
    {0x6309, 0x20},
    {0x630b, 0x0a},
    {0x630d, 0x48},
    {0x630f, 0x06},
    {0x6311, 0xa4},
    {0x6313, 0x03},
    {0x6314, 0x20},
    {0x6316, 0x0a},
    {0x6317, 0x31},
    {0x6318, 0x4a},
    {0x631a, 0x06},
    {0x631b, 0x40},
    {0x631c, 0xa4},
    {0x631e, 0x03},
    {0x631f, 0x20},
    {0x6321, 0x0a},
    {0x6323, 0x4a},
    {0x6328, 0x80},
    {0x6329, 0x01},
    {0x632a, 0x30},
    {0x632b, 0x02},
    {0x632c, 0x20},
    {0x632d, 0x02},
    {0x632e, 0x30},
    {0x6330, 0x60},
    {0x6332, 0x90},
    {0x6333, 0x01},
    {0x6334, 0x30},
    {0x6335, 0x02},
    {0x6336, 0x20},
    {0x6338, 0x80},
    {0x633a, 0xa0},
    {0x633b, 0x01},
    {0x633c, 0x60},
    {0x633d, 0x02},
    {0x633e, 0x60},
    {0x633f, 0x01},
    {0x6340, 0x30},
    {0x6341, 0x02},
    {0x6342, 0x20},
    {0x6343, 0x03},
    {0x6344, 0x80},
    {0x6345, 0x03},
    {0x6346, 0x90},
    {0x6348, 0xf0},
    {0x6349, 0x01},
    {0x634a, 0x20},
    {0x634b, 0x02},
    {0x634c, 0x10},
    {0x634d, 0x03},
    {0x634e, 0x60},
    {0x6350, 0xa0},
    {0x6351, 0x01},
    {0x6352, 0x60},
    {0x6353, 0x02},
    {0x6354, 0x50},
    {0x6355, 0x02},
    {0x6356, 0x60},
    {0x6357, 0x01},
    {0x6358, 0x30},
    {0x6359, 0x02},
    {0x635a, 0x30},
    {0x635b, 0x03},
    {0x635c, 0x90},
    {0x635f, 0x01},
    {0x6360, 0x10},
    {0x6361, 0x01},
    {0x6362, 0x40},
    {0x6363, 0x02},
    {0x6364, 0x50},
    {0x6368, 0x70},
    {0x636a, 0xa0},
    {0x636b, 0x01},
    {0x636c, 0x50},
    {0x637d, 0xe4},
    {0x637e, 0xb4},
    {0x638c, 0x8e},
    {0x638d, 0x38},
    {0x638e, 0xe3},
    {0x638f, 0x4c},
    {0x6390, 0x30},
    {0x6391, 0xc3},
    {0x6392, 0xae},
    {0x6393, 0xba},
    {0x6394, 0xeb},
    {0x6395, 0x6e},
    {0x6396, 0x34},
    {0x6397, 0xe3},
    {0x6398, 0xcf},
    {0x6399, 0x3c},
    {0x639a, 0xf3},
    {0x639b, 0x0c},
    {0x639c, 0x30},
    {0x639d, 0xc1},
    {0x63b9, 0xa3},
    {0x63ba, 0xfe},
    {0x7600, 0x01},
    {0x79a0, 0x01},
    {0x79a1, 0x01},
    {0x79a2, 0x01},
    {0x79a3, 0x01},
    {0x79a4, 0x01},
    {0x79a5, 0x20},
    {0x79a9, 0x00},
    {0x79aa, 0x01},
    {0x79ad, 0x00},
    {0x79af, 0x00},
    {0x8173, 0x01},
    {0x835c, 0x01},
    {0x8a74, 0x01},
    {0x8c1f, 0x00},
    {0x8c27, 0x00},
    {0x8c3b, 0x03},
    {0x9004, 0x0b},
    {0x920c, 0x6a},
    {0x920d, 0x22},
    {0x920e, 0x6a},
    {0x920f, 0x23},
    {0x9214, 0x6a},
    {0x9215, 0x20},
    {0x9216, 0x6a},
    {0x9217, 0x21},
    {0x9385, 0x3e},
    {0x9387, 0x1b},
    {0x938d, 0x4d},
    {0x938f, 0x43},
    {0x9391, 0x1b},
    {0x9395, 0x4d},
    {0x9397, 0x43},
    {0x9399, 0x1b},
    {0x939d, 0x3e},
    {0x939f, 0x2f},
    {0x93a5, 0x43},
    {0x93a7, 0x2f},
    {0x93a9, 0x2f},
    {0x93ad, 0x34},
    {0x93af, 0x2f},
    {0x93b5, 0x3e},
    {0x93b7, 0x2f},
    {0x93bd, 0x4d},
    {0x93bf, 0x43},
    {0x93c1, 0x2f},
    {0x93c5, 0x4d},
    {0x93c7, 0x43},
    {0x93c9, 0x2f},
    {0x974b, 0x02},
    {0x995c, 0x8c},
    {0x995d, 0x00},
    {0x995e, 0x00},
    {0x9963, 0x64},
    {0x9964, 0x50},
    {0xaa0a, 0x26},
    {0xae03, 0x04},
    {0xae04, 0x03},
    {0xae05, 0x03},
    {0xbc1c, 0x08},
    {0xbcf1, 0x02},
    {REG_NULL, 0x00},
};

/* 1920x1080 2-lane SRGGB10 */
static const struct regval imx519_linear_10bit_1920x1080_regs[] = {
    {0x0111, 0x02},
    {0x0112, 0x0a},
    {0x0113, 0x0a},
    {0x0114, 0x01},
    {0x0342, 0x17},
    {0x0343, 0x8b},
    {0x0340, 0x04},
    {0x0341, 0x9c},
    {0x0344, 0x01},
    {0x0345, 0x98},
    {0x0346, 0x02},
    {0x0347, 0xa2},
    {0x0348, 0x10},
    {0x0349, 0x97},
    {0x034a, 0x0b},
    {0x034b, 0x15},
    {0x0220, 0x00},
    {0x0221, 0x11},
    {0x0222, 0x01},
    {0x0900, 0x01},
    {0x0901, 0x22},
    {0x0902, 0x0a},
    {0x3f4c, 0x01},
    {0x3f4d, 0x01},
    {0x4254, 0x7f},
    {0x0401, 0x00},
    {0x0404, 0x00},
    {0x0405, 0x10},
    {0x0408, 0x00},
    {0x0409, 0x00},
    {0x040a, 0x00},
    {0x040b, 0x00},
    {0x040c, 0x07},
    {0x040d, 0x80},
    {0x040e, 0x04},
    {0x040f, 0x38},
    {0x034c, 0x07},
    {0x034d, 0x80},
    {0x034e, 0x04},
    {0x034f, 0x38},
    {0x0301, 0x06},
    {0x0303, 0x04},
    {0x0305, 0x06},
    {0x0306, 0x01},
    {0x0307, 0x40},
    {0x0309, 0x0a},
    {0x030b, 0x02},
    {0x030d, 0x04},
    {0x030e, 0x01},
    {0x030f, 0x10},
    {0x0310, 0x01},
    {0x0820, 0x0a},
    {0x0821, 0x20},
    {0x0822, 0x00},
    {0x0823, 0x00},
    {0x3e20, 0x01},
    {0x3e37, 0x00},
    {0x3e3b, 0x00},
    {0x0106, 0x00},
    {0x0b00, 0x00},
    {0x3230, 0x00},
    {0x3f14, 0x01},
    {0x3f3c, 0x01},
    {0x3f0d, 0x0a},
    {0x3fbc, 0x00},
    {0x3c06, 0x00},
    {0x3c07, 0x48},
    {0x3c0a, 0x00},
    {0x3c0b, 0x00},
    {0x3f78, 0x00},
    {0x3f79, 0x40},
    {0x3f7c, 0x00},
    {0x3f7d, 0x00},
    {REG_NULL, 0x00},
};

static const struct imx519_mode supported_modes_2lane[] = {
    {
        .bus_fmt = MEDIA_BUS_FMT_SRGGB10_1X10,
        .width = 1920,
        .height = 1080,
        .max_fps = {
            .numerator = 10000,
            .denominator = 600000,
        },
        .exp_def = IMX519_EXPOSURE_DEFAULT,
        .hts_def = 0x178b,
        .vts_def = 0x049c,
        .global_reg_list = imx519_mode_common_regs,
        .reg_list = imx519_linear_10bit_1920x1080_regs,
        .hdr_mode = NO_HDR,
        .mipi_freq_idx = 0,
        .bpp = 10,
        .vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
        .xvclk = IMX519_XVCLK_FREQ_24M,
    },
};

static const s64 link_freq_items[] = {
    IMX519_LINK_FREQ_493M,
};

static const char * const imx519_test_pattern_menu[] = {
    "Disabled",
    "Color Bars",
    "Solid Color",
    "Grey Color Bars",
    "PN9",
};

static const int imx519_test_pattern_val[] = {
    IMX519_TEST_PATTERN_DISABLE,
    IMX519_TEST_PATTERN_COLOR_BARS,
    IMX519_TEST_PATTERN_SOLID_COLOR,
    IMX519_TEST_PATTERN_GREY_COLOR,
    IMX519_TEST_PATTERN_PN9,
};

/* Write registers up to 4 at a time */
static int imx519_write_reg(struct i2c_client *client, u16 reg,
                u32 len, u32 val)
{
    u32 buf_i, val_i;
    u8 buf[6];
    u8 *val_p;
    __be32 val_be;

    if (len > 4)
        return -EINVAL;

    buf[0] = reg >> 8;
    buf[1] = reg & 0xff;

    val_be = cpu_to_be32(val);
    val_p = (u8 *)&val_be;
    buf_i = 2;
    val_i = 4 - len;

    while (val_i < 4)
        buf[buf_i++] = val_p[val_i++];

    if (i2c_master_send(client, buf, len + 2) != len + 2)
        return -EIO;

    return 0;
}

static int imx519_write_array(struct i2c_client *client,
                  const struct regval *regs)
{
    u32 i;
    int ret = 0;

    if (!regs)
        return 0;

    for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++) {
        if (regs[i].addr == REG_DELAY) {
            usleep_range(regs[i].val * 1000, regs[i].val * 1000 + 500);
        } else {
            ret = imx519_write_reg(client, regs[i].addr,
                           IMX519_REG_VALUE_08BIT,
                           regs[i].val);
        }
    }

    return ret;
}

/* Read registers up to 4 at a time */
static int imx519_read_reg(struct i2c_client *client, u16 reg, unsigned int len,
               u32 *val)
{
    struct i2c_msg msgs[2];
    u8 *data_be_p;
    __be32 data_be = 0;
    __be16 reg_addr_be = cpu_to_be16(reg);
    int ret;

    if (len > 4 || !len)
        return -EINVAL;

    data_be_p = (u8 *)&data_be;
    msgs[0].addr = client->addr;
    msgs[0].flags = 0;
    msgs[0].len = 2;
    msgs[0].buf = (u8 *)&reg_addr_be;

    msgs[1].addr = client->addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = len;
    msgs[1].buf = &data_be_p[4 - len];

    ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
    if (ret != ARRAY_SIZE(msgs))
        return -EIO;

    *val = be32_to_cpu(data_be);

    return 0;
}

static int imx519_get_reso_dist(const struct imx519_mode *mode,
                struct v4l2_mbus_framefmt *framefmt)
{
    return abs(mode->width - framefmt->width) +
           abs(mode->height - framefmt->height);
}

static const struct imx519_mode *
imx519_find_best_fit(struct imx519 *imx519, struct v4l2_subdev_format *fmt)
{
    struct v4l2_mbus_framefmt *framefmt = &fmt->format;
    int dist;
    int cur_best_fit = 0;
    int cur_best_fit_dist = -1;
    unsigned int i;

    for (i = 0; i < imx519->cfg_num; i++) {
        dist = imx519_get_reso_dist(&imx519->supported_modes[i], framefmt);
        if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
            cur_best_fit_dist = dist;
            cur_best_fit = i;
        }
    }

    return &imx519->supported_modes[cur_best_fit];
}

static void imx519_change_mode(struct imx519 *imx519,
                   const struct imx519_mode *mode)
{
    imx519->cur_mode = mode;
    imx519->cur_vts = mode->vts_def;
    dev_info(&imx519->client->dev, "set fmt: cur_mode: %dx%d, bpp: %d\n",
         mode->width, mode->height, mode->bpp);
}

static int imx519_set_fmt(struct v4l2_subdev *sd,
              struct v4l2_subdev_pad_config *cfg,
              struct v4l2_subdev_format *fmt)
{
    struct imx519 *imx519 = to_imx519(sd);
    const struct imx519_mode *mode;
    s64 h_blank, vblank_def;
    u64 pixel_rate = 0;
    u8 lanes = imx519->bus_cfg.bus.mipi_csi2.num_data_lanes;

    mutex_lock(&imx519->mutex);

    mode = imx519_find_best_fit(imx519, fmt);
    fmt->format.code = mode->bus_fmt;
    fmt->format.width = mode->width;
    fmt->format.height = mode->height;
    fmt->format.field = V4L2_FIELD_NONE;
    if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
        *v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
        mutex_unlock(&imx519->mutex);
        return -ENOTTY;
#endif
    } else {
        imx519_change_mode(imx519, mode);
        h_blank = mode->hts_def - mode->width;
        __v4l2_ctrl_modify_range(imx519->hblank, h_blank,
                     h_blank, 1, h_blank);
        vblank_def = mode->vts_def - mode->height;
        __v4l2_ctrl_modify_range(imx519->vblank, vblank_def,
                     IMX519_VTS_MAX - mode->height,
                     1, vblank_def);
        __v4l2_ctrl_s_ctrl(imx519->vblank, vblank_def);
        __v4l2_ctrl_s_ctrl(imx519->link_freq, mode->mipi_freq_idx);
        pixel_rate = (u32)link_freq_items[mode->mipi_freq_idx] /
            mode->bpp * 2 * lanes;
        __v4l2_ctrl_s_ctrl_int64(imx519->pixel_rate, pixel_rate);
    }

    mutex_unlock(&imx519->mutex);

    return 0;
}

static int imx519_get_fmt(struct v4l2_subdev *sd,
              struct v4l2_subdev_pad_config *cfg,
              struct v4l2_subdev_format *fmt)
{
    struct imx519 *imx519 = to_imx519(sd);
    const struct imx519_mode *mode = imx519->cur_mode;

    mutex_lock(&imx519->mutex);
    if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
        fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
        mutex_unlock(&imx519->mutex);
        return -ENOTTY;
#endif
    } else {
        fmt->format.width = mode->width;
        fmt->format.height = mode->height;
        fmt->format.code = mode->bus_fmt;
        fmt->format.field = V4L2_FIELD_NONE;
        fmt->reserved[0] = mode->vc[PAD0];
    }
    mutex_unlock(&imx519->mutex);

    return 0;
}

static int imx519_enum_mbus_code(struct v4l2_subdev *sd,
                 struct v4l2_subdev_pad_config *cfg,
                 struct v4l2_subdev_mbus_code_enum *code)
{
    struct imx519 *imx519 = to_imx519(sd);

    if (code->index != 0)
        return -EINVAL;
    code->code = imx519->cur_mode->bus_fmt;

    return 0;
}

static int imx519_enum_frame_sizes(struct v4l2_subdev *sd,
                   struct v4l2_subdev_pad_config *cfg,
                   struct v4l2_subdev_frame_size_enum *fse)
{
    struct imx519 *imx519 = to_imx519(sd);

    if (fse->index >= imx519->cfg_num)
        return -EINVAL;

    if (fse->code != imx519->supported_modes[fse->index].bus_fmt)
        return -EINVAL;

    fse->min_width  = imx519->supported_modes[fse->index].width;
    fse->max_width  = imx519->supported_modes[fse->index].width;
    fse->max_height = imx519->supported_modes[fse->index].height;
    fse->min_height = imx519->supported_modes[fse->index].height;

    return 0;
}

static int imx519_g_frame_interval(struct v4l2_subdev *sd,
                   struct v4l2_subdev_frame_interval *fi)
{
    struct imx519 *imx519 = to_imx519(sd);
    const struct imx519_mode *mode = imx519->cur_mode;

    fi->interval = mode->max_fps;

    return 0;
}

static int imx519_enum_frame_interval(struct v4l2_subdev *sd,
                      struct v4l2_subdev_pad_config *cfg,
                      struct v4l2_subdev_frame_interval_enum *fie)
{
    struct imx519 *imx519 = to_imx519(sd);

    if (fie->index >= imx519->cfg_num)
        return -EINVAL;

    fie->code = imx519->supported_modes[fie->index].bus_fmt;
    fie->width = imx519->supported_modes[fie->index].width;
    fie->height = imx519->supported_modes[fie->index].height;
    fie->interval = imx519->supported_modes[fie->index].max_fps;
    fie->reserved[0] = imx519->supported_modes[fie->index].hdr_mode;

    return 0;
}

static int imx519_g_mbus_config(struct v4l2_subdev *sd, unsigned int pad_id,
                struct v4l2_mbus_config *config)
{
    struct imx519 *imx519 = to_imx519(sd);
    u8 lanes = imx519->bus_cfg.bus.mipi_csi2.num_data_lanes;
    u32 val;

    val = 1 << (lanes - 1) |
          V4L2_MBUS_CSI2_CHANNEL_0 |
          V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
    config->type = V4L2_MBUS_CSI2_DPHY;
    config->flags = val;

    return 0;
}

static void imx519_get_module_inf(struct imx519 *imx519,
                  struct rkmodule_inf *inf)
{
    memset(inf, 0, sizeof(*inf));
    strlcpy(inf->base.sensor, IMX519_NAME, sizeof(inf->base.sensor));
    strlcpy(inf->base.module, imx519->module_name,
        sizeof(inf->base.module));
    strlcpy(inf->base.lens, imx519->len_name, sizeof(inf->base.lens));
}

static long imx519_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
    struct imx519 *imx519 = to_imx519(sd);
    struct rkmodule_hdr_cfg *hdr;
    struct rkmodule_channel_info *ch_info;
    long ret = 0;
    u32 stream = 0;

    switch (cmd) {
    case RKMODULE_GET_MODULE_INFO:
        imx519_get_module_inf(imx519, (struct rkmodule_inf *)arg);
        break;
    case RKMODULE_GET_HDR_CFG:
        hdr = (struct rkmodule_hdr_cfg *)arg;
        hdr->esp.mode = HDR_NORMAL_VC;
        hdr->hdr_mode = imx519->cur_mode->hdr_mode;
        break;
    case RKMODULE_SET_HDR_CFG:
        break;
    case RKMODULE_SET_QUICK_STREAM:
        stream = *((u32 *)arg);
        ret = imx519_write_reg(imx519->client, IMX519_REG_CTRL_MODE,
                       IMX519_REG_VALUE_08BIT,
                       stream ? IMX519_MODE_STREAMING :
                       IMX519_MODE_SW_STANDBY);
        break;
    case RKMODULE_GET_CHANNEL_INFO:
        ch_info = (struct rkmodule_channel_info *)arg;
        if (ch_info->index != 0)
            return -EINVAL;
        ch_info->vc = imx519->cur_mode->vc[PAD0];
        ch_info->width = imx519->cur_mode->width;
        ch_info->height = imx519->cur_mode->height;
        ch_info->bus_fmt = imx519->cur_mode->bus_fmt;
        break;
    default:
        ret = -ENOIOCTLCMD;
        break;
    }

    return ret;
}

#ifdef CONFIG_COMPAT
static long imx519_compat_ioctl32(struct v4l2_subdev *sd,
                  unsigned int cmd, unsigned long arg)
{
    void __user *up = compat_ptr(arg);
    struct rkmodule_inf *inf;
    struct rkmodule_hdr_cfg *hdr;
    struct rkmodule_channel_info *ch_info;
    long ret;
    u32 stream = 0;

    switch (cmd) {
    case RKMODULE_GET_MODULE_INFO:
        inf = kzalloc(sizeof(*inf), GFP_KERNEL);
        if (!inf)
            return -ENOMEM;
        ret = imx519_ioctl(sd, cmd, inf);
        if (!ret) {
            ret = copy_to_user(up, inf, sizeof(*inf));
            if (ret)
                ret = -EFAULT;
        }
        kfree(inf);
        break;
    case RKMODULE_GET_HDR_CFG:
        hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
        if (!hdr)
            return -ENOMEM;
        ret = imx519_ioctl(sd, cmd, hdr);
        if (!ret) {
            ret = copy_to_user(up, hdr, sizeof(*hdr));
            if (ret)
                ret = -EFAULT;
        }
        kfree(hdr);
        break;
    case RKMODULE_SET_HDR_CFG:
        hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
        if (!hdr)
            return -ENOMEM;
        ret = copy_from_user(hdr, up, sizeof(*hdr));
        if (!ret)
            ret = imx519_ioctl(sd, cmd, hdr);
        else
            ret = -EFAULT;
        kfree(hdr);
        break;
    case RKMODULE_SET_QUICK_STREAM:
        ret = copy_from_user(&stream, up, sizeof(u32));
        if (!ret)
            ret = imx519_ioctl(sd, cmd, &stream);
        else
            ret = -EFAULT;
        break;
    case RKMODULE_GET_CHANNEL_INFO:
        ch_info = kzalloc(sizeof(*ch_info), GFP_KERNEL);
        if (!ch_info)
            return -ENOMEM;
        ret = imx519_ioctl(sd, cmd, ch_info);
        if (!ret) {
            ret = copy_to_user(up, ch_info, sizeof(*ch_info));
            if (ret)
                ret = -EFAULT;
        }
        kfree(ch_info);
        break;
    default:
        ret = -ENOIOCTLCMD;
        break;
    }

    return ret;
}
#endif

static int __imx519_start_stream(struct imx519 *imx519)
{
    int ret;

    ret = imx519_write_array(imx519->client, imx519->cur_mode->global_reg_list);
    if (ret)
        return ret;
    ret = imx519_write_array(imx519->client, imx519->cur_mode->reg_list);
    if (ret)
        return ret;

    /* In case these controls are set before streaming */
    ret = __v4l2_ctrl_handler_setup(&imx519->ctrl_handler);
    if (ret)
        return ret;

    return imx519_write_reg(imx519->client, IMX519_REG_CTRL_MODE,
                IMX519_REG_VALUE_08BIT, IMX519_MODE_STREAMING);
}

static int __imx519_stop_stream(struct imx519 *imx519)
{
    return imx519_write_reg(imx519->client, IMX519_REG_CTRL_MODE,
                IMX519_REG_VALUE_08BIT, IMX519_MODE_SW_STANDBY);
}

static int imx519_s_stream(struct v4l2_subdev *sd, int on)
{
    struct imx519 *imx519 = to_imx519(sd);
    struct i2c_client *client = imx519->client;
    int ret = 0;

    dev_info(&client->dev, "s_stream: %d. %dx%d, bpp: %d\n",
         on, imx519->cur_mode->width, imx519->cur_mode->height,
         imx519->cur_mode->bpp);

    mutex_lock(&imx519->mutex);
    on = !!on;
    if (on == imx519->streaming)
        goto unlock_and_return;

    if (on) {
        ret = pm_runtime_get_sync(&client->dev);
        if (ret < 0) {
            pm_runtime_put_noidle(&client->dev);
            goto unlock_and_return;
        }

        ret = __imx519_start_stream(imx519);
        if (ret) {
            v4l2_err(sd, "start stream failed while write regs\n");
            pm_runtime_put(&client->dev);
            goto unlock_and_return;
        }
    } else {
        __imx519_stop_stream(imx519);
        pm_runtime_put(&client->dev);
    }

    imx519->streaming = on;

unlock_and_return:
    mutex_unlock(&imx519->mutex);

    return ret;
}

static int __imx519_power_on(struct imx519 *imx519)
{
    int ret;
    struct device *dev = &imx519->client->dev;

    if (!IS_ERR_OR_NULL(imx519->pins_default)) {
        ret = pinctrl_select_state(imx519->pinctrl,
                       imx519->pins_default);
        if (ret < 0)
            dev_err(dev, "could not set pins\n");
    }

    if (!IS_ERR(imx519->power_gpio))
        gpiod_direction_output(imx519->power_gpio, 1);

    usleep_range(1000, 2000);

    if (!IS_ERR(imx519->reset_gpio))
        gpiod_direction_output(imx519->reset_gpio, 0);

    usleep_range(1000, 2000);

    if (imx519->xvclk) {
        ret = clk_set_rate(imx519->xvclk, imx519->cur_mode->xvclk);
        if (ret < 0)
            dev_warn(dev, "Failed to set xvclk rate\n");
        ret = clk_prepare_enable(imx519->xvclk);
        if (ret < 0) {
            dev_err(dev, "Failed to enable xvclk\n");
            return ret;
        }
    }

    /* At least 20us between XCLR and I2C communication */
    usleep_range(20 * 1000, 30 * 1000);

    return 0;
}

static void __imx519_power_off(struct imx519 *imx519)
{
    struct device *dev = &imx519->client->dev;
    int ret;

    if (!IS_ERR(imx519->reset_gpio))
        gpiod_direction_output(imx519->reset_gpio, 1);
    if (imx519->xvclk)
        clk_disable_unprepare(imx519->xvclk);
    if (!IS_ERR_OR_NULL(imx519->pins_sleep)) {
        ret = pinctrl_select_state(imx519->pinctrl,
                       imx519->pins_sleep);
        if (ret < 0)
            dev_dbg(dev, "could not set pins\n");
    }
    if (!IS_ERR(imx519->power_gpio))
        gpiod_direction_output(imx519->power_gpio, 0);
}

static int imx519_s_power(struct v4l2_subdev *sd, int on)
{
    struct imx519 *imx519 = to_imx519(sd);
    struct i2c_client *client = imx519->client;
    int ret = 0;

    mutex_lock(&imx519->mutex);

    if (imx519->power_on == !!on)
        goto unlock_and_return;

    if (on) {
        ret = pm_runtime_get_sync(&client->dev);
        if (ret < 0) {
            pm_runtime_put_noidle(&client->dev);
            goto unlock_and_return;
        }
        imx519->power_on = true;
    } else {
        pm_runtime_put(&client->dev);
        imx519->power_on = false;
    }

unlock_and_return:
    mutex_unlock(&imx519->mutex);

    return ret;
}

static int __maybe_unused imx519_runtime_resume(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct v4l2_subdev *sd = i2c_get_clientdata(client);
    struct imx519 *imx519 = to_imx519(sd);

    return __imx519_power_on(imx519);
}

static int __maybe_unused imx519_runtime_suspend(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct v4l2_subdev *sd = i2c_get_clientdata(client);
    struct imx519 *imx519 = to_imx519(sd);

    __imx519_power_off(imx519);

    return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int imx519_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
    struct imx519 *imx519 = to_imx519(sd);
    struct v4l2_mbus_framefmt *try_fmt =
                v4l2_subdev_get_try_format(sd, fh->pad, 0);
    const struct imx519_mode *def_mode = &imx519->supported_modes[0];

    mutex_lock(&imx519->mutex);
    try_fmt->width = def_mode->width;
    try_fmt->height = def_mode->height;
    try_fmt->code = def_mode->bus_fmt;
    try_fmt->field = V4L2_FIELD_NONE;
    mutex_unlock(&imx519->mutex);

    return 0;
}
#endif

static const struct dev_pm_ops imx519_pm_ops = {
    SET_RUNTIME_PM_OPS(imx519_runtime_suspend,
               imx519_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops imx519_internal_ops = {
    .open = imx519_open,
};
#endif

static const struct v4l2_subdev_core_ops imx519_core_ops = {
    .s_power = imx519_s_power,
    .ioctl = imx519_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl32 = imx519_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops imx519_video_ops = {
    .s_stream = imx519_s_stream,
    .g_frame_interval = imx519_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops imx519_pad_ops = {
    .enum_mbus_code = imx519_enum_mbus_code,
    .enum_frame_size = imx519_enum_frame_sizes,
    .enum_frame_interval = imx519_enum_frame_interval,
    .get_fmt = imx519_get_fmt,
    .set_fmt = imx519_set_fmt,
    .get_mbus_config = imx519_g_mbus_config,
};

static const struct v4l2_subdev_ops imx519_subdev_ops = {
    .core    = &imx519_core_ops,
    .video    = &imx519_video_ops,
    .pad    = &imx519_pad_ops,
};

static int imx519_set_ctrl(struct v4l2_ctrl *ctrl)
{
    struct imx519 *imx519 = container_of(ctrl->handler,
                         struct imx519, ctrl_handler);
    struct i2c_client *client = imx519->client;
    s64 max;
    u32 vts, val;
    int ret = 0;

    /* Propagate change of current control to all related controls */
    switch (ctrl->id) {
    case V4L2_CID_VBLANK:
        /* Update max exposure while meeting expected vblanking */
        max = imx519->cur_mode->height + ctrl->val -
              IMX519_EXPOSURE_OFFSET;
        __v4l2_ctrl_modify_range(imx519->exposure,
                     imx519->exposure->minimum, max,
                     imx519->exposure->step,
                     imx519->exposure->default_value);
        break;
    }

    if (!pm_runtime_get_if_in_use(&client->dev))
        return 0;

    switch (ctrl->id) {
    case V4L2_CID_EXPOSURE:
        ret = imx519_write_reg(client, IMX519_REG_EXPOSURE,
                       IMX519_REG_VALUE_16BIT, ctrl->val);
        break;
    case V4L2_CID_ANALOGUE_GAIN:
        ret = imx519_write_reg(client, IMX519_REG_ANALOG_GAIN,
                       IMX519_REG_VALUE_16BIT, ctrl->val);
        break;
    case V4L2_CID_DIGITAL_GAIN:
        ret = imx519_write_reg(client, IMX519_REG_DIGITAL_GAIN,
                       IMX519_REG_VALUE_16BIT, ctrl->val);
        break;
    case V4L2_CID_VBLANK:
        vts = ctrl->val + imx519->cur_mode->height;
        imx519->cur_vts = vts;
        ret = imx519_write_reg(client, IMX519_REG_VTS,
                       IMX519_REG_VALUE_16BIT, vts);
        break;
    case V4L2_CID_TEST_PATTERN:
        ret = imx519_write_reg(client, IMX519_REG_TEST_PATTERN,
                       IMX519_REG_VALUE_16BIT,
                       imx519_test_pattern_val[ctrl->val]);
        break;
    case V4L2_CID_HFLIP:
    case V4L2_CID_VFLIP:
        ret = imx519_read_reg(client, IMX519_REG_ORIENTATION,
                      IMX519_REG_VALUE_08BIT, &val);
        if (ret)
            break;
        if (ctrl->id == V4L2_CID_HFLIP) {
            if (ctrl->val)
                val |= IMX519_MIRROR_BIT_MASK;
            else
                val &= ~IMX519_MIRROR_BIT_MASK;
        } else {
            if (ctrl->val)
                val |= IMX519_FLIP_BIT_MASK;
            else
                val &= ~IMX519_FLIP_BIT_MASK;
        }
        ret = imx519_write_reg(client, IMX519_REG_ORIENTATION,
                       IMX519_REG_VALUE_08BIT, val);
        break;
    default:
        dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
             __func__, ctrl->id, ctrl->val);
        break;
    }

    pm_runtime_put(&client->dev);

    return ret;
}

static const struct v4l2_ctrl_ops imx519_ctrl_ops = {
    .s_ctrl = imx519_set_ctrl,
};

static int imx519_initialize_controls(struct imx519 *imx519)
{
    const struct imx519_mode *mode;
    struct v4l2_ctrl_handler *handler;
    s64 exposure_max, vblank_def;
    u64 pixel_rate;
    u32 h_blank;
    int ret;
    u8 lanes = imx519->bus_cfg.bus.mipi_csi2.num_data_lanes;

    handler = &imx519->ctrl_handler;
    mode = imx519->cur_mode;
    ret = v4l2_ctrl_handler_init(handler, 10);
    if (ret)
        return ret;
    handler->lock = &imx519->mutex;

    imx519->link_freq = v4l2_ctrl_new_int_menu(handler, NULL,
                V4L2_CID_LINK_FREQ,
                ARRAY_SIZE(link_freq_items) - 1, 0,
                link_freq_items);
    v4l2_ctrl_s_ctrl(imx519->link_freq, mode->mipi_freq_idx);

    pixel_rate = (u32)link_freq_items[mode->mipi_freq_idx] /
        mode->bpp * 2 * lanes;
    imx519->pixel_rate = v4l2_ctrl_new_std(handler, NULL,
        V4L2_CID_PIXEL_RATE, 0, pixel_rate, 1, pixel_rate);

    h_blank = mode->hts_def - mode->width;
    imx519->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
                h_blank, h_blank, 1, h_blank);
    if (imx519->hblank)
        imx519->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

    vblank_def = mode->vts_def - mode->height;
    imx519->vblank = v4l2_ctrl_new_std(handler, &imx519_ctrl_ops,
                V4L2_CID_VBLANK, vblank_def,
                IMX519_VTS_MAX - mode->height,
                1, vblank_def);
    imx519->cur_vts = mode->vts_def;

    exposure_max = mode->vts_def - IMX519_EXPOSURE_OFFSET;
    imx519->exposure = v4l2_ctrl_new_std(handler, &imx519_ctrl_ops,
                V4L2_CID_EXPOSURE, IMX519_EXPOSURE_MIN,
                exposure_max, IMX519_EXPOSURE_STEP,
                mode->exp_def);

    imx519->anal_a_gain = v4l2_ctrl_new_std(handler, &imx519_ctrl_ops,
                V4L2_CID_ANALOGUE_GAIN, IMX519_ANA_GAIN_MIN,
                IMX519_ANA_GAIN_MAX, IMX519_ANA_GAIN_STEP,
                IMX519_ANA_GAIN_DEFAULT);

    imx519->digi_gain = v4l2_ctrl_new_std(handler, &imx519_ctrl_ops,
                V4L2_CID_DIGITAL_GAIN, IMX519_DGTL_GAIN_MIN,
                IMX519_DGTL_GAIN_MAX, IMX519_DGTL_GAIN_STEP,
                IMX519_DGTL_GAIN_DEFAULT);

    imx519->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
                &imx519_ctrl_ops, V4L2_CID_TEST_PATTERN,
                ARRAY_SIZE(imx519_test_pattern_menu) - 1,
                0, 0, imx519_test_pattern_menu);

    v4l2_ctrl_new_std(handler, &imx519_ctrl_ops, V4L2_CID_HFLIP, 0, 1, 1, 0);
    v4l2_ctrl_new_std(handler, &imx519_ctrl_ops, V4L2_CID_VFLIP, 0, 1, 1, 0);

    if (handler->error) {
        ret = handler->error;
        dev_err(&imx519->client->dev,
            "Failed to init controls(%d)\n", ret);
        goto err_free_handler;
    }

    imx519->subdev.ctrl_handler = handler;

    return 0;

err_free_handler:
    v4l2_ctrl_handler_free(handler);

    return ret;
}

static int imx519_check_sensor_id(struct imx519 *imx519,
                  struct i2c_client *client)
{
    struct device *dev = &imx519->client->dev;
    u32 id = 0;
    int ret;

    ret = imx519_read_reg(client, IMX519_REG_CHIP_ID,
                  IMX519_REG_VALUE_16BIT, &id);
    if (id != IMX519_CHIP_ID) {
        dev_err(dev, "Unexpected sensor id(%04x), ret(%d)\n", id, ret);
        return -ENODEV;
    }

    dev_info(dev, "Detected imx519 id %04x\n", IMX519_CHIP_ID);

    return 0;
}

static int imx519_probe(struct i2c_client *client,
            const struct i2c_device_id *id)
{
    struct device *dev = &client->dev;
    struct device_node *node = dev->of_node;
    struct imx519 *imx519;
    struct v4l2_subdev *sd;
    struct device_node *endpoint;
    char facing[2];
    int ret;

    dev_info(dev, "driver version: %02x.%02x.%02x",
         DRIVER_VERSION >> 16,
         (DRIVER_VERSION & 0xff00) >> 8,
         DRIVER_VERSION & 0x00ff);

    imx519 = devm_kzalloc(dev, sizeof(*imx519), GFP_KERNEL);
    if (!imx519)
        return -ENOMEM;

    ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
                   &imx519->module_index);
    ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
                       &imx519->module_facing);
    ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
                       &imx519->module_name);
    ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
                       &imx519->len_name);
    if (ret) {
        dev_err(dev, "could not get module information!\n");
        return -EINVAL;
    }

    endpoint = of_graph_get_next_endpoint(node, NULL);
    if (!endpoint) {
        dev_err(dev, "Failed to get endpoint\n");
        return -EINVAL;
    }

    ret = v4l2_fwnode_endpoint_parse(of_fwnode_handle(endpoint),
                     &imx519->bus_cfg);
    of_node_put(endpoint);
    if (ret) {
        dev_err(dev, "Failed to get bus config\n");
        return -EINVAL;
    }

    imx519->client = client;
    imx519->supported_modes = supported_modes_2lane;
    imx519->cfg_num = ARRAY_SIZE(supported_modes_2lane);
    imx519->cur_mode = &imx519->supported_modes[0];
    dev_info(dev, "detect imx519 lane %d\n",
         imx519->bus_cfg.bus.mipi_csi2.num_data_lanes);

    imx519->xvclk = devm_clk_get_optional(dev, "xvclk");
    if (IS_ERR(imx519->xvclk)) {
        dev_err(dev, "Failed to get xvclk\n");
        return PTR_ERR(imx519->xvclk);
    }

    imx519->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_ASIS);
    if (IS_ERR(imx519->reset_gpio))
        dev_warn(dev, "Failed to get reset-gpios\n");
    imx519->power_gpio = devm_gpiod_get(dev, "power", GPIOD_ASIS);
    if (IS_ERR(imx519->power_gpio))
        dev_warn(dev, "Failed to get power-gpios\n");

    imx519->pinctrl = devm_pinctrl_get(dev);
    if (!IS_ERR(imx519->pinctrl)) {
        imx519->pins_default =
            pinctrl_lookup_state(imx519->pinctrl,
                         OF_CAMERA_PINCTRL_STATE_DEFAULT);
        if (IS_ERR(imx519->pins_default))
            dev_info(dev, "could not get default pinstate\n");

        imx519->pins_sleep =
            pinctrl_lookup_state(imx519->pinctrl,
                         OF_CAMERA_PINCTRL_STATE_SLEEP);
        if (IS_ERR(imx519->pins_sleep))
            dev_info(dev, "could not get sleep pinstate\n");
    } else {
        dev_info(dev, "no pinctrl\n");
    }

    mutex_init(&imx519->mutex);

    sd = &imx519->subdev;
    v4l2_i2c_subdev_init(sd, client, &imx519_subdev_ops);
    ret = imx519_initialize_controls(imx519);
    if (ret)
        goto err_destroy_mutex;

    ret = __imx519_power_on(imx519);
    if (ret)
        goto err_free_handler;

    ret = imx519_check_sensor_id(imx519, client);
    if (ret)
        goto err_power_off;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
    sd->internal_ops = &imx519_internal_ops;
    sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
             V4L2_SUBDEV_FL_HAS_EVENTS;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
    imx519->pad.flags = MEDIA_PAD_FL_SOURCE;
    sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
    ret = media_entity_pads_init(&sd->entity, 1, &imx519->pad);
    if (ret < 0)
        goto err_power_off;
#endif

    memset(facing, 0, sizeof(facing));
    if (strcmp(imx519->module_facing, "back") == 0)
        facing[0] = 'b';
    else
        facing[0] = 'f';

    snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
         imx519->module_index, facing,
         IMX519_NAME, dev_name(sd->dev));
    ret = v4l2_async_register_subdev_sensor_common(sd);
    if (ret) {
        dev_err(dev, "v4l2 async register subdev failed\n");
        goto err_clean_entity;
    }

    pm_runtime_set_active(dev);
    pm_runtime_enable(dev);
    pm_runtime_idle(dev);

    return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
    media_entity_cleanup(&sd->entity);
#endif
err_power_off:
    __imx519_power_off(imx519);
err_free_handler:
    v4l2_ctrl_handler_free(&imx519->ctrl_handler);
err_destroy_mutex:
    mutex_destroy(&imx519->mutex);

    return ret;
}

static int imx519_remove(struct i2c_client *client)
{
    struct v4l2_subdev *sd = i2c_get_clientdata(client);
    struct imx519 *imx519 = to_imx519(sd);

    v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
    media_entity_cleanup(&sd->entity);
#endif
    v4l2_ctrl_handler_free(&imx519->ctrl_handler);
    mutex_destroy(&imx519->mutex);

    pm_runtime_disable(&client->dev);
    if (!pm_runtime_status_suspended(&client->dev))
        __imx519_power_off(imx519);
    pm_runtime_set_suspended(&client->dev);

    return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id imx519_of_match[] = {
    { .compatible = "sony,imx519" },
    {},
};
MODULE_DEVICE_TABLE(of, imx519_of_match);
#endif

static const struct i2c_device_id imx519_match_id[] = {
    { "sony,imx519", 0 },
    { },
};

static struct i2c_driver imx519_i2c_driver = {
    .driver = {
        .name = IMX519_NAME,
        .pm = &imx519_pm_ops,
        .of_match_table = of_match_ptr(imx519_of_match),
    },
    .probe        = &imx519_probe,
    .remove        = &imx519_remove,
    .id_table    = imx519_match_id,
};

static int __init sensor_mod_init(void)
{
    return i2c_add_driver(&imx519_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
    i2c_del_driver(&imx519_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("Sony imx519 sensor driver");
MODULE_LICENSE("GPL v2");
