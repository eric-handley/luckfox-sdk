#!/bin/sh
cmd=`realpath $0`
_DIR=`dirname $cmd`
cd $_DIR

udevadm control --stop-exec-queue

__insmod()
{
    if [ -f "$1" ];then
        insmod $@
    fi
}

# --- Camera sensor modules (disabled, re-enable when camera is integrated) ---
# __insmod rk_dvbm.ko
# __insmod videobuf2-memops.ko
# __insmod videobuf2-common.ko
# __insmod videobuf2-v4l2.ko
# __insmod videobuf2-vmalloc.ko
# __insmod videobuf2-cma-sg.ko
# __insmod imx415.ko
# __insmod os04a10.ko
# __insmod sc4336.ko
# __insmod sc3336.ko
# __insmod sc530ai.ko
# __insmod gc2053.ko
# __insmod sc200ai.ko
# __insmod sc401ai.ko
# __insmod sc450ai.ko
# __insmod techpoint.ko
# __insmod mis5001.ko
# __insmod mia1321.ko

# --- ISP/CIF pipeline (disabled, re-enable when camera is integrated) ---
# __insmod video_rkcif.ko
# __insmod video_rkisp.ko
# __insmod phy-rockchip-csi2-dphy-hw.ko
# __insmod phy-rockchip-csi2-dphy.ko
# echo 1 > /sys/module/video_rkcif/parameters/clr_unready_dev
# echo 1 > /sys/module/video_rkisp/parameters/clr_unready_dev

# --- Hardware acceleration ---
__insmod rga3.ko
# mpp_vcodec depends on rk_dvbm (disabled above) — re-enable both when camera is integrated
# __insmod mpp_vcodec.ko
__insmod rknpu.ko

# --- Audio ---
__insmod snd-soc-rv1106.ko

# --- Motor (disabled, not needed) ---
# __insmod motor.ko

# --- Rockit media framework (disabled, depends on ISP/CIF) ---
# __insmod rockit.ko mcu_fw_path="./hpmcu_wrap.bin" mcu_fw_addr=0xff6fe000 isp_max_h=$sensor_height

# --- RVE (disabled, not needed) ---
# __insmod rve.ko

udevadm control --start-exec-queue

# --- WiFi (disabled, no wifi on this board) ---
# $(pwd)/insmod_wifi.sh &