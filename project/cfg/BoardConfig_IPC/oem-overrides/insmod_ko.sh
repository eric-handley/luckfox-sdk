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

# --- DVBM buffer manager (ISP <-> VENC online path; needed by mpp_vcodec) ---
__insmod rk_dvbm.ko

# --- Camera sensor (step 1: probe only, ISP/CIF pipeline still disabled) ---
__insmod imx519.ko

# --- Camera sensor modules (disabled, re-enable when camera is integrated) ---
# videobuf2-* are built-in (CONFIG_VIDEOBUF2_*=y), no insmod needed
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

# --- CSI receiver pipeline (step 2a: D-PHY + CIF) + ISP (step 3) ---
__insmod phy-rockchip-csi2-dphy-hw.ko
__insmod phy-rockchip-csi2-dphy.ko
__insmod video_rkcif.ko
echo 1 > /sys/module/video_rkcif/parameters/clr_unready_dev
__insmod video_rkisp.ko
echo 1 > /sys/module/video_rkisp/parameters/clr_unready_dev

# --- Hardware acceleration ---
__insmod rga3.ko
__insmod mpp_vcodec.ko
__insmod rknpu.ko

# --- Audio ---
__insmod snd-soc-rv1106.ko

# --- Motor (disabled, not needed) ---
# __insmod motor.ko

# --- Rockit media framework (step 4: userspace capture + VENC) ---
__insmod rockit.ko mcu_fw_path="./hpmcu_wrap.bin" mcu_fw_addr=0xff6fe000 isp_max_h=1080

# --- RVE (disabled, not needed) ---
# __insmod rve.ko

udevadm control --start-exec-queue

# --- WiFi (disabled, no wifi on this board) ---
# $(pwd)/insmod_wifi.sh &