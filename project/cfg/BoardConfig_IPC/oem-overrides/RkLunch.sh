#!/bin/sh

rcS() {
    for i in /oem/usr/etc/init.d/S??*; do
        [ ! -f "$i" ] && continue
        case "$i" in
        *.sh)
            (
                trap - INT QUIT TSTP
                set start
                . $i
            )
            ;;
        *)
            $i start
            ;;
        esac
    done
}

check_linker() {
    [ ! -L "$2" ] && ln -sf $1 $2
}

network_init() {
    ethaddr1=$(ifconfig -a | grep "eth.*HWaddr" | awk '{print $5}')

    if [ -f /data/ethaddr.txt ]; then
        ethaddr2=$(cat /data/ethaddr.txt)
        if [ $ethaddr1 == $ethaddr2 ]; then
            echo "eth HWaddr cfg ok"
        else
            ifconfig eth0 down
            ifconfig eth0 hw ether $ethaddr2
        fi
    else
        echo $ethaddr1 >/data/ethaddr.txt
    fi
    ifconfig eth0 up && udhcpc -i eth0 >/dev/null 2>&1
}

post_chk() {
    cnt=0
    while [ $cnt -lt 30 ]; do
        cnt=$((cnt + 1))
        if mount | grep -w userdata; then
            break
        fi
        sleep .1
    done

    # Load kernel modules
    default_ko_dir=/ko
    if [ -f "/oem/usr/ko/insmod_ko.sh" ]; then
        default_ko_dir=/oem/usr/ko
    fi
    if [ -f "$default_ko_dir/insmod_ko.sh" ]; then
        cd $default_ko_dir && sh insmod_ko.sh && cd -
    fi

    network_init &

    # --- rkipc camera daemon (disabled, re-enable when camera is integrated) ---
    # check_linker /userdata /oem/usr/www/userdata
    # check_linker /media/usb0 /oem/usr/www/usb0
    # check_linker /mnt/sdcard /oem/usr/www/sdcard
    #
    # rkipc_ini=/userdata/rkipc.ini
    # default_rkipc_ini=/tmp/rkipc-factory-config.ini
    #
    # if [ ! -f "/oem/usr/share/rkipc.ini" ]; then
    #     # Auto-detect sensor and link config
    #     lsmod | grep imx415 && ln -s -f /oem/usr/share/rkipc-800w.ini $default_rkipc_ini
    #     lsmod | grep sc3336 && ln -s -f /oem/usr/share/rkipc-300w.ini $default_rkipc_ini
    #     lsmod | grep sc4336 && ln -s -f /oem/usr/share/rkipc-400w.ini $default_rkipc_ini
    #     lsmod | grep sc530ai && ln -s -f /oem/usr/share/rkipc-500w.ini $default_rkipc_ini
    #     lsmod | grep mis5001 && ln -s -f /oem/usr/share/rkipc-mis5001-500w.ini $default_rkipc_ini
    #     lsmod | grep mia1321 && ln -s -f /oem/usr/share/rkipc-mia1321-100w.ini $default_rkipc_ini
    # fi
    #
    # if [ ! -f "$default_rkipc_ini" ]; then
    #     echo "Error: not found rkipc.ini !!!"
    #     return
    # fi
    # cp $default_rkipc_ini $rkipc_ini -f
    #
    # if [ -d "/oem/usr/share/iqfiles" ]; then
    #     rkipc -a /oem/usr/share/iqfiles &
    # else
    #     rkipc &
    # fi
}

rcS

ulimit -c unlimited
echo "/data/core-%p-%e" >/proc/sys/kernel/core_pattern
echo 1 >/proc/sys/vm/overcommit_memory

post_chk &