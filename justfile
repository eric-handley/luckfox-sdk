default: config build

# Build Docker image
docker-build:
    docker build -t luckfox-sdk-builder .

# Run command in Docker container
docker-run CMD:
    docker run --rm -v "$(pwd):/workspace" -u $(id -u):$(id -g) luckfox-sdk-builder {{CMD}}

config:
    docker run --rm -v "$(pwd):/workspace" --tmpfs /tmp:exec -u $(id -u):$(id -g) luckfox-sdk-builder bash -c "echo -e '9\n10' | ./build.sh lunch"

build:
    mkdir -p logs
    rm -rf output/out/rootfs_uclibc_rv1106 sysdrv/out/rootfs_uclibc_rv1106 sysdrv/out/rootfs_uclibc_rv1106.tar
    docker run --rm -v "$(pwd):/workspace" --tmpfs /tmp:exec -u $(id -u):$(id -g) luckfox-sdk-builder ./build.sh | tee logs/docker-build-$(date +%Y-%m-%d_%H:%M:%S).log

# Build the uvr-vicap capture binary in the container and copy it to the board
vicap board="172.32.0.1" dest="":
    #!/usr/bin/env bash
    set -euo pipefail
    rm -f media/samples/simple_test/uvr-vicap
    just docker-run "make -C media/samples/simple_test uvr-vicap RK_MEDIA_CROSS=arm-rockchip830-linux-uclibcgnueabihf RK_CHIP=rv1106"
    bin=media/samples/simple_test/uvr-vicap
    want=$(md5sum "$bin" | cut -d' ' -f1)
    scp "$bin" root@{{board}}:{{dest}}/uvr-vicap
    # flush to the SD card, then verify the on-disk copy matches the host build
    got=$(ssh root@{{board}} "sync && md5sum {{dest}}/uvr-vicap" | cut -d' ' -f1)
    if [ "$want" != "$got" ]; then
        echo "ERROR: checksum mismatch (host $want != board $got) - transfer corrupt" >&2
        exit 1
    fi
    echo "uvr-vicap copied and verified ($want) at {{dest}}/uvr-vicap"

flash device="/dev/mmcblk0":
    #!/usr/bin/env bash
    set -euo pipefail

    # Check if device exists first
    if [ ! -e "{{device}}" ]; then
        echo "Error: {{device}} does not exist"
        exit 1
    fi

    # Check if it's a block device
    if [ ! -b "{{device}}" ]; then
        echo "Error: {{device}} is not a block device"
        exit 1
    fi

    echo "Flashing sd_update.img to /dev/mmcblk0..."
    sudo dd if=output/image/sd_update.img of=/dev/mmcblk0 bs=1M status=progress conv=fsync oflag=direct
    echo "Flash complete! Syncing..."
    sync
    echo "Done." 

picocom-logs:
    picocom -b 115200 /dev/ttyACM0 | tee logs/rv1106-boot-$(date +%Y-%m-%d_%H:%M:%S).log

clean:
    rm -rf sysdrv/source/buildroot/buildroot-2023.02.6 output sysdrv/out sysdrv/source/objs_kernel
    rm -f sysdrv/source/kernel/scripts/kconfig/conf sysdrv/source/kernel/scripts/kconfig/*conf
    @if [ -f sysdrv/source/kernel/.config ]; then make -C sysdrv/source/kernel distclean 2>/dev/null || true; fi
    @if [ -f sysdrv/source/uboot/u-boot/.config ]; then make -C sysdrv/source/uboot/u-boot distclean 2>/dev/null || true; fi
    @echo "Cleaned all build directories"

pull filepath:
    scp root@172.32.0.1:{{filepath}} .

copy-iq board="172.32.0.1":
    scp symlinks/imx519_arducam-imx519_default.json root@{{board}}:/etc/iqfiles/imx519_arducam-imx519_default.json
    scp symlinks/imx519_arducam-imx519_default.json root@{{board}}:/oem/usr/share/iqfiles/imx519_arducam-imx519_default.json

# Web UI to edit the IQ JSON and deploy it to the board (scp + restart streamer)
tuner board="172.32.0.1" port="8099":
    BOARD={{board}} PORT={{port}} node ../iq-tuner/server.js

deploy board="172.32.0.1" deploy="true":
    #!/usr/bin/env bash
    set -euo pipefail
    streamer="simple_vi_bind_venc_rtsp -I 0 -w 1920 -h 1080 -e h265"
    proc="simple_vi_bind_venc_rtsp"
    if [ "{{deploy}}" = "true" ]; then
        scp symlinks/imx519_arducam-imx519_default.json root@{{board}}:/etc/iqfiles/imx519_arducam-imx519_default.json
        scp symlinks/imx519_arducam-imx519_default.json root@{{board}}:/oem/usr/share/iqfiles/imx519_arducam-imx519_default.json
    else
        echo "deploy=false: skipping IQ JSON copy"
    fi
    # newlines (not "; ") so the "nohup ... &" line isn't followed by ";" (busybox sh)
    ssh root@{{board}} "$(printf '%s\n' \
        '. /etc/profile >/dev/null 2>&1 || true' \
        "killall $proc 2>/dev/null || true" \
        'sleep 1' \
        "nohup $streamer >/tmp/streamer.log 2>&1 &" \
        'sleep 2' \
        "pidof $proc && echo STREAMER_RUNNING || echo STREAMER_DOWN" \
        'echo "--- /tmp/streamer.log tail ---"' \
        'tail -n 20 /tmp/streamer.log 2>/dev/null')"