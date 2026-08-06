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

    # Wipe /data too: sd_update.img only covers env..rootfs, so the growup data
    # partition survives a plain reflash. No on-disk partition table exists
    # (Rockchip uses the kernel blkdevparts= cmdline), so target it by byte
    # offset from RK_PARTITION_CMD_IN_ENV:
    #   32K env +512K idblock +256K uboot +32M boot +512M oem +6G rootfs = data@
    DATA_OFFSET=7013695488
    magic=$(sudo dd if=/dev/mmcblk0 bs=1 skip=$((DATA_OFFSET + 1080)) count=2 2>/dev/null | xxd -p)
    label=$(sudo dd if=/dev/mmcblk0 bs=1 skip=$((DATA_OFFSET + 1144)) count=16 2>/dev/null | tr -d '\0')
    if [ "$magic" = "53ef" ] && [ "$label" = "data" ]; then
        echo "Wiping /data (ext4 'data' @ $DATA_OFFSET)..."
        sudo dd if=/dev/zero of=/dev/mmcblk0 bs=512 seek=$((DATA_OFFSET / 512)) count=32768 conv=fsync status=none
        sync
        echo "/data wiped (16 MiB zeroed: superblock, descriptors, journal)."
    else
        echo "No ext4 'data' fs at $DATA_OFFSET (magic=$magic label='$label'); skipping /data wipe."
    fi
    echo "Done."

picocom-logs:
    picocom -b 115200 /dev/ttyACM0 | tee logs/rv1106-boot-$(date +%Y-%m-%d_%H:%M:%S).log

clean:
    rm -rf sysdrv/source/buildroot/buildroot-2023.02.6 output sysdrv/out sysdrv/source/objs_kernel
    rm -f sysdrv/source/kernel/scripts/kconfig/conf sysdrv/source/kernel/scripts/kconfig/*conf
    @if [ -f sysdrv/source/kernel/.config ]; then make -C sysdrv/source/kernel distclean 2>/dev/null || true; fi
    @if [ -f sysdrv/source/uboot/u-boot/.config ]; then make -C sysdrv/source/uboot/u-boot distclean 2>/dev/null || true; fi
    @echo "Cleaned all build directories"

# -r so a run directory (or /data/latest, which is a symlink to one) works too
pull filepath:
    scp -r root@172.32.0.1:{{filepath}} .

# Mux a raw HEVC elementary stream into an mp4. The stream carries no timestamps
# or framerate, so force 30fps on both input and output
mux src="latest/video.h265" dst="latest/video.mp4":
    ffmpeg -y -r 30 -i "{{src}}" -c copy -r 30 -video_track_timescale 30000 "{{dst}}"

# Copy a file to the board, flush it to the SD card and verify the on-disk copy
# matches the host. Drop caches before reading back so the checksum reflects
# what actually landed on flash, not the page cache (which passes even when the
# on-disk bytes are still unflushed and a power cut would leave nulls).
push src dst board="172.32.0.1":
    #!/usr/bin/env bash
    set -euo pipefail
    scp "{{src}}" root@{{board}}:"{{dst}}"
    want=$(sha256sum "{{src}}" | cut -d' ' -f1)
    got=$(ssh root@{{board}} "sync && echo 3 > /proc/sys/vm/drop_caches; sha256sum '{{dst}}'" | cut -d' ' -f1)
    if [ "$want" != "$got" ]; then
        echo "ERROR: checksum mismatch for {{dst}} (host $want != board $got) - transfer corrupt" >&2
        exit 1
    fi
    echo "pushed {{src}} -> {{dst}} ($want)"

# Copy the SoC supervisor to the board, restart it and show any startup errors
run-supervisor board="172.32.0.1" dest="/usr/bin/uvr_supervisor.py":
    #!/usr/bin/env bash
    set -euo pipefail
    ssh root@{{board}} "/etc/init.d/S99uvr stop"
    just push symlinks/supervisor.py {{dest}} {{board}}
    just push symlinks/S99uvr /etc/init.d/S99uvr {{board}}
    ssh root@{{board}} "$(printf '%s\n' \
        'chmod +x /etc/init.d/S99uvr' \
        '/etc/init.d/S99uvr start' \
        'sleep 3' \
        'pidof python3 >/dev/null && echo SUPERVISOR_RUNNING || echo SUPERVISOR_DOWN' \
        'echo "--- /tmp/supervisor-startup.log ---"' \
        'cat /tmp/supervisor-startup.log 2>/dev/null' \
        'echo "--- /data/latest/supervisor.log ---"' \
        'cat /data/latest/supervisor.log 2>/dev/null')"

copy-supervisor:
    just push symlinks/supervisor.py /usr/bin/uvr_supervisor.py

copy-iq:
    just push symlinks/imx519_arducam-imx519_default.json /etc/iqfiles/imx519_arducam-imx519_default.json
    just push symlinks/imx519_arducam-imx519_default.json /oem/usr/share/iqfiles/imx519_arducam-imx519_default.json

# Build the uvr-vicap capture binary in the container and copy it to the board
copy-vicap dest="/oem/usr/bin/":
    #!/usr/bin/env bash
    set -euo pipefail
    rm -f media/samples/simple_test/uvr-vicap
    just docker-run "make -C media/samples/simple_test uvr-vicap RK_MEDIA_CROSS=arm-rockchip830-linux-uclibcgnueabihf RK_CHIP=rv1106"
    just push media/samples/simple_test/uvr-vicap {{dest}}/uvr-vicap

copy-latest:
    rm latest/* | continue
    just pull /data/startup.log && mv startup.log latest
    just pull /data/recordings.log && mv recordings.log latest
    just pull /data/latest/
    just mux

# data-journal-info:
#     DEV=/dev/mmcblk0
#     OFF=7013695488
#     LOOP=$(sudo losetup -f --show -r -o 7013695488 "$DEV")
#     echo "loop=$LOOP"
#     echo "=== superblock ==="
#     sudo dumpe2fs -h "$LOOP" 2>/dev/null | grep -Ei 'Filesystem state|feature|Mount count|Last (mount|write)|Lifetime'
#     echo "=== journal (empty == cleanly unmounted) ==="
#     sudo debugfs -R "logdump -S" "$LOOP" 2>/dev/null | head -50
#     sudo losetup -d "$LOOP"

restart-rtsp board="172.32.0.1":
    #!/usr/bin/env bash
    set -euo pipefail
    streamer="simple_vi_bind_venc_rtsp -I 0 -w 1920 -h 1080 -e h265"
    proc="simple_vi_bind_venc_rtsp"
    just copy-iq {{board}}
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
