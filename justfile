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

    # Reformat /data. sd_update.img only covers env..rootfs, so the growup data
    # partition keeps its old (and, after power cuts, damaged) contents across a
    # reflash. There is no on-disk partition table -- Rockchip defines partitions
    # via the kernel blkdevparts= cmdline -- so we address partitions by their byte
    # offsets from RK_PARTITION_CMD_IN_ENV:
    #   32K env +512K idblock +256K uboot +32M boot +512M oem +6G rootfs = data@
    # data is the '-' growup partition, running from its offset to the end of the
    # card, which is exactly what a loop device with -o OFFSET and no size limit
    # covers. Format it here into a known-clean ext4 instead of leaving the board
    # to detect damage and reformat itself on boot.
    ROOTFS_OFFSET=571244544
    DATA_OFFSET=7013695488

    # Safety: these offsets are only valid for the current partition layout. rootfs
    # was just written fresh by the dd above, so its superblock is definitely valid;
    # if the ext4 magic isn't where we expect it, the layout changed and every
    # offset below (including data's) is wrong -- refuse rather than format blind.
    rootfs_magic=$(sudo dd if=/dev/mmcblk0 bs=1 skip=$((ROOTFS_OFFSET + 1080)) count=2 2>/dev/null | xxd -p)
    if [ "$rootfs_magic" != "53ef" ]; then
        echo "ERROR: no ext4 superblock at expected rootfs offset $ROOTFS_OFFSET" \
             "(magic=$rootfs_magic) -- partition layout changed, refusing to format" \
             "/data at a now-unknown offset." >&2
        exit 1
    fi

    echo "Reformatting /data (ext4 @ $DATA_OFFSET)..."
    LOOP=$(sudo losetup -f --show -o $DATA_OFFSET /dev/mmcblk0)
    trap 'sudo losetup -d "$LOOP" 2>/dev/null || true' EXIT
    # Match S21uvrdata's on-device format so a host reflash and a from-blank board
    # boot produce the same filesystem.
    #   ^metadata_csum: this board's kernel mishandles ext4 directory-leaf checksums
    #     (logs "No space for directory leaf checksum" then EBADMSGs every write to a
    #     full directory), so the feature is off.
    #   ^orphan_file: the host's e2fsprogs (1.47+) enables this by default, but the
    #     board's mkfs/e2fsck are 1.46.5 and don't understand it -- leaving it on
    #     makes the board's boot-time e2fsck bail ("unsupported feature FEATURE_C12")
    #     so /data can never be checked after a power cut. Off = 1.46.5-compatible.
    #   lazy_*_init=0: write all inode-table and journal metadata now rather than
    #     from a post-mount background thread that a power cut could interrupt.
    sudo mkfs.ext4 -F -L data -O ^metadata_csum,^orphan_file -E lazy_itable_init=0,lazy_journal_init=0 "$LOOP"
    sudo tune2fs -c 0 -i 0 "$LOOP"
    sudo losetup -d "$LOOP"
    trap - EXIT
    sync
    echo "/data reformatted clean."
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

# Mux a raw HEVC elementary stream into an mp4, folding in the matching audio
# sidecar if present. The stream carries no timestamps or framerate, so force
# 60fps on both input and output. The mic lands on the codec's LEFT ADC, so take
# the left channel (c0) explicitly -> mono AAC. Missing audio falls back to a
# video-only mux so a recording is never lost.
mux src="latest/video.h265" dst="latest/video.mp4":
    #!/usr/bin/env bash
    set -euo pipefail
    src="{{src}}"; dst="{{dst}}"
    dir="${src%/*}"
    base="${src##*/}"          # video.h265 or video.1.h265
    mid="${base#video}"        # .h265 or .1.h265
    tag="${mid%.h265}"         # "" or ".1"
    audio="$dir/audio$tag.wav"
    # Video-only mux, used both when no audio sidecar exists and as a fallback
    # when the audio is present but unusable (corrupt/short wav -> ffmpeg errors).
    video_only() { ffmpeg -y -r 60 -i "$src" -c copy -r 60 -video_track_timescale 60000 "$dst"; }
    if [ -f "$audio" ]; then
        # Align by the real capture-start stamps each stream logs (SYNC_START_US=
        # <CLOCK_MONOTONIC us>): vicap at its first encoded frame, the supervisor
        # at tinycap launch. Shift the audio so its start maps to the video start
        # (adelay if it began later, -ss trim if earlier), pad the tail, and let
        # -shortest cut the output to the video length. No fps/duration math.
        vlog="$dir/vicap$tag.log"; alog="$dir/audio$tag.log"
        vs=$(grep -oE 'SYNC_START_US=[0-9]+' "$vlog" 2>/dev/null | head -1 | cut -d= -f2)
        as=$(grep -oE 'SYNC_START_US=[0-9]+' "$alog" 2>/dev/null | head -1 | cut -d= -f2)
        # Fixed residual after stamp alignment (encoder pipeline latency on the
        # video anchor + ALSA-open latency on the audio anchor). Positive delays
        # the audio; raise if audio is still early, lower if it lags.
        av_skew_ms=190
        ss=0; pre=""
        if [ -n "$vs" ] && [ -n "$as" ]; then
            off=$((as - vs + av_skew_ms*1000))   # audio start vs video start, us
            if [ "$off" -ge 0 ]; then
                pre=",adelay=$(awk "BEGIN{print $off/1000}")"   # audio later -> delay (ms)
            else
                ss=$(awk "BEGIN{print -($off)/1000000}")        # audio earlier -> trim (s)
            fi
        fi
        # Cap the output to the true video length (60fps is exact -> packets/60)
        # and apad-fill the audio tail. -shortest can't be used here: the copied
        # raw h265 carries no timestamps, so it reads as zero-length and cuts the
        # audio to nothing.
        vpkts=$(ffprobe -v error -select_streams v:0 -count_packets \
            -show_entries stream=nb_read_packets -of csv=p=0 "$src" 2>/dev/null || echo "")
        if [ -n "$vpkts" ]; then
            ffmpeg -y -r 60 -i "$src" -ss "$ss" -i "$audio" -c:v copy \
                -af "pan=mono|c0=c0${pre},apad" -c:a aac -t "$(awk "BEGIN{print $vpkts/60}")" \
                -r 60 -video_track_timescale 60000 "$dst" || video_only
        else
            ffmpeg -y -r 60 -i "$src" -ss "$ss" -i "$audio" -c:v copy \
                -af "pan=mono|c0=c0${pre}" -c:a aac \
                -r 60 -video_track_timescale 60000 "$dst" || video_only
        fi
    else
        video_only
    fi

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
