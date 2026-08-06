#!/bin/bash
#
# Post-build rootfs fixups, run against the staged rootfs before it is packed
# into rootfs.img (build.sh: __RUN_POST_BUILD_SCRIPT, just before post_overlay).
#
# The Luckfox base rootfs ships /data as a symlink to userdata (their stock
# layout mounts a "userdata" partition at /userdata). This board has no userdata
# partition: the recording partition is named "data" and S21uvrdata mounts it at
# /data. With a read-only rootfs we cannot collapse that symlink at runtime, so
# bake /data as a real, empty mountpoint here and drop the unused userdata dir.

set -e

ROOTFS="$RK_PROJECT_PACKAGE_ROOTFS_DIR"
[ -n "$ROOTFS" ] && [ -d "$ROOTFS" ] || { echo "post: no rootfs dir, skipping"; exit 0; }

# Replace the /data -> userdata symlink with a real mountpoint directory.
if [ -L "$ROOTFS/data" ] || [ ! -d "$ROOTFS/data" ]; then
    echo "post: making /data a real mountpoint directory"
    rm -rf "$ROOTFS/data"
    mkdir -p "$ROOTFS/data"
    chmod 755 "$ROOTFS/data"
fi

# Unused on this board; nothing ever mounts /userdata.
rm -rf "$ROOTFS/userdata"
