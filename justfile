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
    docker run --rm -v "$(pwd):/workspace" --tmpfs /tmp:exec -u $(id -u):$(id -g) luckfox-sdk-builder ./build.sh | tee logs/docker-build-$(date +%Y-%m-%d_%H:%M:%S).log

clean:
    rm -rf sysdrv/source/buildroot/buildroot-2023.02.6 output sysdrv/out sysdrv/source/objs_kernel
    rm -f sysdrv/source/kernel/scripts/kconfig/conf sysdrv/source/kernel/scripts/kconfig/*conf
    @if [ -f sysdrv/source/kernel/.config ]; then make -C sysdrv/source/kernel distclean 2>/dev/null || true; fi
    @if [ -f sysdrv/source/uboot/u-boot/.config ]; then make -C sysdrv/source/uboot/u-boot distclean 2>/dev/null || true; fi
    @echo "Cleaned all build directories"