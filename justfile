default: config build

# Build Docker image
docker-build:
    docker build -t luckfox-sdk-builder .

# Run command in Docker container
docker-run CMD:
    docker run --rm -v "$(pwd):/workspace" -u $(id -u):$(id -g) luckfox-sdk-builder {{CMD}}

config:
    @echo -e "9\n10" | docker run --rm -v "$(pwd):/workspace" -u $(id -u):$(id -g) luckfox-sdk-builder ./build.sh lunch

build:
    docker run --rm -v "$(pwd):/workspace" -u $(id -u):$(id -g) luckfox-sdk-builder ./build.sh