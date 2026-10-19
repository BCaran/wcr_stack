#!/bin/bash
#
# wcr_container - manage the WCR Humble Docker container
#
# Usage:
#   wcr_container build   Build the Docker image from the Dockerfile
#   wcr_container up      Start the container (in the background)
#   wcr_container shell   Open a bash shell inside the running container
#   wcr_container down    Stop and remove the container
#   wcr_container status  Show image/container status
#
# Real-time optimizations for ros2_control at 500Hz.
# Support for Dynamixel, Intel RealSense T265 and OpenVINS.

set -e

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
IMAGE_NAME="wcr_humble_image"
CONTAINER_NAME="wcr_container"
DOCKERFILE="Dockerfile"
USB_DEVICE="/dev/ttyUSB0"
CPUSET="2,3"

# Folder containing the Dockerfile. Defaults to the folder this script lives in
# (symlinks are resolved), can be overridden: WCR_DOCKER_DIR=/path wcr_container build
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
DOCKER_DIR="${WCR_DOCKER_DIR:-$SCRIPT_DIR}"

WS_ROOT="/home/wcr/dck_wcr_humble"

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
info()  { echo -e "\e[32m[wcr]\e[0m $*"; }
warn()  { echo -e "\e[33m[wcr]\e[0m $*"; }
error() { echo -e "\e[31m[wcr]\e[0m $*" >&2; }

container_exists()  { [ -n "$(docker ps -aq -f name=^/${CONTAINER_NAME}$)" ]; }
container_running() { [ -n "$(docker ps -q  -f name=^/${CONTAINER_NAME}$)" ]; }
image_exists()      { docker image inspect "${IMAGE_NAME}" > /dev/null 2>&1; }

usage() {
    cat <<EOF
Usage: $(basename "$0") <command>

Commands:
  build    Build the Docker image (${IMAGE_NAME}) from ${DOCKER_DIR}/${DOCKERFILE}
  up       Start the container (${CONTAINER_NAME}) in the background
  shell    Open a bash shell inside the running container
  down     Stop and remove the container
  status   Show image and container status
EOF
}

# ---------------------------------------------------------------------------
# Commands
# ---------------------------------------------------------------------------
cmd_build() {
    if [ ! -f "${DOCKER_DIR}/${DOCKERFILE}" ]; then
        error "Dockerfile not found: ${DOCKER_DIR}/${DOCKERFILE}"
        error "Set WCR_DOCKER_DIR to the folder containing your Dockerfile."
        exit 1
    fi

    info "Building image '${IMAGE_NAME}' from ${DOCKER_DIR}/${DOCKERFILE}"
    docker build \
        -t "${IMAGE_NAME}" \
        -f "${DOCKER_DIR}/${DOCKERFILE}" \
        --build-arg USERNAME=wcr_docker \
        --build-arg USER_UID="$(id -u)" \
        --build-arg USER_GID="$(id -g)" \
        "${DOCKER_DIR}"
    info "Build finished."
}

cmd_up() {
    if ! image_exists; then
        error "Image '${IMAGE_NAME}' not found. Run: $(basename "$0") build"
        exit 1
    fi

    if container_running; then
        warn "Container '${CONTAINER_NAME}' is already running. Use: $(basename "$0") shell"
        exit 0
    fi

    # Set USB latency timer to 1ms for the Dynamixel USB adapter (host side)
    local dev
    dev="$(basename "${USB_DEVICE}")"
    if [ -f "/sys/bus/usb-serial/devices/${dev}/latency_timer" ]; then
        echo 1 | sudo tee "/sys/bus/usb-serial/devices/${dev}/latency_timer" > /dev/null
        info "USB latency timer for ${dev} set to 1ms"
    fi

    # Only pass --device if the adapter is plugged in (otherwise docker run fails)
    local device_args=()
    if [ -e "${USB_DEVICE}" ]; then
        device_args=(--device="${USB_DEVICE}")
    else
        warn "${USB_DEVICE} not found - starting without Dynamixel adapter."
    fi

    # Remove a stopped leftover container with the same name
    if container_exists; then
        docker rm -f "${CONTAINER_NAME}" > /dev/null
    fi

    info "Starting container '${CONTAINER_NAME}'"
    docker run -dit \
        --name "${CONTAINER_NAME}" \
        --net=host \
        --ipc=host \
        --pid=host \
        --privileged \
        "${device_args[@]}" \
        --group-add dialout \
        --cap-add=sys_nice \
        --ulimit rtprio=99 \
        --ulimit memlock=-1 \
        --cpuset-cpus="${CPUSET}" \
        -e DISPLAY="${DISPLAY}" \
        -e QT_X11_NO_MITSHM=1 \
        -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v /dev:/dev \
        -v "${WS_ROOT}/dxl_ws:/dxl_ws" \
        -v "${WS_ROOT}/vins_ws:/vins_ws" \
        "${IMAGE_NAME}" > /dev/null

    info "Container is up. Enter it with: $(basename "$0") shell"
}

cmd_shell() {
    if ! container_running; then
        if container_exists; then
            info "Container is stopped - starting it again."
            docker start "${CONTAINER_NAME}" > /dev/null
        else
            error "Container '${CONTAINER_NAME}' does not exist. Run: $(basename "$0") up"
            exit 1
        fi
    fi
    docker exec -it "${CONTAINER_NAME}" /bin/bash
}

cmd_down() {
    if container_exists; then
        info "Stopping and removing container '${CONTAINER_NAME}'"
        docker rm -f "${CONTAINER_NAME}" > /dev/null
        info "Container is down."
    else
        warn "Container '${CONTAINER_NAME}' is not running."
    fi
}

cmd_status() {
    if image_exists; then
        info "Image '${IMAGE_NAME}': present"
    else
        warn "Image '${IMAGE_NAME}': not built"
    fi

    if container_running; then
        info "Container '${CONTAINER_NAME}': running"
    elif container_exists; then
        warn "Container '${CONTAINER_NAME}': stopped"
    else
        warn "Container '${CONTAINER_NAME}': not created"
    fi
}

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
case "$1" in
    build)          cmd_build ;;
    up)             cmd_up ;;
    shell)          cmd_shell ;;
    down)           cmd_down ;;
    status)         cmd_status ;;
    -h|--help|help) usage ;;
    "")             usage; exit 1 ;;
    *)              error "Unknown command: $1"; usage; exit 1 ;;
esac
