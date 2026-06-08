#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
IMAGE_NAME="${AGV_GALACTIC_IMAGE:-agv-sim-galactic:latest}"
CONTAINER_NAME="${AGV_GALACTIC_CONTAINER:-agv-sim-galactic}"
BASE_IMAGE="${AGV_GALACTIC_BASE_IMAGE:-osrf/ros:galactic-desktop}"
DOCKERFILE="${AGV_GALACTIC_DOCKERFILE:-${WS_DIR}/galactic/docker/Dockerfile.galactic}"

usage() {
  cat <<EOF
Usage:
  $0 build
  $0 shell
  $0 gui
  $0 sim [launch args...]
  $0 root
  $0 clean

Environment:
  AGV_GALACTIC_IMAGE      Docker image name. Default: agv-sim-galactic:latest
  AGV_GALACTIC_CONTAINER  Docker container name. Default: agv-sim-galactic
  AGV_GALACTIC_BASE_IMAGE Docker base image. Default: osrf/ros:galactic-desktop
  AGV_GALACTIC_DOCKERFILE Dockerfile path. Default: galactic/docker/Dockerfile.galactic
  AGV_GALACTIC_DOCKER_ARGS Extra docker run args.

Examples:
  $0 build
  AGV_GALACTIC_BASE_IMAGE=ros:galactic-ros-base $0 build
  AGV_GALACTIC_BASE_IMAGE=dockerproxy.net/osrf/ros:galactic-desktop $0 build
  AGV_GALACTIC_DOCKERFILE=galactic/docker/Dockerfile.galactic-apt $0 build
  $0 shell
  $0 gui
  $0 sim use_rviz:=false
  colcon build --symlink-install
EOF
}

docker_build() {
  if ! command -v docker >/dev/null 2>&1; then
    cat >&2 <<EOF
docker command not found.

Install Docker on the host first:

  sudo apt update
  sudo apt install -y docker.io
  sudo usermod -aG docker "$USER"

Then log out and log back in, or run:

  newgrp docker

EOF
    exit 127
  fi

  if ! docker info >/dev/null 2>&1; then
    cat >&2 <<EOF
Cannot access the Docker daemon.

If the error is "permission denied while trying to connect to the docker API",
refresh your docker group membership:

  newgrp docker

or close this terminal and open a new one.

EOF
    exit 1
  fi

  docker build \
    --build-arg BASE_IMAGE="${BASE_IMAGE}" \
    -f "${DOCKERFILE}" \
    -t "${IMAGE_NAME}" \
    "${WS_DIR}"
}

docker_shell() {
  local mode="${1:-shell}"
  local user="${2:-agv}"

  if ! command -v docker >/dev/null 2>&1; then
    cat >&2 <<EOF
docker command not found.

Install Docker on the host first:

  sudo apt update
  sudo apt install -y docker.io
  sudo usermod -aG docker "$USER"

Then log out and log back in, or run:

  newgrp docker

EOF
    exit 127
  fi

  if ! docker info >/dev/null 2>&1; then
    cat >&2 <<EOF
Cannot access the Docker daemon.

If the error is "permission denied while trying to connect to the docker API",
refresh your docker group membership:

  newgrp docker

or close this terminal and open a new one.

EOF
    exit 1
  fi

  if ! docker image inspect "${IMAGE_NAME}" >/dev/null 2>&1; then
    cat >&2 <<EOF
Docker image not found: ${IMAGE_NAME}

Build it first:

  $0 build

EOF
    exit 1
  fi

  local display="${DISPLAY:-}"
  local xauthority="${XAUTHORITY:-${HOME}/.Xauthority}"
  local x11_args=()
  local runtime_args=()

  if [ "${mode}" = "gui" ] && [ -n "${display}" ]; then
    xhost +local:docker >/dev/null 2>&1 || true
    x11_args+=(
      -e DISPLAY="${display}"
      -e QT_X11_NO_MITSHM=1
      -v /tmp/.X11-unix:/tmp/.X11-unix:rw
    )
    if [ -f "${xauthority}" ]; then
      x11_args+=(-e XAUTHORITY=/tmp/.docker.xauth -v "${xauthority}:/tmp/.docker.xauth:ro")
    fi
  fi

  if [ "${mode}" = "gui" ]; then
    runtime_args+=(--net host --ipc host)
  fi

  if [ -n "${AGV_GALACTIC_DOCKER_ARGS:-}" ]; then
    # shellcheck disable=SC2206
    runtime_args+=(${AGV_GALACTIC_DOCKER_ARGS})
  fi

  docker run --rm -it \
    --name "${CONTAINER_NAME}" \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}" \
    "${runtime_args[@]}" \
    "${x11_args[@]}" \
    -e COLCON_BUILD_BASE=/workspace/AGV_sim_ws/build_galactic \
    -e COLCON_INSTALL_BASE=/workspace/AGV_sim_ws/install_galactic \
    -e COLCON_LOG_BASE=/workspace/AGV_sim_ws/log_galactic \
    -v "${WS_DIR}:/workspace/AGV_sim_ws:rw" \
    -w /workspace/AGV_sim_ws \
    --user "${user}" \
    "${IMAGE_NAME}" \
    /bin/bash
}

docker_clean() {
  docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true
}

docker_sim() {
  local display="${DISPLAY:-}"
  local xauthority="${XAUTHORITY:-${HOME}/.Xauthority}"
  local x11_args=()
  local runtime_args=(--net host --ipc host)

  if ! command -v docker >/dev/null 2>&1; then
    cat >&2 <<EOF
docker command not found.

Install Docker on the host first.

EOF
    exit 127
  fi

  if ! docker info >/dev/null 2>&1; then
    cat >&2 <<EOF
Cannot access the Docker daemon.

Refresh your docker group membership:

  newgrp docker

or close this terminal and open a new one.

EOF
    exit 1
  fi

  if ! docker image inspect "${IMAGE_NAME}" >/dev/null 2>&1; then
    cat >&2 <<EOF
Docker image not found: ${IMAGE_NAME}

Build it first:

  $0 build

EOF
    exit 1
  fi

  if [ -n "${display}" ]; then
    xhost +local:docker >/dev/null 2>&1 || true
    x11_args+=(
      -e DISPLAY="${display}"
      -e QT_X11_NO_MITSHM=1
      -v /tmp/.X11-unix:/tmp/.X11-unix:rw
    )
    if [ -f "${xauthority}" ]; then
      x11_args+=(-e XAUTHORITY=/tmp/.docker.xauth -v "${xauthority}:/tmp/.docker.xauth:ro")
    fi
  fi

  if [ -n "${AGV_GALACTIC_DOCKER_ARGS:-}" ]; then
    # shellcheck disable=SC2206
    runtime_args+=(${AGV_GALACTIC_DOCKER_ARGS})
  fi

  docker rm -f "${CONTAINER_NAME}-sim" >/dev/null 2>&1 || true
  docker run --rm -it \
    --name "${CONTAINER_NAME}-sim" \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}" \
    "${runtime_args[@]}" \
    "${x11_args[@]}" \
    -e COLCON_BUILD_BASE=/workspace/AGV_sim_ws/build_galactic \
    -e COLCON_INSTALL_BASE=/workspace/AGV_sim_ws/install_galactic \
    -e COLCON_LOG_BASE=/workspace/AGV_sim_ws/log_galactic \
    -v "${WS_DIR}:/workspace/AGV_sim_ws:rw" \
    -w /workspace/AGV_sim_ws \
    --user agv \
    "${IMAGE_NAME}" \
    /bin/bash -lc 'cd /workspace/AGV_sim_ws && ./sim.sh "$@"' bash "$@"
}

case "${1:-}" in
  build)
    docker_build
    ;;
  shell)
    docker_shell shell agv
    ;;
  gui)
    docker_shell gui agv
    ;;
  sim)
    shift
    docker_sim "$@"
    ;;
  root)
    docker_shell shell root
    ;;
  clean)
    docker_clean
    ;;
  -h|--help|help|"")
    usage
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac
