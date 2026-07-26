#!/usr/bin/env bash

set -eo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd -- "${script_dir}/.." && pwd)"

source /opt/ros/jazzy/setup.bash
source "${project_dir}/install/setup.bash"

launch_pid=""

cleanup() {
  trap - EXIT INT TERM

  if [[ -n "${launch_pid}" ]] && kill -0 -- "-${launch_pid}" 2>/dev/null; then
    kill -TERM -- "-${launch_pid}" 2>/dev/null || true
    wait "${launch_pid}" 2>/dev/null || true
  fi
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

setsid ros2 launch mobile_robot_description display.launch.py &
launch_pid=$!

LIBGL_ALWAYS_SOFTWARE=1 \
MESA_GL_VERSION_OVERRIDE=3.3COMPAT \
rviz2 \
  -d "${project_dir}/install/mobile_robot_description/share/mobile_robot_description/rviz/mobile_robot.rviz"
