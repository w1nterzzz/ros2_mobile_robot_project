#!/usr/bin/env bash
set -eo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
HEADLESS_LAUNCHER="$PROJECT_ROOT/scripts/run_gazebo_headless.sh"
SMOKE_LOG_DIR="$(mktemp -d /tmp/mobile_robot_stage7_smoke.XXXXXX)"

GAZEBO_PROCESS_GROUP=""
LAUNCH_PROCESS_GROUP=""


process_group_exists()
{
  local process_group="$1"

  [[ -n "$process_group" ]] &&
    /bin/kill -0 -- "-$process_group" 2>/dev/null
}


stop_process_group()
{
  local process_group="$1"

  if [[ -z "$process_group" ]]; then
    return
  fi

  if process_group_exists "$process_group"; then
    /bin/kill \
      -s INT \
      -- \
      "-$process_group" \
      2>/dev/null || true

    for _ in {1..40}; do
      if ! process_group_exists "$process_group"; then
        break
      fi

      sleep 0.25
    done
  fi

  if process_group_exists "$process_group"; then
    /bin/kill \
      -s TERM \
      -- \
      "-$process_group" \
      2>/dev/null || true

    for _ in {1..20}; do
      if ! process_group_exists "$process_group"; then
        break
      fi

      sleep 0.25
    done
  fi

  if process_group_exists "$process_group"; then
    /bin/kill \
      -s KILL \
      -- \
      "-$process_group" \
      2>/dev/null || true
  fi

  wait "$process_group" 2>/dev/null || true
}


cleanup()
{
  local test_status=$?

  trap - EXIT

  stop_process_group "$LAUNCH_PROCESS_GROUP"
  stop_process_group "$GAZEBO_PROCESS_GROUP"

  if [[ "$test_status" -ne 0 ]]; then
    printf '\n[FAIL] Stage 7 multi-robot smoke test failed.\n'
    printf '[INFO] Gazebo log: %s\n' \
      "$SMOKE_LOG_DIR/gazebo.log"
    printf '[INFO] ROS launch log: %s\n' \
      "$SMOKE_LOG_DIR/ros_launch.log"

    printf '\n===== Gazebo log tail =====\n'
    tail -n 80 \
      "$SMOKE_LOG_DIR/gazebo.log" \
      2>/dev/null || true

    printf '\n===== ROS launch log tail =====\n'
    tail -n 120 \
      "$SMOKE_LOG_DIR/ros_launch.log" \
      2>/dev/null || true
  else
    printf '\n[PASS] Stage 7 multi-robot smoke test passed.\n'
    printf '[INFO] Logs: %s\n' "$SMOKE_LOG_DIR"
  fi

  exit "$test_status"
}


wait_for_node()
{
  local node_name="$1"

  for _ in {1..30}; do
    if ros2 node list 2>/dev/null \
      | grep -Fxq "$node_name"
    then
      printf '[PASS] Node available: %s\n' "$node_name"
      return
    fi

    if ! process_group_exists "$LAUNCH_PROCESS_GROUP"; then
      printf '[ERROR] ROS launch exited unexpectedly.\n'
      return 1
    fi

    sleep 1
  done

  printf '[ERROR] Timed out waiting for node: %s\n' \
    "$node_name"
  return 1
}


wait_for_message()
{
  local topic_name="$1"

  for _ in {1..30}; do
    if timeout \
      --signal=INT \
      2s \
      ros2 topic echo \
      "$topic_name" \
      --once \
      >/dev/null 2>&1
    then
      printf '[PASS] Message received: %s\n' \
        "$topic_name"
      return
    fi

    if ! process_group_exists "$LAUNCH_PROCESS_GROUP"; then
      printf '[ERROR] ROS launch exited unexpectedly.\n'
      return 1
    fi

    sleep 1
  done

  printf '[ERROR] Timed out waiting for topic: %s\n' \
    "$topic_name"
  return 1
}


trap cleanup EXIT

source /opt/ros/jazzy/setup.bash

if [[ ! -f "$PROJECT_ROOT/install/setup.bash" ]]; then
  printf '[ERROR] Workspace is not built.\n'
  printf '[INFO] Run colcon build --symlink-install first.\n'
  exit 1
fi

source "$PROJECT_ROOT/install/setup.bash"

if [[ ! -x "$HEADLESS_LAUNCHER" ]]; then
  printf '[ERROR] Headless Gazebo launcher is not executable:\n'
  printf '        %s\n' "$HEADLESS_LAUNCHER"
  exit 1
fi

if ! command -v setsid >/dev/null 2>&1; then
  printf '[ERROR] Required command not found: setsid\n'
  exit 1
fi

if ros2 node list 2>/dev/null \
  | grep -Eq '^/robot_[12]/'
then
  printf '[ERROR] Existing robot_1 or robot_2 nodes detected.\n'
  printf '[INFO] Stop the existing simulation before this test.\n'
  exit 1
fi

printf '[INFO] Logs: %s\n' "$SMOKE_LOG_DIR"
printf '[INFO] Starting headless Gazebo server...\n'

setsid \
  "$HEADLESS_LAUNCHER" \
  >"$SMOKE_LOG_DIR/gazebo.log" \
  2>&1 &
GAZEBO_PROCESS_GROUP=$!

sleep 2

if ! process_group_exists "$GAZEBO_PROCESS_GROUP"; then
  printf '[ERROR] Gazebo server exited during startup.\n'
  exit 1
fi

printf '[INFO] Starting dual-robot ROS simulation...\n'

setsid \
  ros2 launch \
  mobile_robot_description \
  multi_robot_simulation.launch.py \
  >"$SMOKE_LOG_DIR/ros_launch.log" \
  2>&1 &
LAUNCH_PROCESS_GROUP=$!

wait_for_node /robot_1/cmd_vel_safety
wait_for_node /robot_2/cmd_vel_safety
wait_for_node /robot_1/state_estimator
wait_for_node /robot_2/state_estimator

wait_for_message /clock
wait_for_message /robot_1/imu/data
wait_for_message /robot_2/imu/data
wait_for_message /robot_1/wheel/odom
wait_for_message /robot_2/wheel/odom
wait_for_message /robot_1/odom
wait_for_message /robot_2/odom
wait_for_message /robot_1/actuator/cmd_vel
wait_for_message /robot_2/actuator/cmd_vel
