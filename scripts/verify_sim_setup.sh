#!/usr/bin/env bash
# Run inside distrobox: bash scripts/verify_sim_setup.sh
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$SCRIPT_DIR"

_DEBUG_ENABLED=false
if [[ "${ROBOTIC_ARM_DEBUG:-}" =~ ^(1|true|yes)$ ]]; then
  _DEBUG_ENABLED=true
fi

LOG="$WS/.cursor/debug-2f27da.log"
if $_DEBUG_ENABLED; then
  mkdir -p "$(dirname "$LOG")"
fi

log_json() {
  if ! $_DEBUG_ENABLED; then return; fi
  local hyp="$1" loc="$2" msg="$3"
  printf '{"sessionId":"2f27da","runId":"verify","hypothesisId":"%s","location":"%s","message":"%s","timestamp":%s}\n' \
    "$hyp" "$loc" "$msg" "$(date +%s000)" >> "$LOG"
}

BUILD_LOG=$(mktemp /tmp/robotic_arm_build.XXXXXX.log) || {
  echo "ERROR: failed to create temp file"
  exit 1
}
cleanup() { rm -f "$BUILD_LOG" /tmp/robotic_arm_processed.urdf; }
trap cleanup EXIT

source /opt/ros/jazzy/setup.bash
cd "$WS"

log_json "A" "verify:build" "starting colcon build"
colcon build 2>&1 | tee "$BUILD_LOG"
source install/setup.bash

XACRO_INST="install/robotic_arm_control/share/robotic_arm_control/urdf/robotic_arm.urdf.xacro"
if [[ -f "$XACRO_INST" ]]; then
  log_json "A" "verify:install" "xacro installed OK"
else
  log_json "A" "verify:install" "FAIL xacro missing in install"
  echo "ERROR: rebuild required — xacro not in install/"
  exit 1
fi

PROCESSED_URDF=$(mktemp /tmp/robotic_arm_processed.XXXXXX.urdf) || {
  echo "ERROR: failed to create temp file"
  exit 1
}

ROS2_CONTROL_CONFIG="$WS/install/robotic_arm_control/share/robotic_arm_control/config/ros2_control.yaml"
xacro "$XACRO_INST" use_sim:=true \
  ros2_control_config:="$ROS2_CONTROL_CONFIG" \
  > "$PROCESSED_URDF"

if grep -q gz_ros2_control "$PROCESSED_URDF"; then
  log_json "B" "verify:xacro" "gz_ros2_control present in URDF"
else
  log_json "B" "verify:xacro" "FAIL no gz_ros2_control in URDF"
  exit 1
fi

for pkg in ros_gz_sim ros_gz_bridge gz_ros2_control controller_manager; do
  if ros2 pkg prefix "$pkg" &>/dev/null; then
    log_json "D" "verify:deps" "found $pkg"
  else
    log_json "D" "verify:deps" "MISSING $pkg"
    echo "Install: sudo apt install ros-jazzy-${pkg//_/-} (adjust name)"
  fi
done

echo "Verify complete."
