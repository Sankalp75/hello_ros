#!/usr/bin/env bash
# Clean rebuild — required after URDF/launch changes. Run inside fusion-juno-gpu.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$SCRIPT_DIR"

_DEBUG_ENABLED=false
if [[ "${ROBOTIC_ARM_DEBUG:-}" =~ ^(1|true|yes)$ ]]; then
  _DEBUG_ENABLED=true
fi

LOG="${ROBOTIC_ARM_DEBUG_LOG:-}"

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
source "/opt/ros/$ROS_DISTRO/setup.zsh" #Zsh shell is always recommended.
cd "$WS"
rm -rf build install log
colcon build

if $_DEBUG_ENABLED && [[ -n "$LOG" ]]; then
  mkdir -p "$(dirname "$LOG")"
  echo '{"runId":"build","location":"build.sh","message":"colcon build start","timestamp":'$(date +%s000)'}' >> "$LOG"
fi

colcon build
source install/setup.bash

XACRO="install/robotic_arm_control/share/robotic_arm_control/urdf/robotic_arm.urdf.xacro"
LAUNCH="install/robotic_arm_control/share/robotic_arm_control/launch/simulation_launch.py"

if [[ ! -f "$XACRO" ]]; then
  if $_DEBUG_ENABLED && [[ -n "$LOG" ]]; then
    echo '{"runId":"build","location":"build.sh","message":"FAIL xacro not installed","timestamp":'$(date +%s000)'}' >> "$LOG"
  fi
  echo "ERROR: $XACRO missing after build"
  exit 1
fi

if ! grep -q 'ros_gz_bridge' "$LAUNCH"; then
  if $_DEBUG_ENABLED && [[ -n "$LOG" ]]; then
    echo '{"runId":"build","location":"build.sh","message":"FAIL stale launch in install","timestamp":'$(date +%s000)'}' >> "$LOG"
  fi
  echo "ERROR: install/launch is stale — expected ros_gz_bridge in simulation_launch.py"
  exit 1
fi

if $_DEBUG_ENABLED && [[ -n "$LOG" ]]; then
  echo '{"runId":"build","location":"build.sh","message":"build OK xacro and launch installed","timestamp":'$(date +%s000)'}' >> "$LOG"
fi
echo "Build OK. Run: ros2 launch robotic_arm_control simulation_launch.py"
