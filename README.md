# Robotic Arm Control

ROS 2 Jazzy package for controlling a simulated 6-DOF robotic arm in Gazebo Harmonic.

The `ArmControllerNode` subscribes to text commands (`PICK`, `PLACE`, `HOME`, etc.) via `/arm_controller/commands`, converts them to joint-position commands, and publishes to the arm's joints (including gripper).

## Build

```bash
colcon build --packages-select robotic_arm_control
source install/setup.bash
```

## Run

- **Simulation:** `ros2 launch robotic_arm_control simulation_launch.py`
- **Standalone (robot state publisher):** `ros2 launch robotic_arm_control robot_launch.py`

## Structure

- `src/arm_controller_node.cpp` — main controller logic
- `launch/` — launch files for simulation and real-robot
- `urdf/` — robot model definitions
- `config/` — ROS 2 control and controller configs
- `world/` — Gazebo world file
