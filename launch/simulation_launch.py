# NOTE: For production deployment, enable SROS2 security:
#   1. Generate keystore: ros2 security generate_keystore <path>
#   2. Create enclaves for each node
#   3. Set env: ROS_SECURITY_ENABLE=true ROS_SECURITY_STRATEGY=Enforce
#   4. See: https://docs.ros.org/en/jazzy/Tutorials/Advanced/Security.html

import json
import os
import time

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_DEBUG_ENABLED = os.environ.get('ROBOTIC_ARM_DEBUG', '').lower() in ('1', 'true', 'yes')
_DEBUG_LOG = os.environ.get('ROBOTIC_ARM_DEBUG_LOG') if _DEBUG_ENABLED else None

WORLD_NAME = 'robotic_arm_world'
MODEL_NAME = 'robotic_arm'

ARM_JOINTS = [
    'base_rotator', 'shoulder_pitch', 'elbow_pitch', 'wrist_pitch', 'wrist_roll',
]
GRIPPER_JOINTS = ['gripper_joint', 'gripper_joint_right']
ALL_JOINTS = ARM_JOINTS + GRIPPER_JOINTS


def _dbg(hypothesis_id, location, message, data=None, run_id='post-fix'):
    if not _DEBUG_ENABLED or _DEBUG_LOG is None:
        return
    try:
        os.makedirs(os.path.dirname(_DEBUG_LOG), exist_ok=True)
        with open(_DEBUG_LOG, 'a', encoding='utf-8') as f:
            f.write(json.dumps({
                'runId': run_id,
                'hypothesisId': hypothesis_id,
                'location': location,
                'message': message,
                'data': data or {},
                'timestamp': int(time.time() * 1000),
            }) + '\n')
    except OSError:
        pass


def _joint_cmd_bridge(joint_name):
    """ROS -> Gazebo joint position command."""
    gz_topic = f'/model/{MODEL_NAME}/joint/{joint_name}/cmd_pos'
    return f'{gz_topic}@std_msgs/msg/Float64]gz.msgs.Double'


def generate_launch_description():
    pkg_share = get_package_share_directory('robotic_arm_control')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(pkg_share, 'urdf', 'robotic_arm.urdf.xacro')
    world_file = os.path.join(pkg_share, 'world', 'robotic_arm_world.sdf')

    if not os.path.isfile(xacro_file):
        raise FileNotFoundError(
            f'Missing {xacro_file}. Run: bash build.sh'
        )

    doc = xacro.process_file(xacro_file)
    robot_description_xml = doc.toxml()

    _dbg('B', 'simulation_launch.py:xacro', 'xacro processed', {
        'urdf_length': len(robot_description_xml),
        'has_joint_state_pub': 'JointStatePublisher' in robot_description_xml,
        'has_joint_pos_ctrl': 'JointPositionController' in robot_description_xml,
    })
    _dbg('D', 'simulation_launch.py:mode', 'using ros_gz_bridge joint control', {
        'world': WORLD_NAME,
        'model': MODEL_NAME,
        'joint_count': len(ALL_JOINTS),
    })

    use_sim_time = LaunchConfiguration('use_sim_time')
    gz_args = f'-r -v 4 "{world_file}"'

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description_xml, 'use_sim_time': use_sim_time},
        ],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', MODEL_NAME,
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.05',
        ],
    )

    bridge_args = [
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        f'/world/{WORLD_NAME}/model/{MODEL_NAME}/joint_state'
        f'@sensor_msgs/msg/JointState[gz.msgs.Model',
    ]
    bridge_args.extend(_joint_cmd_bridge(j) for j in ALL_JOINTS)

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=bridge_args,
        remappings=[
            (f'/world/{WORLD_NAME}/model/{MODEL_NAME}/joint_state', '/joint_states'),
        ],
    )

    arm_controller = Node(
        package='robotic_arm_control',
        executable='arm_controller_node',
        name='arm_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock',
        ),
        gz_sim,
        robot_state_publisher,
        TimerAction(period=3.0, actions=[spawn_robot]),
        TimerAction(period=5.0, actions=[gz_bridge]),
        TimerAction(period=8.0, actions=[arm_controller]),
    ])
