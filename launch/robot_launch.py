import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robotic_arm_control')
    xacro_file = os.path.join(pkg_share, 'urdf', 'robotic_arm.urdf.xacro')

    import xacro as xacro_module
    doc = xacro_module.process_file(xacro_file)
    robot_description = {'robot_description': doc.toxml()}

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[robot_description, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen',
        ),
        Node(
            package='robotic_arm_control',
            executable='arm_controller_node',
            name='arm_controller',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen',
        ),
    ])
