import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Method to launch the nodes in the package with bag record flag"""
    args_record_bag = DeclareLaunchArgument('record_rosbag', default_value='False', choices=['True', 'False']) 
    
    rosbag_recorder = ExecuteProcess(
            condition=IfCondition(LaunchConfiguration('record_rosbag')),
            cmd=['ros2', 'bag', 'record', '-o', 'bag_list', '-a', '-x /camera.+'],
            shell=True
        )
    return LaunchDescription([

        args_record_bag,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory(
                    'turtlebot3_gazebo'), 'launch'), '/turtlebot3_world.launch.py'
            ])
        ),

        Node(
            package='walker_bot',
            executable='walker',
        ),

        rosbag_recorder

    ])