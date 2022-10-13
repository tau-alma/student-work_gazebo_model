"""
This launch file can be run when user wants to launch gazebo, rviz, gazebo control interface and motion control nodes at the
same time.
"""

import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import launch_ros
from launch_ros.parameter_descriptions import ParameterValue
import os
import sys
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # get paths to the file locations on the system
    pkg_share = launch_ros.substitutions.FindPackageShare(package='avant_description').find('avant_description')
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation/Gazebo clock')

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(pkg_share,
                                   'config', 'slam_params.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    start_async_slam_toolbox_node = launch_ros.actions.Node(
        parameters=[
          slam_params_file,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')
    
    # Launch the defined parameters
    return launch.LaunchDescription([   
    
        # activate slamtoolbox
        declare_slam_params_file_cmd,
        declare_use_sim_time_argument,
        start_async_slam_toolbox_node

    ])
