#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Darby Lim

import os
import launch_ros
import launch
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    pkg_share = launch_ros.substitutions.FindPackageShare(package='avant_description').find('avant_description')
    urdf_file_name = 'avant_bucket.urdf'
    world_path = os.path.join(pkg_share, 'worlds', "test_avant.world")
    print("urdf_file_name : {}".format(urdf_file_name))
    
    x = LaunchConfiguration("x", default="0.0")
    y = LaunchConfiguration("y", default="0.0")
    z = LaunchConfiguration("z", default="0.0")
    psi = LaunchConfiguration("psi",default="0.0")
    use_sim = LaunchConfiguration("use_gazebo", default="true")
    world_path = LaunchConfiguration("world_path")

    urdf = os.path.join(
        get_package_share_directory('avant_description'),
        'urdf',
        urdf_file_name)
    

    # Define the robot state publisher
    robot_state_publisher_node = launch_ros.actions.Node(
        condition=IfCondition(use_sim),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(Command(['xacro ', LaunchConfiguration('model')]), value_type=str)}]
    )

    # Major refactor of the robot_state_publisher
    # Reference page: https://github.com/ros2/demos/pull/426
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    rsp_params = {'robot_description': robot_desc}

    remappings = [('joint_states', 'avant_states')]
    # print (robot_desc) # Printing urdf information.

      # Define the spawner and the object to spawn
    spawn_entity = launch_ros.actions.Node(
        condition=IfCondition(use_sim),
    	package='gazebo_ros', 
    	executable='spawn_entity.py',
        arguments=['-entity', 'avant', '-topic', 'robot_description',
                    '-x', x,
                    '-y', y,
                    '-z', z,
                    '-Y', psi],
        output='screen',
    )

    robot_control_node = launch_ros.actions.Node(
        package='gazebo_control_interface',
        executable='control_client',
    )

    motion_control_node = launch_ros.actions.Node(
        package="motion_control",
        executable="motion_control_node"
    )

    gazebo_server_sim_launch =  launch.actions.ExecuteProcess(condition=IfCondition(use_sim), 
                                                       cmd=['gzserver', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen')

    gazebo_client_sim_launch =  launch.actions.ExecuteProcess(condition=IfCondition(use_sim), 
                                                       cmd=['gzclient'], output='screen')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=urdf,
                                            description='Absolute path to robot urdf file'),
        gazebo_server_sim_launch,
        gazebo_client_sim_launch,
        robot_state_publisher_node,

        spawn_entity,
        robot_control_node,
        motion_control_node,
        launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'joint_state_broadcaster'],
        output='screen'
        ),
        launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'motion_controller'],
        output='screen'
        ),
        launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'manipulator_controller'],
        output='screen'
        ),
    ])