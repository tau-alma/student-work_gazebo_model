# Copyright (c) 2018 Intel Corporation
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

import os
import sys
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace,Node


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('avant_description')
    launch_dir = os.path.join(bringup_dir, 'launch')
    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim = LaunchConfiguration("use_gazebo")
    use_ekf = LaunchConfiguration('use_ekf')
    use_dual_ekf = LaunchConfiguration('use_dual_ekf')
    default_nav_through_poses_bt_xml = LaunchConfiguration('default_nav_through_poses_bt_xml')
    default_nav_to_pose_bt_xml = LaunchConfiguration('default_nav_to_pose_bt_xml')
    controller_bt_xml_filename = LaunchConfiguration('controller_bt_xml_filename')
    world_path = LaunchConfiguration("world_path")

    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    psi = LaunchConfiguration("psi")

    declare_x_cmd = DeclareLaunchArgument(
        'x',
        default_value="0.0",
        description='Top-level namespace')

    declare_y_cmd = DeclareLaunchArgument(
        'y',
        default_value="0.0",
        description='Top-level namespace')

    declare_z_cmd = DeclareLaunchArgument(
        'z',
        default_value="0.0",
        description='Top-level namespace')

    declare_psi_cmd = DeclareLaunchArgument(
        'psi',
        default_value="0.0",
        description='Top-level namespace')
    
    declare_map_cmd = DeclareLaunchArgument(
        "world_path", 
        default_value=os.path.join(bringup_dir, 'worlds', "maze.world"),
        description="map file location"
    )


    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_dir, 'maps', 'map_1658913299.yaml'),
        description='Full path to map yaml file to load')

    declare_gazebo_cmd = DeclareLaunchArgument(
        "use_gazebo",
        default_value="true",
        description="Whether to turn on gazebo sim"
    )    

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_robot_controller_cmd = DeclareLaunchArgument(
        'robot_controller',default_value='true',
        description='Whether to run robot_controller for high-level control'
    )

    default_nav_through_poses_bt_xml_cmd = DeclareLaunchArgument(
        'default_nav_through_poses_bt_xml',
        default_value=os.path.join(bringup_dir,
                'behavior_trees', 'bt_navigator_through_poses.xml'),
        description='Full path to the behavior tree xml file to use')

    default_nav_to_pose_bt_xml_cmd = DeclareLaunchArgument(
        'default_nav_to_pose_bt_xml',
        default_value=os.path.join(bringup_dir,
                'behavior_trees', 'bt_navigator.xml'),
        description='Full path to the behavior tree xml file to use')

    declare_controller_bt_xml_cmd = DeclareLaunchArgument(
        'controller_bt_xml_filename',
        default_value=os.path.join(bringup_dir,
            'behavior_trees', 'mir_navigator.xml'),
        description='Full path to the behavior tree xml file to use')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')
        
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(bringup_dir, 'rviz', 'nav2_native.rviz'),
        description='Full path to the RVIZ config file to use')

    declare_use_ekf_cmd = DeclareLaunchArgument(
        'use_ekf',
        default_value='True',
        description='Whether to start ekf')
        
    declare_use_dual_ekf_cmd = DeclareLaunchArgument(
        'use_dual_ekf',
        default_value='False',
        description='Whether to start dual ekf')
        

    # Specify the actions
    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),
        #Node(
        #    package='tf2_ros',
        #    executable='static_transform_publisher',
        #    output='screen',
        #    arguments=['0', '0', '1.7', '0.0499167', '0.0499167', '-0.0024979173', '0.99750208','avant', 'velodyne']),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz.launch.py')),
            condition=IfCondition(use_rviz),
            launch_arguments={'namespace': '',
                            'use_namespace': 'false',
                            'use_sim_time': use_sim_time,
                            'rviz_config': rviz_config_file}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'avant_state_publisher_launch.py')),
            launch_arguments={'x': x,
                              'y': y,
                              'z': z,
                              'psi': psi,
                              'world_path': world_path,
                              'use_gazebo': use_sim}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'slam.launch.py')),
            condition=IfCondition(slam),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir,
                                                       'localization.launch.py')),
            condition=IfCondition(PythonExpression(['not ', slam])),
            launch_arguments={'namespace': namespace,
                              'map': map_yaml_file,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'use_ekf': use_ekf,
                              'use_dual_ekf': use_dual_ekf,
                              'use_lifecycle_mgr': 'false'}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation.launch.py')),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'use_lifecycle_mgr': 'false',
                              'map_subscribe_transient_local': 'true'}.items()),
    ])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_use_dual_ekf_cmd)
    ld.add_action(declare_use_ekf_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(default_nav_through_poses_bt_xml_cmd)
    ld.add_action(default_nav_to_pose_bt_xml_cmd)
    ld.add_action(declare_controller_bt_xml_cmd)
    # Declare the launch options
    ld.add_action(declare_x_cmd)
    ld.add_action(declare_y_cmd)
    ld.add_action(declare_z_cmd)
    ld.add_action(declare_psi_cmd)
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_gazebo_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    #ld.add_action(start_avant_velodyne_tf_pub)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)

    return ld