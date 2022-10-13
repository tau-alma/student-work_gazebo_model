"""
This launch file can be run when user wants to launch gazebo, rviz, gazebo control interface, motion control, robot_localization and slam_toolbox
at the same time.
"""

import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import launch_ros
from launch_ros.parameter_descriptions import ParameterValue
import os
import sys
from ament_index_python.packages import get_package_share_directory
import yaml


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # get paths to the file locations on the system
    
    # define machine type [bucket or fork]
    machine_type = "bucket" 

    pkg_share = launch_ros.substitutions.FindPackageShare(package='avant_description').find('avant_description')
    default_model_path = os.path.join(pkg_share, 'urdf/avant_' + machine_type +'.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/moveit_config.rviz')
    #use_sim_time = LaunchConfiguration('use_sim_time')
    #slam_params_file = LaunchConfiguration('slam_params_file')
    #world_path = os.path.join(pkg_share, 'worlds', "test.world")
    
    world_path = os.path.join(pkg_share, 'worlds', "hills.world")
    config = os.path.join(
        get_package_share_directory('gazebo_control_interface'),
        'config',
        'params.yaml'
        )
  
  
    
    spawn_x_val = "0.0"
    spawn_y_val = '0.0'
    spawn_z_val = '1.0'
    spawn_yaw_val = '0.0'
    for arg in sys.argv:
        if arg.startswith("x:="):
            spawn_x_val = arg.split(":=")[1]

        if arg.startswith("y:="):
            spawn_y_val = arg.split(":=")[1]

        if arg.startswith("z:="):
            spawn_z_val = arg.split(":=")[1]

        if arg.startswith("yaw:="):
            spawn_yaw_val = arg.split(":=")[1]



    # --- MoveIt configurations
    robot_description_config =  os.path.join(
            get_package_share_directory("avant_description"),
            "urdf",
            "avant_bucket.urdf",   
    )
    
    with open(robot_description_config, 'r') as infp:
        robot_desc_cfg = infp.read()
    
    robot_description = {"robot_description": robot_desc_cfg.replace("file://$(find avant_description)","package://avant_description")}


    robot_description_semantic_config = load_file(
        "avant_bucket_moveit_config", "config/avant_bucket.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    kinematics_yaml = load_yaml(
        "avant_bucket_moveit_config", "config/kinematics.yaml"
    )

    # # Planning Functionality
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner", #
            "request_adapters": """ default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        "avant_bucket_moveit_config", "config/ompl_planning.yaml"
    )
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    moveit_simple_controllers_yaml = load_yaml(
        "avant_bucket_moveit_config", "config/avant_controllers_gazebo.yaml"
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
      #  "trajectory_execution.execution_duration_monitoring": False,
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
      #  "trajectory_execution.execution_duration_monitoring":False,
       # "trajectory_execution.allowed_execution_duration_scaling": 100.0,
     #   "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
     }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # # Start the actual move_group node/action server
    run_move_group_node = launch_ros.actions.Node( package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        remappings=[
            ('/joint_states', '/bag_joint_states'), # now will subscribe to /manipulator_joint_states [I tried it does not solve the time issue]
        ], #change second one to desired topic
        arguments=['--verbose'],
    )

    
    # ros2 run topic_tools transform /imu --field orientation /norm std_msgs/Float64 'std_msgs.msg.Float64(data=numpy.sqrt(numpy.sum(numpy.array([m.x, m.y, m.z, m.w]))))' --import std_msgs numpy
    # ros2 run topic_tools transform /joint_states --field header name position velocity effort /norm sensor_msgs/JointState "sensor_msgs.msg.JointState(data=[header,name,position,velocity,effort])"


    # Define the robot state publisher
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(Command(['xacro ', LaunchConfiguration('model')]), value_type=str)}]
    )

    # Define the rviz configuration 
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
  
        ],
    )   


    # Define the spawner and the object to spawn
    spawn_entity = launch_ros.actions.Node(
    	package='gazebo_ros', 
    	executable='spawn_entity.py',
        arguments=['-entity', 'avant', '-topic', 'robot_description',
                    '-x', spawn_x_val,
                    '-y', spawn_y_val,
                    '-z', spawn_z_val,
                    '-Y', spawn_yaw_val],
        output='screen',
    )

    robot_control_node = launch_ros.actions.Node(
        package='gazebo_control_interface',
        executable='control_client',
         parameters=[
          config
        ],
    )

    motion_control_node = launch_ros.actions.Node(
        package="motion_control",
        executable="motion_control_node"
    )

    robot_localization_node = launch_ros.actions.Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    ros2_controllers_path = os.path.join(
            get_package_share_directory("avant_bucket_moveit_config"),
            "config",
            "avant_ros_controllers_gazebo.yaml",
        ),


    ros2_control_node = launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        )
        

        # Load controllers # state of the robot not known missing base
    load_controllers = []
    for controller in ["motion_controller","joint_state_broadcaster","manipulator_controller"]: #, joint_state_broadcaster , joint_trajectory_controller, "joint_state_controller" <-- these don't solve missing base issue either
        load_controllers += [
                launch.actions.ExecuteProcess( #"manipulator_controller", #,"joint_trajectory_controller"
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
                )
            ]
    # declare_use_sim_time_argument = DeclareLaunchArgument(
    #     'use_sim_time',
    #     default_value='True',
    #     description='Use simulation/Gazebo clock')

    # declare_slam_params_file_cmd = DeclareLaunchArgument(
    #     'slam_params_file',
    #     default_value=os.path.join(pkg_share,
    #                                'config', 'slam_params.yaml'),
    #     description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    # start_async_slam_toolbox_node = launch_ros.actions.Node(
    #     parameters=[
    #       slam_params_file,
    #       {'use_sim_time': use_sim_time}
    #     ],
    #     package='slam_toolbox',
    #     executable='async_slam_toolbox_node',
    #     name='slam_toolbox',
    #     output='screen')

    
    trajectory_action_server = launch_ros.actions.Node(
        package='manipulator_controller',
        executable='manipulator_trajectory_action_server',
        parameters=[
             {'P_boom': 12.0 }, # 6,10 works good                           # 5.0; 0.00; 0.12; 1.0
             {'I_boom': 0.0 },
             {'D_boom': .1},
             {'FF_boom': 0.3},

             {'P_telescope': 2.0 }, # 2.0; 0.00; 0.2; 1.0
             {'I_telescope': 0.00 },                 
             {'D_telescope': 0.0 },
             {'FF_telescope': 0.0 },

             {'P_bucket': 1.2 },                  # 3.0; 0.3; 0.00; 1.0
             {'I_bucket': 0.0},                # 0.35; 0.005; 0.1; 0.3
             {'D_bucket': 0.2},
             {'FF_bucket': 0.0},
# latest working for bucket was 0.3 and 0.0 0.0 0.0
        ]
    )

    # Launch the defined parameters
    return launch.LaunchDescription([   
        
        # declare_use_sim_time_argument,   
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='true',
                                            description='Flag to enable use_sim_time'), 
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'], output='screen'), #, world_path
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        robot_state_publisher_node,
        spawn_entity,
        robot_control_node,
        motion_control_node,

        # set timer action for launching rviz node to avoid unnecessery amount of warning messages
        launch.actions.TimerAction(
            period=4.0,
            actions = [rviz_node]
        ),





        # run moveit
        run_move_group_node,


        ros2_control_node,
        



        # activate ekf
        #robot_localization_node,

        # activate slamtoolbox
        #declare_slam_params_file_cmd,
        #start_async_slam_toolbox_node,

        # make sure that execution has no time limit
        launch.actions.ExecuteProcess(cmd=['ros2',
        'service',
        'call',
        '/moveit_simple_controller_manager/set_parameters',
        'rcl_interfaces/srv/SetParameters',
        '{parameters: [{name: "trajectory_execution.execution_duration_monitoring", value: {type: 1, bool_value: False}}]}'
        ],
        output='screen'
        ),

        trajectory_action_server,

    ]+ load_controllers ) #
