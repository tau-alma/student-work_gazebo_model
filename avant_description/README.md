# Avant Description

This package contains necessary ROS2 packages to run gazebo simulation with avant.
This package has multiple plugin depecies to show the model in the gazebo environment.

## Sensors

There are sensors also added to the model:
    - IMU for front axle
    - IMU for the boom
    - IMU for the bucket
    - Lidar
    - GNSS
### Depencies

To run this file you will need to have installed the ros2 galactic, rviz2 and gazebo 
environments. 

### plugins

You need to have installed following plugins to run the package.

You install required depencies by running following command in the workspace:

```
rosdep install -r --from-paths src -y --ignore-src
```

or by installing the following manually:

ros2 controller:
```
sudo apt-get install ros-galactic-ros2-control
```

ros2 controllers:
```
sudo apt-get install ros-galactic-ros2-controllers
```

ros2 gazebo controller:
```
sudo apt-get install ros-galactic-gazebo-ros2-control
```

xarco:
```
sudo apt install ros-galactic-xacro
```

robot_localization
```
sudo apt install ros-galactic-robot-localization
```

Please add the following line to the end of your bashrc file to have the april tag adding work!

open bashrc:

```
gedit ~/.bashrc
```

add the following line modified to fit your configuration at the end in the file:

```
export GAZEBO_RESOURCE_PATH=~/<route/to/your/package>:/usr/share/gazebo-<your-gazebo-version>/
```

This will add your package as one of the gazebo resource paths, which allows using custom textures on the model.


### to run the project

download the files to your workspace folder inside the src file after which build it with:

```
colcon build
```

and source the setup file. 

```
source install/local_setup.bash
```

and the you can run needed file to control the model with:

```
ros2 launch avant_description control.launch.py
```

This will launch gazebo, rviz and control node for the model. To control model you need to send it JointState command
to control the linear velocity and angular velocity. motion control node will turn the commands then suitable for the 
controller. 

If the user wants to modify the spawn location of the model, the starting location can be defined after the launch command as follows:

```
ros2 launch avant_description control.launch.py x:=1.0 y:=2.0 z:=0.2 yaw:=1.0
```


example command to use in command line tool:

```
ros2 topic pub /wanted_speeds sensor_msgs/msg/JointState "{name: ['vel', 'ang_vel'], position: [5.0, 10.0], velocity: [0.0, 0.0], effort: [0.0, 0.0]}"
```

This command will send linear velocity command of 5.0 and angular velocity command of 10.0 to motion controller which will scale the command to fit to the model.

### Worlds

There is a possibility to add different worlds to the project. There is a dedicated launch file made for launching your own maps.
There has been made a simple map which contains different amount of elevations for the testing. 

to launch different maps:

```
ros2 launch avant_description avant_world.launch.py 
```

if you want to use your own map use:

```
ros2 launch avant_description avant_world.launch.py world_path:=<path/to/the/world/file>
```


### Nav2 

This package has been also integrated with navigation2 package. User can run the model in premade map environment to try out the nav2.

To run the nav2 stack use following command:

```
ros2 launch avant_description avant_bringup.launch.py
```

This command launches all of the needed files to run nav2 stack in gazebo for the model.

to activate localization user must define starting point with the button "2D pose estimation" for the acml localization with rviz2 screen which opens also. 

After this the acml will have starting point for the localization and then user can give the model goal point with button "nav2 goal". After this the model starts navigating to the goal.

#### Parameters

Provides parameters that can be changed by user parameters are changed by adding parameters to the end of the launch command like following:


```
ros2 launch avant_description avant_bringup.launch.py <param_name>:=<param_value>
```

Package contains following parameters for the user to set:

Param  | Type | default |desc 
:-------------: | :-------------: | :-------------: | :-------------:
slam  | bool | false | If the navigation2 is run with slam
map | path to the map file (.yaml) | map_1658913299.yaml | yaml file of the map to use for the navigation2
use_sim_time | bool | true | Use simulator time (gazebo) or real unix time
param_file | path to the navigation2 params file (yaml.) | nav2_params.yaml | yaml file of the map to use for the navigation2
autostart | bool | true | if the navigation2 package is ran automatically
rviz_config_file | path to the rviz2 params file (yaml.) | nav2_native.rviz | yaml file for the rviz2 view
use_rviz2 | bool | true | if the rviz2 is launched
use_sim | bool | true | if the gazebo sim is launched 
world_path | path to the world file (.world (.sdf?)) | maze.world | world file where the robot is launched
x | float | 0.0 | location of the robot on x-axis
y | float | 0.0 | location of the robot on y-axis
z | float | 0.0 | location of the robot on z-axis 
psi | float | 0.0 | twist of the robot along z-axis

#### Groot

if you wish to use groot to make behaviour trees or monitor them in the real time you can use groot to show 
and build them.

You can download groot from the following site [Groot](https://github.com/BehaviorTree/Groot):

After building the groot package you can run it by following command:

```
ros2 run groot Groot
```

this will open the software and you can use it to monitor or build your trees.





### Link to the issue

[Issue](https://github.com/tau-alma/peams-planning/issues/45)