# Avant gazebo control

This package adds and controllability interface to the gazebo simulation. With this package the avant simulation model can be sent messages and they will moved to the model.

This package will mimic the [hw_interface](https://github.com/tau-alma/avant_ros/tree/main/hw_interface) node so that running simulations with the avant would be possible. 

with this interface it is possible to control the movement of the model. Over the topic /motion_controller/commands the interface listens the commands

which come as JointState commands in form of:

```
ros2 topic pub /Motion_commands sensor_msgs/msg/JointState "{name: ['gear', 'steering', 'gas'], position: [0.0, 0.0, 0.0]}"
```

These values are then turned to velocity commands for the wheels and articulated joint.

For the manipulator there are two controllers possible to use. 

JointTrajectory controller:

With this user can give angles wanted to execute for the manipulator, but there is no command interface for the user yet.

Velocity controller:

With this controller user can give commands in same fashion as for the motion control. Messages used to control manipulator come in as

```
ros2 topic pub /manipulator_commands sensor_msgs/msg/JointState "{name: ['boom', 'bucket', 'telescope'], velocity: [1.0, 0.0, 0.0]}"
```


## To run and install needed files

This package is used with avant description package which launches this package when needed. To see how to run and please read
the README file in following: 

[avant description](https://github.com/tau-alma/avant_ros/tree/avant_simulation/avant_description)

There you can find commands which will instruct how to run each controller mode and how the simulation is started!


### Depencies

To run this file you will need to have installed the ros2 galactic, rviz2 and gazebo 
environments. 

To install all depencies please follow instructions in the following git repo:

[avant description](https://github.com/tau-alma/avant_ros/tree/avant_simulation/avant_description)


### Link to the issue

[Issue](https://github.com/tau-alma/peams-planning/issues/45)