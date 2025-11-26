README – Control Square Task & ROS Message Task
📌 Overview

This repository contains two ROS packages developed for the CO3242 coursework:

ControlSquareOdometry – A package that drives a differential-drive robot in a square using encoder-based odometry and simple motion control.

RosMessageTask – A package demonstrating ROS publish/subscribe communication using custom message processing.

Both packages run on ROS (ROS Noetic/ROS1) and follow standard catkin workspace structure.

---------------------------------------------------------
🚗 ControlSquareOdometry Package
---------------------------------------------------------
📝 Introduction

The Control Square task uses wheel encoder data to estimate robot odometry and command the robot to move along a square path.
The node alternates between straight-line motion and 90° rotations using state-based control until all four sides are completed.

⚙️ Features

Reads left & right encoder tick counts

Computes odometry: x, y, θ

State machine for:

MOVE_FORWARD

TURN_LEFT

NEXT_SIDE

Publishes velocity commands (geometry_msgs/Twist)

Stops after drawing a full square

▶️ How to Run
1. Build the workspace
cd ~/catkin_ws
catkin_make
source devel/setup.bash

2. Launch the node

Using rosrun:

rosrun ControlSquareOdometry control_square_odometry


Using roslaunch:

roslaunch ControlSquareOdometry control_square.launch

---------------------------------------------------------
📨 RosMessageTask Package
---------------------------------------------------------
📝 Introduction

The ROS Message task demonstrates basic ROS communication.
The node subscribes to a topic, processes incoming messages, and republishes processed data. It is designed to teach message flow, callbacks, and rate-based publishing.

⚙️ Features

Subscribes to input topic

Callback-based message handling

Publishes processed message

Maintains steady loop rate

Clean separation of node init, callback, and main loop logic

▶️ How to Run
1. Build
cd ~/catkin_ws
catkin_make
source devel/setup.bash

2. Run the node
rosrun RosMessageTask message_task

---------------------------------------------------------
🧩 Dependencies

Both packages depend on standard ROS message libraries:

roscpp

std_msgs

sensor_msgs

geometry_msgs

Ensure these are installed on your ROS environment.

---------------------------------------------------------
📚 Notes

ros::spin() should not block your control loop.
Use ros::spinOnce() inside loops when performing continuous control (as in ControlSquareOdometry).

For message-only tasks without loops, ros::spin() is recommended.
