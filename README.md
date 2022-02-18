# nemesis

![Monsterborg Base](https://live.staticflickr.com/65535/51802147128_ae0ffda62b_k.jpg)

This project is an adaptation of the [Husky ROS stack from Clearpath Robotics](http://wiki.ros.org/Robots/Husky), to integrate and control the [PiBorg Monsterborg robot platform](https://www.piborg.org/robots-1/monsterborg) with ROS.

Forked from: https://github.com/husky/husky/tree/kinetic-devel commit [2d368cf32530401238cb45f31e54f40080dd6dc1](https://github.com/husky/husky/commit/2d368cf32530401238cb45f31e54f40080dd6dc1)

It provides a ROS hardware_interface::RobotHW implementation using a cythonized version of the original python [ThunderBorg](https://www.piborg.org/motor-control-1135/thunderborg) motor controller driver from PiBorg, for the purpose of reducing performance overhead in the main ROS control loop.

A high level overview of this project can be found in this post https://antoan.github.io/Nemesis/

 - nemesis_base : includes a hardware_interface::RobotHW implementation.
 - nemesis_control : Control configuration
 - nemesis_description : Robot description (URDF)
 - nemesis_msgs : Message definitions
 - nemesis_viz : Visualization (rviz) configuration and bringup
