nemesis
=====

Forked from: https://github.com/husky/husky/tree/kinetic-devel commit [2d368cf32530401238cb45f31e54f40080dd6dc1](https://github.com/husky/husky/commit/2d368cf32530401238cb45f31e54f40080dd6dc1)

This project is an adaptation of the [Husky ROS stack from Clearpath Robotics](http://wiki.ros.org/Robots/Husky), to integrate and control the [PiBorg Monsterborg robot platform](https://www.piborg.org/robots-1/monsterborg) with ROS.

It provides a ROS RobotHW interface implementation using a cythonized version of the original python [ThunderBorg](https://www.piborg.org/motor-control-1135/thunderborg) motor controller driver from PiBorg, for the purpose of reducing performance overhead in the main ROS control loop.

 - nemesis_control : Control configuration
 - nemesis_description : Robot description (URDF)
 - nemesis_msgs : Message definitions
 - nemesis_viz : Visualization (rviz) configuration and bringup







