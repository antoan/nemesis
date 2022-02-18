# nemesis

![Mapping Session](/docs/images/nemesis_mapping.gif)

A high level overview of the project this repository is part of, can be found in this post https://antoan.github.io/Nemesis/

It is based on an adaptation of the [Husky ROS stack from Clearpath Robotics](http://wiki.ros.org/Robots/Husky), to integrate and control the [PiBorg Monsterborg robot platform](https://www.piborg.org/robots-1/monsterborg) with ROS Kinetic.

<p align="center">
  <img src="/docs/images/20220204_164520.jpg" width="200">
  <img src="/docs/images/DSC_0023.jpg" width="200">
  <img src="/docs/images/DSC_0024.jpg" width="200">
  <img src="/docs/images/DSC_0026.jpg" width="200">
  <img src="/docs/images/IMG_20210909_154740524.jpg" width="200">
  <img src="/docs/images/IMG_20210814_193007750.jpg" width="200">
  <img src="/docs/images/IMG_20211023_150932701_HDR.jpg" width="200">
</p>

Forked from: https://github.com/husky/husky/tree/kinetic-devel [commit](https://github.com/husky/husky/commit/2d368cf32530401238cb45f31e54f40080dd6dc1)

It includes a ROS `hardware_interface::RobotHW` implementation, linked with a cythonized version of the original python [ThunderBorg](https://www.piborg.org/motor-control-1135/thunderborg) motor controller driver from PiBorg for the purpose of reducing performance overhead in the main ROS control loop.

It is used in conjunction with a related project repository which provides prerception, tracking & mapping and experimental `move_base` setup on ROS Melodic. [antoan/nemesis_core](https://github.com/antoan/nemesis_core).  

- nemesis_base : includes a `hardware_interface::RobotHW` implementation.
- nemesis_control : Control configuration
- nemesis_description : Robot description (URDF)
- nemesis_msgs : Message definitions
- nemesis_viz : Visualization (rviz) configuration
