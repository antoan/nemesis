/**
 *
 *  \author     Antoan Bekele <antoan.bekele@gmail.com.com>
 *
 */

/**
 *
 *  \author     Paul Bovbel <pbovbel@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2014-2015, Clearpath Robotics, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 *
 */

#ifndef NEMESIS_BASE_NEMESIS_HARDWARE_H
#define NEMESIS_BASE_NEMESIS_HARDWARE_H

#include "cython_thunderborg.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/robot_hw.h"
#include "nemesis_base/nemesis_diagnostics.h"
#include "nemesis_msgs/NemesisStatus.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <Python.h>
#include <string>

namespace nemesis_base {

/**
 * Class representing Nemesis hardware, allows for ros_control to modify internal
 * state via joint interfaces
 */
class NemesisHardware : public hardware_interface::RobotHW {
public:
  NemesisHardware(ros::NodeHandle nh, ros::NodeHandle private_nh,
                double target_control_freq);

  void updateJointsFromHardware();

  void writeCommandsToHardware();

  void updateDiagnostics();

  void reportLoopDuration(const ros::Duration &duration);

  virtual ~NemesisHardware() override;

private:
  void initilizeCythonThunderborg();

  void initializeDiagnostics();

  void resetTravelOffset();

  void registerControlInterfaces();

  // computes the throttle for the requested angular velocity using a linear
  // model of their relationship, obtained experimentally, of the form y = mx +
  // c where: y = throttle m = grad_  c = 0
  double angularToThrottle(double angle);

  // the gradient of the angular velocity/throttle linear model
  double const rads_to_throttle_grad_ = 0.04;

  void limitDifferentialSpeed(double &travel_speed_left,
                              double &travel_speed_right);

  ros::NodeHandle nh_, private_nh_;

  // ROS Control interfaces
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;

  // cython-thunderborg driver
  PyObject *pName, *pModule, *pDict;

  // Diagnostics
  ros::Publisher diagnostic_publisher_;
  nemesis_msgs::NemesisStatus nemesis_status_msg_;
  diagnostic_updater::Updater diagnostic_updater_;
  NemesisSoftwareDiagnosticTask software_status_task_;
  NemesisBatteryDiagnosticTask battery_status_task_;

  // ROS Parameters
  double wheel_diameter_, min_throttle_, max_throttle_;

  double polling_timeout_;

  /**
   * Joint structure that is hooked to ros_control's InterfaceManager, to allow
   * control via diff_drive_controller
   */
  struct Joint {
    double position;
    double position_offset;
    double velocity;
    double effort;
    double velocity_command;

    Joint() : position(0), velocity(0), effort(0), velocity_command(0) {}
  } joints_[4];
};

} // namespace nemesis_base
#endif // NEMESIS_BASE_NEMESIS_HARDWARE_H
