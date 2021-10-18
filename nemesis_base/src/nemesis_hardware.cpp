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

#include "nemesis_base/nemesis_hardware.h"
#include <boost/assign/list_of.hpp>

namespace {
const uint8_t LEFT = 0, RIGHT = 1;
};

namespace nemesis_base {

/**
 * Initialize Nemesis hardware
 */
NemesisHardware::NemesisHardware(ros::NodeHandle nh, ros::NodeHandle private_nh,
                             double target_control_freq)
    : nh_(nh), private_nh_(private_nh),
      software_status_task_(nemesis_status_msg_, target_control_freq),
      battery_status_task_(nemesis_status_msg_) {

  private_nh_.param<double>("wheel_diameter", wheel_diameter_, 0.105);
  private_nh_.param<double>("max_throttle", max_throttle_, 1.0);
  private_nh_.param<double>("min_throttle", min_throttle_, 0.3);
  private_nh_.param<double>("polling_timeout_", polling_timeout_, 10.0);

  initilizeCythonThunderborg();

  initializeDiagnostics();
  registerControlInterfaces();
}

/**
 * Initialize Cython-Thunderborg driver.
 */
void NemesisHardware::initilizeCythonThunderborg() {

  // Initialize cython-thunderborg

  // Py_SetProgramName(argv[0]); /* optional but recommended */

  auto err = PyImport_AppendInittab("ThunderBorg", initThunderBorg);

  if (err) {
    ROS_ERROR("Cython-Thunderborg PyImport_AppendInittab Failed.");
    return;
  }

  Py_Initialize();

  std::string strModule = "ThunderBorg"; // module to be loaded
  pName = PyString_FromString(strModule.c_str());

  pModule = PyImport_Import(pName);

  if (pModule == NULL) {
    ROS_ERROR("Cython-Thunderborg module failed to load.");
    return;
  }

  if (!InitTB())
    ROS_ERROR("Cython-Thunderborg Initialization Failure");
}

/**
 * Register diagnostic tasks with updater class
 */
void NemesisHardware::initializeDiagnostics() {

  diagnostic_updater_.setHardwareID("Monsterborg");
  diagnostic_updater_.add(software_status_task_);
  diagnostic_updater_.add(battery_status_task_);
  diagnostic_publisher_ = nh_.advertise<nemesis_msgs::NemesisStatus>("status", 10);
}

/**
 * Register interfaces with the RobotHW interface manager, allowing
 * ros_control operation
 */
void NemesisHardware::registerControlInterfaces() {
  ros::V_string joint_names = boost::assign::list_of("front_left_wheel")(
      "front_right_wheel")("rear_left_wheel")("rear_right_wheel");
  for (unsigned int i = 0; i < joint_names.size(); i++) {
    hardware_interface::JointStateHandle joint_state_handle(
        joint_names[i], &joints_[i].position, &joints_[i].velocity,
        &joints_[i].effort);
    joint_state_interface_.registerHandle(joint_state_handle);

    hardware_interface::JointHandle joint_handle(joint_state_handle,
                                                 &joints_[i].velocity_command);
    velocity_joint_interface_.registerHandle(joint_handle);
  }
  registerInterface(&joint_state_interface_);
  registerInterface(&velocity_joint_interface_);
}

/**
 * External hook to trigger diagnostic update
 */
void NemesisHardware::updateDiagnostics() {

  battery_status_task_.updateBatteryStatus(GetBatteryReadingWrapper());

  diagnostic_updater_.force_update();
  nemesis_status_msg_.header.stamp = ros::Time::now();
  diagnostic_publisher_.publish(nemesis_status_msg_);
}

/**
 * Pull latest speed and travel measurements from Thunderborg hardware, and
 * store in joint structure for ros_control
 */
/**
void NemesisHardware::updateJointsFromHardware() {

  // TODO: Stub - convert from throttle level to rads/s, cast to double and
  // write to joints_
  // float power2 = GetMotor2Wrapper();
  // float power1 = GetMotor1Wrapper();
  // ROS_DEBUG_STREAM("Power 1:" << power1 << " Power 2:" << power2);
}
*/

/**
 * Get latest velocity commands from ros_control via joint structure, and send
 * to Tunderborg hardware.
 */
void NemesisHardware::writeCommandsToHardware() {

  double angular_vel_request_left = joints_[LEFT].velocity_command;
  double angular_vel_request_right = joints_[RIGHT].velocity_command;

  // ROS_DEBUG_STREAM("angular_vel_request_left:" << angular_vel_request_left
  //                                              << "
  //                                              angular_vel_request_right"
  //                                              << angular_vel_request_right);

  double throttle_left = angularToThrottle(angular_vel_request_left);
  double throttle_right = angularToThrottle(angular_vel_request_right);

  double throttle_limited_left = throttle_left;
  double throttle_limited_right = throttle_right;

  limitDifferentialSpeed(throttle_limited_left, throttle_limited_right);

  software_status_task_.reportControlVelocities(
      angular_vel_request_left, angular_vel_request_right, throttle_left,
      throttle_right, throttle_limited_left, throttle_limited_right);

  // Keep For Manual Testing ONLY
  // if (!SetMotor1Wrapper(0.5))
  //   ROS_ERROR("Error sending speed command: Motor 1");
  // if (!SetMotor2Wrapper(0.0))
  //   ROS_ERROR("Error sending speed command: Motor 2");

  if (!SetMotor1Wrapper(throttle_limited_right))
    ROS_ERROR("Error sending speed command: Motor 1");
  if (!SetMotor2Wrapper(throttle_limited_left))
    ROS_ERROR("Error sending speed command: Motor 2");
}

/**
 * Update diagnostics with control loop timing information
 */
void NemesisHardware::reportLoopDuration(const ros::Duration &duration) {
  software_status_task_.updateControlFrequency(1 / duration.toSec());
}

/**
 * Scale left and right speed outputs to maintain ros_control's desired
 * trajectory without saturating the outputs
 */
void NemesisHardware::limitDifferentialSpeed(double &diff_speed_left,
                                           double &diff_speed_right) {
  double large_speed =
      std::max(std::abs(diff_speed_left), std::abs(diff_speed_right));

  if (large_speed > max_throttle_) {
    diff_speed_left *= max_throttle_ / large_speed;
    diff_speed_right *= max_throttle_ / large_speed;
  }
}

/**
 * need radians for ros_control RobotHW
 */
// double NemesisHardware::linearToAngular(const double &travel) const {
//   return travel / wheel_diameter_ * 2;
// }

/**
 * RobotHW provides velocity command in rad/s, Nemesis needs m/s,
 */
// double NemesisHardware::angularToLinear(const double &angle) const {
//   // TODO: Map rads/s to Thunderborg throttle level here
//   return angle;
// }

double NemesisHardware::angularToThrottle(double angle) {
  // computes the throttle as a linear function of the requested anglular
  // velocity, of the form y = mx + c where: y = throttle,  m = grad_ ,  c = 0
  return rads_to_throttle_grad_ * angle;
}

NemesisHardware::~NemesisHardware() {

  /* Clean up after using CPython. */
  // PyMem_RawFree(argv[0]);
  // Py_DECREF(args);

  SetMotor1Wrapper(0.0);
  SetMotor2Wrapper(0.0);

  // SetMotor1Wrapper(borg, 0.0);

  Py_DECREF(pModule);
  Py_DECREF(pName);
  Py_Finalize();
}

} // namespace nemesis_base
