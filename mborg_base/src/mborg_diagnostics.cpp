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

#include "mborg_base/mborg_diagnostics.h"

namespace {

const float UNDERVOLT_ERROR = 7.0;
const float UNDERVOLT_WARN = 10.0;
const int CONTROLFREQ_WARN = 90;
// const int OVERVOLT_ERROR = 30;
// const int OVERVOLT_WARN = 29;
// const double LOWPOWER_ERROR = 0.2;
// const double LOWPOWER_WARN = 0.3;
// const unsigned int SAFETY_TIMEOUT = 0x1;
// const unsigned int SAFETY_LOCKOUT = 0x2;
// const unsigned int SAFETY_ESTOP = 0x8;
// const unsigned int SAFETY_WARN = (SAFETY_TIMEOUT | SAFETY_CCI | SAFETY_PSU);
// const unsigned int SAFETY_ERROR =
//     (SAFETY_LOCKOUT | SAFETY_ESTOP | SAFETY_CURRENT);
} // namespace

namespace mborg_base {

MborgSoftwareDiagnosticTask::MborgSoftwareDiagnosticTask(
    mborg_msgs::MborgStatus &msg, double target_control_freq)
    : DiagnosticTask("software_status"), msg_(msg),
      target_control_freq_(target_control_freq) {
  reset();
}

void MborgSoftwareDiagnosticTask::updateControlFrequency(double frequency) {

  // Keep minimum observed frequency for diagnostics purposes
  control_freq_ = std::min(control_freq_, frequency);
}

void MborgSoftwareDiagnosticTask::reportControlVelocities(
    double angular_vel_request_left, double angular_vel_request_right,
    double throttle_left, double throttle_right, double throttle_limited_left,
    double throttle_limited_right) {

  angular_vel_request_left_ = angular_vel_request_left;
  angular_vel_request_right_ = angular_vel_request_right;
  throttle_left_ = throttle_left;
  throttle_right_ = throttle_right;
  throttle_limited_left_ = throttle_limited_left;
  throttle_limited_right_ = throttle_limited_right;
}

void MborgSoftwareDiagnosticTask::run(
    diagnostic_updater::DiagnosticStatusWrapper &stat) {

  msg_.ros_control_loop_freq = control_freq_;
  stat.add("ROS Control Loop Frequency", msg_.ros_control_loop_freq);

  double margin = control_freq_ / target_control_freq_ * 100;

  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Software OK");
  if (margin < CONTROLFREQ_WARN) {
    std::ostringstream message;
    message << "Control loop executing " << 100 - static_cast<int>(margin)
            << "% slower than desired";
    stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, message.str());
  }

  msg_.angular_vel_request_left = angular_vel_request_left_;
  msg_.angular_vel_request_right = angular_vel_request_right_;
  msg_.throttle_left = throttle_left_;
  msg_.throttle_right = throttle_right_;
  msg_.throttle_limited_left = throttle_limited_left_;
  msg_.throttle_limited_right = throttle_limited_right_;

  reset();
}

void MborgSoftwareDiagnosticTask::reset() {
  control_freq_ = std::numeric_limits<double>::infinity();
  target_control_freq_ = 0;
}

MborgBatteryDiagnosticTask::MborgBatteryDiagnosticTask(
    mborg_msgs::MborgStatus &msg)
    : DiagnosticTask("battery_status"), msg_(msg) {}

void MborgBatteryDiagnosticTask::updateBatteryStatus(float voltage) {

  battery_voltage_ = voltage;
}

void MborgBatteryDiagnosticTask::run(
    diagnostic_updater::DiagnosticStatusWrapper &stat) {

  msg_.battery_voltage = battery_voltage_;

  stat.add("Thunderborg Battery Voltage", msg_.battery_voltage);

  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Battery Voltage OK");

  if (battery_voltage_ < UNDERVOLT_WARN) {
    std::ostringstream message;
    message << "Low Battery";
    stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, message.str());
  }
}

} // namespace mborg_base
