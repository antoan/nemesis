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

#include "controller_manager/controller_manager.h"

#include "mborg_base/mborg_hardware.h"
#include "ros/callback_queue.h"

#include <boost/chrono.hpp>
#include <ros/console.h>

typedef boost::chrono::steady_clock time_source;

/**
 * Control loop for Mborg, not realtime safe
 */
void controlLoop(mborg_base::MborgHardware &mborg,
                 controller_manager::ControllerManager &cm,
                 time_source::time_point &last_time) {

  // Calculate monotonic time difference
  time_source::time_point this_time = time_source::now();
  boost::chrono::duration<double> elapsed_duration = this_time - last_time;
  ros::Duration elapsed(elapsed_duration.count());
  last_time = this_time;

  // Process control loop
  mborg.reportLoopDuration(elapsed);
  // mborg.updateJointsFromHardware();
  cm.update(ros::Time::now(), elapsed);
  mborg.writeCommandsToHardware();
}

/**
 * Diagnostics loop for Mborg, not realtime safe
 */
void diagnosticLoop(mborg_base::MborgHardware &mborg) {
  mborg.updateDiagnostics();
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "mborg_base");
  ros::NodeHandle nh, private_nh("~");

  double control_frequency, diagnostic_frequency;
  private_nh.param<double>("control_frequency", control_frequency, 10.0);
  private_nh.param<double>("diagnostic_frequency", diagnostic_frequency, 1.0);

  // Initialize robot hardware and link to controller manager
  mborg_base::MborgHardware mborg(nh, private_nh, control_frequency);
  controller_manager::ControllerManager cm(&mborg, nh);

  // Setup separate queue and single-threaded spinner to process timer
  // callbacks  that interface with Mborg hardware
  // threadsafe. This avoids having to lock around hardware access, but
  // precludes realtime safetyb in the control loop.
  ros::CallbackQueue mborg_queue;
  ros::AsyncSpinner mborg_spinner(1, &mborg_queue);
  time_source::time_point last_time = time_source::now();
  ros::TimerOptions control_timer(ros::Duration(1 / control_frequency),
                                  boost::bind(controlLoop, boost::ref(mborg),
                                              boost::ref(cm),
                                              boost::ref(last_time)),
                                  &mborg_queue);
  ros::Timer control_loop = nh.createTimer(control_timer);

  ros::TimerOptions diagnostic_timer(
      ros::Duration(1 / diagnostic_frequency),
      boost::bind(diagnosticLoop, boost::ref(mborg)), &mborg_queue);
  ros::Timer diagnostic_loop = nh.createTimer(diagnostic_timer);

  mborg_spinner.start();

  // Process remainder of ROS callbacks separately, mainly ControlManager
  // related
  ros::spin();

  return 0;
}
