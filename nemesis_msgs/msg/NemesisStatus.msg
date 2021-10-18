Header header

# Uptime, in ms
uint32 uptime

# ROS Control loop frequency
float64 ros_control_loop_freq

# Control Velocities
float64 angular_vel_request_left
float64 angular_vel_request_right
float64 throttle_left
float64 throttle_right
float64 throttle_limited_left
float64 throttle_limited_right

# Voltage of platform components, in volts
float32 battery_voltage

# Error/Stop conditions
bool timeout
bool lockout
bool e_stop
bool ros_pause
bool no_battery
