# Based on the RttestResult from the demos/pendulum_msgs package:
# https://github.com/ros2/demos/blob/master/pendulum_msgs/msg/RttestResults.msg

builtin_interfaces/Time stamp

uint64 current_latency
float64 mean_latency
uint64 min_latency
uint64 max_latency
uint64 minor_pagefaults
uint64 major_pagefaults
