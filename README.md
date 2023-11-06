this is ROS 1 c++ package for T-motor


<!-- v.1 basic package codes for control T-motor -->
can_base_T.cpp - functions to control T-motor

can_test.cpp - Test code to check the can_base_T.cpp


<!-- v.2(2023.11.06) code for change the reference position and velocity, kp, kd value using keyboard while code is on active -->
can_base_T.cpp

can_test.cpp - subscribing keyboard publication part is added

keyboard_pub.cpp - publish refercne position and velocity, kp, kd value using keyboard.

sub_test.cpp - test code to check whether topic is subscribed appropriately or not.