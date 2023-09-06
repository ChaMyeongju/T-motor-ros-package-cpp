cmake_minimum_required(VERSION 2.8.3)
project(can_driver_t)
set(CMAKE_CXX_FLAGS "-O2")        ## Optimize


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################
## Generate messages in the 'msg' folder
#add_message_files(
#  FILES
#  control_configuration.msg
#  control_feedback.msg
#  feedback_pos_vel_torq.msg
#)

## Generate services in the 'srv' folder
#add_service_files(
#  FILES
#  ControlService.srv
#)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS message_runtime roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/can_base_T.cpp)


add_executable(can_test src/can_test.cpp)
target_link_libraries(can_test ${catkin_LIBRARIES} ${PROJECT_NAME})
add_dependencies(can_test ambpros_can_test_gencpp)
