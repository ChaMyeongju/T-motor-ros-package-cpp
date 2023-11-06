#include "can_driver_T/can_base_T.hpp"
#include <sstream>
#include "ros/ros.h"
#include "cmath"
#include <iostream>

#include "can_driver_t/control_param.h"
// #include "can_driver_t/control_param_srv.h"

using namespace std;

#define MOTOR_ID 1

double *reference_pos, *reference_speed;
double *kp_val, *kd_val;
double *target_torque;
int number_drivers;


void control_param_subscribe(const can_driver_t::control_param::ConstPtr& msg)
{
  cout << "msg recieved" << endl;

}

// bool control_param_setting(can_driver_t::control_param_srv::Request &req, can_driver_t::control_param_srv::Response &res)
// {
//   res.stop_flag     = req.stop_flag;   
//   res.init_flag     = req.init_flag;  
//   res.control_mode  = req.control_mode; 
//   res.ref_pos       = req.ref_pos;      
//   res.ref_vel       = req.ref_vel;     
//   res.kp            = req.kp;          
//   res.kd            = req.kd;

//   return true;          
// }



int main(int argc, char **argv)
{
  ros::init(argc,argv,"sub_test");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe<can_driver_t::control_param>("/control_param_msg", 1000, control_param_subscribe);


  ROS_INFO("START");



  ros::spin();



  return 0;
}
