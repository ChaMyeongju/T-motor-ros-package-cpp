#include "can_driver_T/can_base_T.hpp"
#include <sstream>
#include "ros/ros.h"
#include "cmath"
#include <iostream>

using namespace std;

#define MOTOR_ID 1


double *reference_container;
int number_drivers = 1;
int *current_driver_ID;
Motor_Info *motor_info;
// AK70_10 ak70_10;
// AK80_64 ak80_64;

int can_baud = 1000;

void configuration_handler(can_base *CanDriver)
{

  reference_container = new double[number_drivers];
  current_driver_ID = new int[number_drivers];

  motor_info = new Motor_Info[number_drivers];
  // motor_info[0].reduction_ratio = ak70_10.reduction_ratio;
  // motor_info[0].torque_constant = ak70_10.torque_constant;
  // motor_info[0].rated_torque = ak70_10.rated_torque;
  // motor_info[0].peak_torque = ak70_10.peak_torque;


  for(int i = 0; i < number_drivers; i++){
    current_driver_ID[i] = 1 + i;
    motor_info[i].reduction_ratio = Motor_spec::ak70_10.reduction_ratio;
    motor_info[i].torque_constant = Motor_spec::ak70_10.torque_constant;
    motor_info[i].rated_torque = Motor_spec::ak70_10.rated_torque;
    motor_info[i].peak_torque = Motor_spec::ak70_10.peak_torque;
    motor_info[i].rated_current = Motor_spec::ak70_10.rated_current;
    motor_info[i].peak_current = Motor_spec::ak70_10.peak_current;

  }

  std::cout<<"Configuring!!"<<std::endl;
  CanDriver->Can_information(number_drivers, current_driver_ID,motor_info);

}


int main(int argc, char **argv)
{
  ros::init(argc,argv,"can_test");
  ros::NodeHandle n;

  double rate = 200;
  ros::Rate loop_rate(rate);

  int flag;

  int clock = 0;


  can_base CanDriver(can_baud); // 1000 Kbps

  CanDriver.Open_can();
  configuration_handler(&CanDriver);

  double *reference;
  double *target_torque, *target_speed;
  reference = new double[number_drivers];
  target_torque = new double[number_drivers];
  target_speed = new double[number_drivers];
  for (int i = 0; i < number_drivers; i++){
    reference[i] = 0;
    target_torque[i] = 0.615;
  }

  answer_format *answer;
  answer = new answer_format[number_drivers];

  sleep(4);


  ROS_INFO("START");

  ros::Time t_start,t_end;
  ros::Duration t_diff;

  while(ros::ok()){
    // sleep(1);


    // CanDriver.TargetPosition(reference);
    t_start = ros::Time::now();
    CanDriver.Recieve_state(answer);
    t_end = ros::Time::now();
    // target_torque[0] = 0.05*(reference[0] - answer[0].position) + 0.02*(target_speed[0] - answer[0].speed);
    target_torque[0] = 0.01*(reference[0] - answer[0].position);
    // CanDriver.TargetTorque(target_torque);
    ROS_INFO("real torque : %f || target torque : %f || position : %f",answer[0].torque,target_torque[0],answer[0].position);
    // ROS_INFO("position : %f || speed : %f || torque : %f || temperature : %d",answer[0].position,answer[0].speed,answer[0].torque,answer[0].motor_temperature);
    // cout << "Position : " << answer[0].position << " speed : " << answer[0].speed << " torque : " << answer[0].torque << " temperature : " << answer[0].motor_temperature << endl;
    ros::spinOnce();
    loop_rate.sleep();

  }

  CanDriver.Close_can();
  return 0;
}
