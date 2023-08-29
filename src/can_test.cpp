#include "can_driver_T/can_base_T.hpp"
#include <sstream>
#include "ros/ros.h"
#include "cmath"
#include <iostream>

using namespace std;

#define MOTOR_ID 1


void MD_parameter_setting(int *drive_ids_arr, Motor_Type *motor_types_arr, Drive_Mode *drive_modes_arr, int _drive_id, Motor_Type _motor_type, Drive_Mode _drive_mode, int arr_index)
{
  drive_ids_arr[arr_index] = _drive_id;
  motor_types_arr[arr_index] = _motor_type;
  drive_modes_arr[arr_index] = _drive_mode;
}


int main(int argc, char **argv)
{
  ros::init(argc,argv,"can_test");
  ros::NodeHandle n;

  double rate = 200;
  ros::Rate loop_rate(rate);

  int flag;

  int clock = 0;

  int number_drivers = 1;
  int *current_driver_IDs;
  Motor_Type *motor_types;
  Drive_Mode *drive_modes;

  current_driver_IDs = new int[number_drivers];
  motor_types = new Motor_Type[number_drivers];
  drive_modes = new Drive_Mode[number_drivers];

  // MD_parameter_setting(current_driver_IDs, motor_types, drive_modes, 1, Motor_Type::AK80_64, Drive_Mode::SERVO_MODE, 0);
  MD_parameter_setting(current_driver_IDs, motor_types, drive_modes, 1, Motor_Type::AK80_64, Drive_Mode::MIT_MODE, 0);

  // MD_parameter_setting(current_driver_IDs, motor_types, drive_modes, 1, Motor_Type::AK80_64, Drive_Mode::SERVO_MODE, 1);

  // cout << current_driver_IDs[1] << endl;
  // cout << motor_types[1] << endl;
  // cout << drive_modes[1] << endl;

  can_base CanDriver(number_drivers, current_driver_IDs, motor_types, drive_modes);


  double *reference;
  double *target_torque, *target_speed;
  reference = new double[number_drivers];
  target_torque = new double[number_drivers];
  target_speed = new double[number_drivers];
  for (int i = 0; i < number_drivers; i++){
    reference[i] = 1.57;
    target_torque[i] = 0.615;
  }

  answer_format *answer;
  answer = new answer_format[number_drivers];

  sleep(4);


  ROS_INFO("START");

  double t_start,t_end,t_diff;

  while(ros::ok()){
    // sleep(1);


    // CanDriver.TargetPosition(reference);
    t_start = ros::Time::now().toSec();
    CanDriver.Recieve_state(answer);
    t_end = ros::Time::now().toSec();
    t_diff = t_start - t_end;
    // target_torque[0] = 0.05*(reference[0] - answer[0].position) + 0.02*(target_speed[0] - answer[0].speed);
    target_torque[0] = 100*(reference[0] - answer[0].position);
    CanDriver.TargetTorque(target_torque);
    // ROS_INFO("real torque : %f || target torque : %f || position : %f",answer[0].torque,target_torque[0],answer[0].position);
    ROS_INFO("time : %f || position : %f || speed : %f || torque : %f || temperature : %d",t_diff,answer[0].position,answer[0].speed,answer[0].torque,answer[0].motor_temperature);
    // cout << "Position : " << answer[0].position << " speed : " << answer[0].speed << " torque : " << answer[0].torque << " temperature : " << answer[0].motor_temperature << endl;
    ros::spinOnce();
    loop_rate.sleep();

  }


  CanDriver.MotorOff();
  CanDriver.Close_can();
  return 0;
}
