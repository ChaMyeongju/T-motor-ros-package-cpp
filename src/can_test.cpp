#include "can_driver_T/can_base_T.hpp"
#include <sstream>
#include "ros/ros.h"
#include "cmath"
#include <iostream>

using namespace std;

#define MOTOR_ID 1


Motor_Type Motor_type_setting(int motor_type_flag)
{
  if(motor_type_flag == AK70_10)
    return AK70_10;
  else if(motor_type_flag == AK80_64)
    return AK80_64;
}

Drive_Mode Drive_mode_setting(int drive_mode_flag)
{
  if(drive_mode_flag == SERVO_MODE)
    return SERVO_MODE;
  else if(drive_mode_flag == MIT_MODE)
    return MIT_MODE;
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"can_test");
  ros::NodeHandle n;

  double rate;
  ros::param::get("/ros_param/rate",rate);

  ros::Rate loop_rate(rate);

  int flag;

  int clock = 0;

  int number_drivers;
  int *current_driver_IDs;
  Motor_Type *motor_types;
  Drive_Mode *drive_modes;

  ros::param::get("/t_motor/n_drives",number_drivers);

  current_driver_IDs = new int[number_drivers];
  motor_types = new Motor_Type[number_drivers];
  drive_modes = new Drive_Mode[number_drivers];


  std::vector<int> temp_driver_ids;
  std::vector<int> temp_motor_types;
  std::vector<int> temp_drive_modes;


  ros::param::get("/t_motor/motor/drive_ids",temp_driver_ids);
  ros::param::get("/t_motor/motor/motor_types",temp_motor_types);
  ros::param::get("/t_motor/motor/drive_modes",temp_drive_modes);    

  for(int i = 0; i < number_drivers; i++)
  {
    current_driver_IDs[i] = temp_driver_ids[i];
    motor_types[i] = Motor_type_setting(temp_motor_types[i]);
    drive_modes[i] = Drive_mode_setting(temp_drive_modes[i]);
  }

  can_base CanDriver(number_drivers, current_driver_IDs, motor_types, drive_modes);


  double *reference_pos, *reference_speed;
  double *kp_val, *kd_val;
  double *target_torque;
  reference_pos = new double[number_drivers];
  reference_speed = new double[number_drivers];
  kp_val = new double[number_drivers];
  kd_val = new double[number_drivers];
  target_torque = new double[number_drivers];


  std::vector<double> temp_ref_pos;
  std::vector<double> temp_ref_speed;
  std::vector<double> temp_kp;
  std::vector<double> temp_kd;

  ros::param::get("/t_motor/control/reference_pos",temp_ref_pos);
  ros::param::get("/t_motor/control/reference_speed",temp_ref_speed);
  ros::param::get("/t_motor/control/kp",temp_kp);
  ros::param::get("/t_motor/control/kd",temp_kd);

  for (int i = 0; i < number_drivers; i++){
    reference_pos[i] = temp_ref_pos[i];
    reference_speed[i] = temp_ref_speed[i];
    kp_val[i] = temp_kp[i];
    kd_val[i] = temp_kd[i];
  }

  int torquePD_flag = 0;
  ros::param::get("/t_motor/control/torquePD_flag",torquePD_flag);  


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

    for(int i = 0; i < number_drivers; i++)
    {
      target_torque[i] = kp_val[i]*(reference_pos[i] - answer[i].position) + kd_val[i]*(reference_speed[i] - answer[i].speed);      
    }

    if(torquePD_flag == 1)
      CanDriver.TargetTorque(target_torque);
    else if(torquePD_flag == 2)
      CanDriver.TargetTorqueWithPD(target_torque, kp_val, reference_pos, kd_val, reference_speed);

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
