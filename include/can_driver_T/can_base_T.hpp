#ifndef CAN_BASE_HPP
#define CAN_BASE_HPP

#include "std_msgs/String.h"
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>

typedef unsigned char byte;

struct HexaByte{
  byte lowByte;
  byte HighByte;
};


union CAN_Frame
{
  struct{
    uint32_t canID;
    byte frame_byte[8];
  };

  struct{
    uint32_t canID;
    byte duty_cycle[4];
  }Duty_Cycle;

  struct{
    uint32_t canID;
    byte current[4];
  }Current_Loop;

  struct{
    uint32_t canID;
    byte brake_current[4];
  }Current_Brake;

  struct{
    uint32_t canID;
    byte speed[4];
  }Velocity;

  struct{
    uint32_t canID;
    byte position[4];
  }Position_Loop;

  struct{
    uint32_t canID;
    byte Data;
  }Set_Origin;

  struct{
    uint32_t canID;
    byte position[4];
    byte speed[2];
    byte acceleration[2];
  }Position_Velocity_Loop;

  struct{
    uint32_t canID;
    byte position[2];
    byte speed[2];
    byte current[2];
    byte motor_temperature;
    byte error_code;
  }Servo_Upload;

  struct{
    uint32_t canID;
    byte drive_id;
    byte position[2];
    byte speed_high;
    byte speed_low_current_high;
    byte current_low;
    byte motor_temperature;
    byte error_code;
  }Mit_Upload;

  struct{
    uint32_t canID;
    byte position[2];
    byte speed_high;
    byte speed_low_kp_high;
    byte kp_low;
    byte kd_high;
    byte kd_low_current_high;
    byte current_low;
  }Mit_Download;

};

struct answer_format_int{
  int position;
  int speed;
  int current;
  int motor_temperature;
  int error_code;
};

struct answer_format{
  double position;
  double speed;
  double torque;
  int motor_temperature;
  int error_code;
};

  /* COB-ID for T-motor CAN communication
  /* https://cubemars.com/images/file/20230721/1689907184190344.pdf */

namespace COB_ID
{
  namespace SERVO
  {
    int DUTY_CYCLE = 0;
    int CURRENT_LOOP = 256;
    int CURRENT_BRAKE = 512;
    int VELOCITY = 768;
    int POSITION = 1024;
    int SET_ORIGIN = 1280;
    int POSITION_VELOCITY_LOOP = 1536;
  }; // namespace SERVO

  namespace MIT
  {
    int COB_ID_MIT = 0;
  }; // namespace MIT

}; // namespace COB-ID

enum Drive_Mode{
  SERVO_MODE,
  MIT_MODE
};

enum Motor_Type{
  AK70_10,
  AK80_64
};

struct Motor_Drive_Info{
  struct Motor_Info{
    Motor_Type motor_type;
    double reduction_ratio;
    double torque_constant;
    double rated_torque;
    double peak_torque;
    double rated_current;
    double peak_current;    
    double peak_speed;
    double peak_position;
  }motor_info;

  struct Drive_Info{
    Drive_Mode drive_mode;
  }drive_info;
};




namespace Motor_spec{
  struct AK80_64{
    double reduction_ratio = 64;    
    double torque_constant = 0.136; // Nm/A
    double rated_torque = 48;       // Nm
    double peak_torque = 144;       // Nm
    double rated_current = 7;       // A
    double peak_current = 19;       // A
    double peak_speed = 8;          // rad/s
    double peak_position = 12.5;    // rad
  }ak80_64;

  struct AK70_10{
    double reduction_ratio = 10;
    double torque_constant = 0.123; // Nm/A
    double rated_torque = 8.3;      // Nm
    double peak_torque = 25;      // Nm
    double rated_current = 7.2;     // A
    double peak_current = 23.2;     // A
    double peak_speed = 50;          // rad/s
    double peak_position = 12.5;    // rad
  }ak70_10;  
};


class can_base
{
private:

  // CAN information

  // drive information
  int nDrives;  //
  int *DriveIDs;  //
  Motor_Drive_Info *MotorDriveInformations;
  bool *driveStat;

  // socket
  int s; /* socket */
  int nbytes;
  struct sockaddr_can addr;
  struct ifreq ifr;

  // motor specification
  double PositionResolution;
  double SpeedResolution;
  double CurrentResolution;


public:
  can_base(int _nDrives, int *_drive_ID, Motor_Type *_motor_type, Drive_Mode *_drive_modes);  //

  /* Show the informations about motors and drives */
  void Show_information();  //

  /* Low level functions */
  void Open_can();
  void Close_can();
  void Read_frame(CAN_Frame &, int &);
  void Send_frame(CAN_Frame, int);

  // Motor on/off
  void MotorOn();
  void MotorOff();

  // Recieving state CAN message function
  int Recieve_state(answer_format *answer_values);
  int Recieve_state_SERVO(answer_format &answer_values, int _drive_ID, Motor_Drive_Info _MotorDriveInformations);
  int Recieve_state_MIT(answer_format &answer_values, int _drive_ID, Motor_Drive_Info _MotorDriveInformations);

  // Sending control CAN message function
  int TargetPosition(int *target_position);
  // int TargetPosition_SERVO(int *target_position);
  // int TargetPosition_MIT(int *target_position);

  int TargetTorque(double *target_torque);
  int TargetTorqueFrame(CAN_Frame &current_frame, double target_torque, int _drive_ID, Motor_Drive_Info _MotorDriveInformations);

  void Convert_to_byte_data_SERVO(double raw_data, byte *byte_data, int flag_type, Motor_Drive_Info _MotorDriveInformations);
  void Convert_to_byte_torque_MIT(byte *frame, double torque, Motor_Drive_Info _MotorDriveInformations);
  void Convert_to_byte_position_MIT(byte *frame, double kp, double kd, double ref_pos, double ref_speed, Motor_Drive_Info _MotorDriveInformations);
  void Double_to_bytes_speed_MIT(byte *frame, double kd, double ref_speed, Motor_Drive_Info _MotorDriveInformations);

  class utilities{
  public:
      int Hex2number(char *word);
  };
};

#endif