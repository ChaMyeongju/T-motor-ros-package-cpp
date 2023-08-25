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
    int cobe_id_mit = 0;
  }; // namespace MIT

}; // namespace COB-ID

enum Drive_Mode{
  SERVO_MODE,
  MIT_MODE
};

struct Motor_Drive_Info{
  struct Motor_Info{
    double reduction_ratio;
    double torque_constant;
    double rated_torque;
    double peak_torque;
    double rated_current;
    double peak_current;    
  }motor_info;

  struct Drive_Info{
    Drive_Mode drive_mode;
  }drive_info;
};

namespace Motor_spec{
  struct AK80_64{
    double reduction_ratio = 64;
    double torque_constant = 0.136;
    double rated_torque = 48;
    double peak_torque = 120;
    double rated_current = 7;
    double peak_current = 19;
  }ak80_64;

  struct AK70_10{
    double reduction_ratio = 10;
    double torque_constant = 0.123;
    double rated_torque = 8.3;
    double peak_torque = 24.8;
    double rated_current = 7.2;
    double peak_current = 23.2;
  }ak70_10;  
};


class can_base
{
private:

  // CAN information
  int can_port; //
  int bitrate;  //
  bool loopback;  //

  // drive information
  int nDrives;  //
  int *DriveIDs;  //
  Motor_Info *MotorInformations;
  bool *driveStat;
  Drive_Mode *drive_modes;

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
  can_base(int);  //

  /* Basic Information Setting */
  void Can_information(int _nDrives, int *_drive_ID, Motor_Drive_Info *_MotorDriveInforamtions);  //

  /* Low level functions */
  void Open_can();
  void Close_can();
  void Read_frame(CAN_Frame &, int &);
  void Send_frame(CAN_Frame, int);

  // Recieving state CAN message function
  int Recieve_state(answer_format *answer_values);
  int Recieve_state_SERVO(answer_format *answer_values);
  int Recieve_state_MIT(answer_format *answer_values);

  // Sending control CAN message function
  int TargetPosition(int *target_position);
  int TargetPosition_SERVO(int *target_position);
  int TargetPosition_MIT(int *target_position);

  int TargetTorque(double *target_torque);
  int TargetTorque_SERVO(double *target_torque);
  int TargetTorque_MIT(double *target_torque);

  void Convert_to_integer_data(double *raw_data, int *int_data, int flag_type);
  void Convert_to_double_data(double *raw_data, int *int_data, int flag_type);

  class utilities{
  public:
      int Hex2number(char *word);
  };
};

#endif