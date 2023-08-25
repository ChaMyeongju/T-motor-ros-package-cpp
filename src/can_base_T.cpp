#include "can_driver_T/can_base_T.hpp"
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <string>
#include <sys/socket.h>
#include <sys/uio.h>

using namespace std;

// frame[0] is LSB
void Bytes_to_decimal(byte *frame, int nbytes, int &decimal)
{
  int signed_var = 0;

  decimal = 0;

  if(frame[nbytes - 1] >> 7 == 1){ /* Is it a negative number? */
    signed_var = 1;
    for(int i = 0; i < nbytes; i++){
      frame[i] = 255 - frame[i];
    }

    frame[0] = frame[0] + 1;

  }

  for(int i = 0; i < nbytes; i++){
    decimal += frame[i]*pow(256,i);
  }

  if(signed_var == 1){
    decimal = -decimal;
  }
}

// frame[0] is LSB
void Decimal_to_bytes(byte *frame, int &nbytes, int decimal)
{
  // cout << "Decimal_to_bytes started" << endl;
  int signed_var = 0;

  if(decimal < 0)
  {
    decimal = -decimal;
    signed_var = 1;
  }

  nbytes = floor((1.0/8.0 * log2(decimal))) + 1;
  int t_dec = decimal;

  for(int i = 0; i < nbytes; i++){
    frame[nbytes- 1 -i] = (t_dec - (t_dec % (int)(pow(256,nbytes- 1 -i)))) >> (int)(8 * (nbytes - 1 -i));
    t_dec = decimal % (int)(pow(256,nbytes- 1 -i));
  }

  if(signed_var == 1){
    for(int i = 0; i < 4; i++){
    frame[i] = 255 - frame[i];
    }

    frame[0] = frame[0]+1;
  }
}

HexaByte Int_to_address(int addr)
{
  HexaByte address;

  address.lowByte = (addr % 256);
  address.HighByte = (addr - address.lowByte) / 256;

  return address;
}

HexaByte Int_to_hex( int i )
{
  HexaByte Hexa;
  char lo;
  char hi;

  if(i <= 255 && i >= 0){
    lo = i - ((i >> 4) << 4);
    hi = i >> 4;
  }else{
    /* Not a byte */
    /* ROS ERROR */
  }

  if(lo >= 0 && lo < 10){
    Hexa.lowByte = lo + 48;
  }else if(lo >= 10 && lo <= 15){
    Hexa.lowByte = lo + 55;
  }

  if(hi >= 0 && hi < 10){
    Hexa.HighByte = hi + 48;
  }else if(hi >= 10 && hi <= 15){
    Hexa.HighByte = hi + 55;
  }

  return Hexa;
}


void Debug_print_frame(CAN_Frame Frame, int nbytes=8)
{
  HexaByte temp;
  cout<<Frame.canID<<"#";
  for(int i = 0; i < nbytes; i++){
    temp = Int_to_hex(Frame.frame_byte[i]);
    cout<<temp.HighByte<<temp.lowByte<<" ";
  }
  cout<<endl;
}

int BitAt(byte b, int index)
{
  return ((b >> (index)) - ((b >> index+1)<<1));
}

void SetBitAt(byte &b, int index, int value)
{
  b = (b + (value<<index));
}

/**
  Constructor for can_base class, initialize bitrates and some specific details
  @arg int Baudrate: Read the bitrate in kbps.
**/
can_base::can_base(int Baudrate)
{
  int PermissibleBauds[] = {10,20,50,125,250,500,1000};
  bool isPermissible = false;

  for(int i = 0; i < 7; i++){
    if(Baudrate == PermissibleBauds[i] || Baudrate == 0){
      isPermissible = true;
      break;
    }
  }

  if(isPermissible == false){
    cout<<"WARNING: Non-Standard baudrate for CANopen implementation"<<endl;
  }

  this->bitrate = Baudrate;

  if(this->bitrate == 0){
    this->loopback = 1;
    cout<<"CAN: CAN assigned to work on loopback"<<endl;
  }else{
    this->loopback = 0;
    cout<<"CAN: CAN assigned to work with "<<this->bitrate<<" kbps"<<endl;
  }
}

void can_base::Can_information(int _nDrives, int *_drive_ID, Motor_Drive_Info *_MotorDriveInformations)
{
  this->nDrives = _nDrives;
  this->DriveIDs = new int[this->nDrives];
  this->MotorInformations = new Motor_Info[this->nDrives];
  this->drive_modes = new Drive_Mode[this->nDrives];

  for(int i = 0; i < nDrives; i++){
    this->DriveIDs[i] = _drive_ID[i];
    this->MotorInformations[i].reduction_ratio = _MotorDriveInformations[i].motor_info.reduction_ratio;
    this->MotorInformations[i].torque_constant = _MotorDriveInformations[i].motor_info.torque_constant;
    this->MotorInformations[i].rated_torque = _MotorDriveInformations[i].motor_info.rated_torque;
    this->MotorInformations[i].peak_torque = _MotorDriveInformations[i].motor_info.peak_torque;
    this->drive_modes[i] = _MotorInformations[i].drive_info.drive_mode;
  }

  this->PositionResolution = 10;
  this->SpeedResolution = 0.1;
  this->CurrentResolution = 1000;

}


/**
  Talk with system to open can0 device and start to work with it
**/
void can_base::Open_can()
{ 
  cout<<"Opening CAN driver ..."<<endl;

//  system("${DRIVER_ELMO_PATH}/scripts/open_can.sh");

  this->s = socket(PF_CAN, SOCK_RAW, CAN_RAW);

  strcpy(this->ifr.ifr_name, "can1");
  ioctl(this->s, SIOCGIFINDEX, &this->ifr);

  this->addr.can_family = AF_CAN;
  this->addr.can_ifindex = this->ifr.ifr_ifindex;

  bind(this->s, (struct sockaddr *) &this->addr, sizeof(this->addr));
  sleep(1);
}

/**
  Talk with shell system and close CAN driver to specific device
**/
void can_base::Close_can()
{
  cout<<"Closing CANopen driver ..."<<endl;

  close(this->s);
}

void can_base::Read_frame(CAN_Frame &UpFrame, int &dlc)
{
  struct can_frame frame;
  volatile int nbytes = 0;
  struct timespec tim, tim2;
  tim.tv_sec = 0;
  tim.tv_nsec = 1000;

  bind(s, (struct sockaddr *) &addr, sizeof(addr));

  nanosleep(&tim , &tim2); /* Workaround */

  while(nbytes == 0){
    nbytes = read(s, &frame, sizeof(struct can_frame));
  }

  if(nbytes < 0){
    perror("can raw socket read");
    return;
  }

  if(nbytes < sizeof(struct can_frame)){
    fprintf(stderr, "read: incomplete CAN frame \n");
  }

  dlc = frame.can_dlc;

  UpFrame.canID = frame.can_id;

  for(int i = 0; i < dlc; i++){
    UpFrame.frame_byte[i] = frame.data[i];
  }

  //close(s);
}

void can_base::Send_frame(CAN_Frame UpcomingFrame, int dlc)
{
  int nbytes;
  struct can_frame frame;

  frame.can_id = UpcomingFrame.canID;
  frame.can_dlc = dlc;

  for(int i = 0; i <= dlc; i++){
    frame.data[i] = UpcomingFrame.frame_byte[i];
  }

  /* send frame */
  if ((nbytes = write(s, &frame, sizeof(frame))) != sizeof(frame)) {
    perror("write");
    return;
  }

  //close(s);

}

int can_base::Recieve_state_servo(answer_format *answer_values)
{
    int limiter = 0;
    int nbytes;
    CAN_Frame *answer_frame;
    answer_frame = new CAN_Frame[this->nDrives];

    byte position_byte[this->nDrives][2];
    byte speed_byte[this->nDrives][2];
    byte current_byte[this->nDrives][2];
    byte temperature_byte[this->nDrives][1];
    byte error_code_byte[this->nDrives][1];

    answer_format_int *answer_values_int;
    answer_values_int = new answer_format_int[this->nDrives];

    for(int i = 0; i < this->nDrives; i++)
    {
      Read_frame(answer_frame[i],nbytes);
      while(answer_frame[i].canID != (2147483648 + 10496 + this->DriveIDs[i])) // 0x80000000 = 2147483648, 0x2900 = 10496, 0x80002900 = 2147494144
      {
        Read_frame(answer_frame[i],nbytes);
        limiter ++;
      }
      if(limiter >= 6)
      {
        cout << "cannot recieve answer within 6 times" << endl;
      }
    }

    for(int i = 0; i < this->nDrives; i++)
    {
      // position
      position_byte[i][0] = answer_frame[i].Servo_Upload.position[1];
      position_byte[i][1] = answer_frame[i].Servo_Upload.position[0];
      Bytes_to_decimal(position_byte[i], 2, answer_values_int[i].position);

      // speed
      speed_byte[i][0] = answer_frame[i].Servo_Upload.speed[1];
      speed_byte[i][1] = answer_frame[i].Servo_Upload.speed[0];
      Bytes_to_decimal(speed_byte[i], 2, answer_values_int[i].speed);

      // current
      current_byte[i][0] = answer_frame[i].Servo_Upload.current[1];
      current_byte[i][1] = answer_frame[i].Servo_Upload.current[0];
      Bytes_to_decimal(current_byte[i], 2, answer_values_int[i].current);

      // temperature
      temperature_byte[i][1] = answer_frame[i].Servo_Upload.motor_temperature;
      // Bytes_to_decimal(temperature_byte[i], 1, answer_values_int[i].motor_temperature);
      answer_values_int[i].motor_temperature = temperature_byte[i][1];

      // error code
      error_code_byte[i][1] = answer_frame[i].Servo_Upload.error_code;
      // Bytes_to_decimal(error_code_byte[i], 1, answer_values_int[i].error_code);
      answer_values_int[i].error_code = error_code_byte[i][1];

      // cout << "position : " << answer_values_int[i].position << " temp : " << answer_values_int[i].motor_temperature << endl;

      answer_values[i].position = ((double)(answer_values_int[i].position))/10;
      answer_values[i].speed = ((double)(answer_values_int[i].speed))/10;
      answer_values[i].torque = ((double)(answer_values_int[i].current))/100*this->MotorInformations[i].reduction_ratio*this->MotorInformations[i].torque_constant;
      answer_values[i].motor_temperature = answer_values_int[i].motor_temperature;
      answer_values[i].error_code = answer_values_int[i].error_code;

    }
} 

int can_base::TargetTorque(double *target_torque)
{

}

int can_base::TargetTorque(double *target_torque)
{
  CAN_Frame current_frame[this->nDrives];
  byte data[this->nDrives][4] = {0};
  int nbytes[this->nDrives];

  int temp_target_current;
  double temp_target_torque[this->nDrives];

  for(int i = 0; i < this->nDrives; i++){
    if(target_torque[i] > this->MotorInformations[i].peak_torque)
    {
      temp_target_torque[i] = this->MotorInformations[i].peak_torque;
    }
    else if(target_torque[i] < -this->MotorInformations[i].peak_torque)
    {
      temp_target_torque[i] = -this->MotorInformations[i].peak_torque;
    }
    else
    {
      temp_target_torque[i] = target_torque[i];
    }

    temp_target_current = (int)(temp_target_torque[i]/(this->MotorInformations[i].reduction_ratio*this->MotorInformations[i].torque_constant)*this->CurrentResolution);
    Decimal_to_bytes(data[i], nbytes[i], temp_target_current);

    current_frame[i].Current_Loop.canID = COB_ID::SERVO::CURRENT_LOOP + this->DriveIDs[i];

    for(int j = 0; j < 4; j++)
    {
      current_frame[i].Current_Loop.current[j] = data[i][j];   
    }
  }

  for(int i = 0; i < this->nDrives; i++){
    Send_frame(current_frame[i],4);
  }

}

int can_base::TargetTorque(double *target_torque)
{
  
}



int can_base::TargetPosition(int *target_position)
{
  CAN_Frame position_frame[this->nDrives];
  byte data[this->nDrives][4] = {0};
  int nbytes[this->nDrives];

  for(int i = 0; i < this->nDrives; i++){

    Decimal_to_bytes(data[i], nbytes[i], target_position[i]);

    position_frame[i].Position_Loop.canID = COB_ID::SERVO::POSITION + this->DriveIDs[i];

    for(int j = 0; j < 4; j++)
    {
      position_frame[i].Position_Loop.position[j] = data[i][j];   
    }
  }

  for(int i = 0; i < this->nDrives; i++){
    Send_frame(position_frame[i],4);
  }

}


void can_base::Convert_to_integer_data(double *raw_data, int *int_data, int flag_type)
{  
  if(flag_type == 1){ /* Position Mode */ /* output-deg */
    for(int i = 0; i < this->nDrives; i++){
      int_data[i] = (int)(raw_data[i] * this->PositionResolution);
    }
  }else if(flag_type == 2){ /* Velocity Mode */ /* output-deg/s */
    for(int i = 0; i < this->nDrives; i++){
      int_data[i] = (int)(raw_data[i] * this->SpeedResolution);
    }
  }else if(flag_type == 3){ /* Torque Mode */ /* Motor Current - A */
    for(int i = 0; i < this->nDrives; i++){
      int_data[i] = (int)(raw_data[i] * this->CurrentResolution);
    }
  }
}


void can_base::Convert_to_double_data(double *raw_data, int *int_data, int flag_type)
{
  for(int i = 0; i < this->nDrives; i++){
    raw_data[3*i]     = (double)int_data[3*i] / this->PositionResolution;   // feedback joint position
    raw_data[3*i + 1] = (double)int_data[3*i+1] / this->SpeedResolution;    // feedback joint velocity
    raw_data[3*i + 2] = (double)int_data[3*i+2] / this->CurrentResolution;  // feedback joint torque
  }
}


int can_base::utilities::Hex2number(char* word)
{
    std::string myhex(word);
    int x = strtoul(myhex.substr(0, 4).c_str(), NULL, 16);

    return x;
}