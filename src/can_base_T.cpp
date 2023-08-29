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
can_base::can_base(int _nDrives, int *_drive_ID, Motor_Type *_motor_type, Drive_Mode *_drive_modes)
{
  this->nDrives = _nDrives;
  this->DriveIDs = new int[this->nDrives];
  this->MotorDriveInformations = new Motor_Drive_Info[this->nDrives];

  cout << "Setting the parameters of motors and drives" << endl;

  for(int i = 0; i < nDrives; i++){
    this->DriveIDs[i] = _drive_ID[i];
    cout << _drive_ID[i] << " || " << this->DriveIDs[i] << endl;
    this->MotorDriveInformations[i].motor_info.motor_type = _motor_type[i];
    this->MotorDriveInformations[i].drive_info.drive_mode = _drive_modes[i];
    if(_motor_type[i] == AK70_10)
    {
      // cout << "ak70_10" << endl;
      this->MotorDriveInformations[i].motor_info.reduction_ratio = Motor_spec::ak70_10.reduction_ratio;
      this->MotorDriveInformations[i].motor_info.torque_constant = Motor_spec::ak70_10.torque_constant;
      this->MotorDriveInformations[i].motor_info.rated_torque = Motor_spec::ak70_10.rated_torque;
      this->MotorDriveInformations[i].motor_info.peak_torque = Motor_spec::ak70_10.peak_torque;
      this->MotorDriveInformations[i].motor_info.rated_current = Motor_spec::ak70_10.rated_current;
      this->MotorDriveInformations[i].motor_info.peak_current = Motor_spec::ak70_10.peak_current;
      this->MotorDriveInformations[i].motor_info.peak_speed = Motor_spec::ak70_10.peak_speed;
      this->MotorDriveInformations[i].motor_info.peak_position = Motor_spec::ak70_10.peak_position;
    }

    if(_motor_type[i] == AK80_64)
    {
      // cout << "ak80_64"  << endl;
      this->MotorDriveInformations[i].motor_info.reduction_ratio = Motor_spec::ak80_64.reduction_ratio;
      this->MotorDriveInformations[i].motor_info.torque_constant = Motor_spec::ak80_64.torque_constant;
      this->MotorDriveInformations[i].motor_info.rated_torque = Motor_spec::ak80_64.rated_torque;
      this->MotorDriveInformations[i].motor_info.peak_torque = Motor_spec::ak80_64.peak_torque;
      this->MotorDriveInformations[i].motor_info.rated_current = Motor_spec::ak80_64.rated_current;
      this->MotorDriveInformations[i].motor_info.peak_current = Motor_spec::ak80_64.peak_current;
      this->MotorDriveInformations[i].motor_info.peak_speed = Motor_spec::ak80_64.peak_speed;
      this->MotorDriveInformations[i].motor_info.peak_position = Motor_spec::ak80_64.peak_position;
    }

  }

  this->PositionResolution = 10;
  this->SpeedResolution = 0.1;
  this->CurrentResolution = 1000;  

  cout << "Parameters setting is done" << endl << endl;

  Show_information();
  Open_can();
  MotorOn();
}

void can_base::Show_information()
{
  cout << "These are the informations of each motor and drive" << endl;
  for(int i = 0; i < this->nDrives; i++)
  {
    cout << "Motor " << i+1 << endl;
    cout << "CAN ID : " << this->DriveIDs[i] << endl;
    if(this->MotorDriveInformations[i].motor_info.motor_type == Motor_Type::AK70_10)
      cout << "Motor type : AK70-10" << endl;
    else if(this->MotorDriveInformations[i].motor_info.motor_type == Motor_Type::AK80_64)
      cout << "Motor type : AK80-64" << endl;
    if(this->MotorDriveInformations[i].drive_info.drive_mode == Drive_Mode::SERVO_MODE)
      cout << "Drive mode : SERVO"<< endl;
    if(this->MotorDriveInformations[i].drive_info.drive_mode == Drive_Mode::MIT_MODE)
      cout << "Drive mode : MIT"<< endl;    
    cout << "Reduction ratio : " << this->MotorDriveInformations[i].motor_info.reduction_ratio << endl;
    cout << "Torque constant : " << this->MotorDriveInformations[i].motor_info.torque_constant << " Nm/A" << endl;
    cout << "Rated torque : " << this->MotorDriveInformations[i].motor_info.rated_torque << " Nm" << endl;
    cout << "Peak torque : " << this->MotorDriveInformations[i].motor_info.peak_torque << " Nm" << endl;
    cout << "Rated current : " << this->MotorDriveInformations[i].motor_info.rated_current << " A" << endl;
    cout << "Peak current: " << this->MotorDriveInformations[i].motor_info.peak_current << " A" << endl;
    cout << "Peak speed(for MIT) : " << this->MotorDriveInformations[i].motor_info.peak_speed << " rad/s" << endl;
    cout << "Peak position(for MIT) : " << this->MotorDriveInformations[i].motor_info.peak_position << " rad" << endl << endl;

  }
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


void can_base::MotorOn()
{
  CAN_Frame *enter_control_mode_frame;
  enter_control_mode_frame = new CAN_Frame[this->nDrives];

  for(int i = 0; i < this->nDrives; i++)
  {
    enter_control_mode_frame[i].canID = COB_ID::MIT::COB_ID_MIT + this->DriveIDs[i];
    enter_control_mode_frame[i].frame_byte[0] = 255;
    enter_control_mode_frame[i].frame_byte[1] = 255;
    enter_control_mode_frame[i].frame_byte[2] = 255;
    enter_control_mode_frame[i].frame_byte[3] = 255;
    enter_control_mode_frame[i].frame_byte[4] = 255;
    enter_control_mode_frame[i].frame_byte[5] = 255;
    enter_control_mode_frame[i].frame_byte[6] = 255;
    enter_control_mode_frame[i].frame_byte[7] = 252;
    Send_frame(enter_control_mode_frame[i],8);
  }

}

void can_base::MotorOff()
{
  CAN_Frame *enter_control_mode_frame;
  enter_control_mode_frame = new CAN_Frame[this->nDrives];

  for(int i = 0; i < this->nDrives; i++)
  {
    enter_control_mode_frame[i].canID = COB_ID::MIT::COB_ID_MIT + this->DriveIDs[i];
    enter_control_mode_frame[i].frame_byte[0] = 255;
    enter_control_mode_frame[i].frame_byte[1] = 255;
    enter_control_mode_frame[i].frame_byte[2] = 255;
    enter_control_mode_frame[i].frame_byte[3] = 255;
    enter_control_mode_frame[i].frame_byte[4] = 255;
    enter_control_mode_frame[i].frame_byte[5] = 255;
    enter_control_mode_frame[i].frame_byte[6] = 255;
    enter_control_mode_frame[i].frame_byte[7] = 253;
    Send_frame(enter_control_mode_frame[i],8);
  }  
}


int can_base::Recieve_state(answer_format *answer_values)
{
  for(int i = 0; i < this->nDrives; i++){
    if(this->MotorDriveInformations[i].drive_info.drive_mode == Drive_Mode::SERVO_MODE)
      Recieve_state_SERVO(answer_values[i],this->DriveIDs[i],this->MotorDriveInformations[i]);
    else if(this->MotorDriveInformations[i].drive_info.drive_mode == Drive_Mode::MIT_MODE)
      Recieve_state_MIT(answer_values[i],this->DriveIDs[i],this->MotorDriveInformations[i]);
  }
}

int can_base::Recieve_state_SERVO(answer_format &answer_values, int _drive_ID, Motor_Drive_Info _MotorDriveInformations)
{
  int limiter = 0;
  int nbytes;
  CAN_Frame answer_frame;

  byte position_byte[2];
  byte speed_byte[2];
  byte current_byte[2];
  byte temperature_byte;
  byte error_code_byte;

  answer_format_int answer_values_int;

  Read_frame(answer_frame,nbytes);
  while(answer_frame.canID != (2147483648 + 10496 + _drive_ID)) // 0x80000000 = 2147483648, 0x2900 = 10496, 0x80002900 = 2147494144
  {
    Read_frame(answer_frame,nbytes);
    limiter ++;
  }
  if(limiter >= 6)
  {
    cout << "cannot recieve answer within 6 times" << endl;
  }

  // position
  position_byte[0] = answer_frame.Servo_Upload.position[1];
  position_byte[1] = answer_frame.Servo_Upload.position[0];
  Bytes_to_decimal(position_byte, 2, answer_values_int.position);

  // speed
  speed_byte[0] = answer_frame.Servo_Upload.speed[1];
  speed_byte[1] = answer_frame.Servo_Upload.speed[0];
  Bytes_to_decimal(speed_byte, 2, answer_values_int.speed);

  // current
  current_byte[0] = answer_frame.Servo_Upload.current[1];
  current_byte[1] = answer_frame.Servo_Upload.current[0];
  Bytes_to_decimal(current_byte, 2, answer_values_int.current);

  // temperature
  temperature_byte = answer_frame.Servo_Upload.motor_temperature;
  answer_values_int.motor_temperature = temperature_byte;

  // error code
  error_code_byte = answer_frame.Servo_Upload.error_code;
  answer_values_int.error_code = error_code_byte;

  // cout << "position : " << answer_values_int[i].position << " temp : " << answer_values_int[i].motor_temperature << endl;

  answer_values.position = ((double)(answer_values_int.position))/10;
  answer_values.speed = ((double)(answer_values_int.speed))/10;
  answer_values.torque = ((double)(answer_values_int.current))/100*_MotorDriveInformations.motor_info.reduction_ratio*_MotorDriveInformations.motor_info.torque_constant;
  answer_values.motor_temperature = answer_values_int.motor_temperature;
  answer_values.error_code = answer_values_int.error_code;


} 

int can_base::Recieve_state_MIT(answer_format &answer_values, int _drive_ID, Motor_Drive_Info _MotorDriveInformations)
{
  int limiter = 0;
  int nbytes;

  double peak_torque = _MotorDriveInformations.motor_info.peak_torque;
  double peak_pos = _MotorDriveInformations.motor_info.peak_position;
  double peak_speed = _MotorDriveInformations.motor_info.peak_speed;

  double torque_range = 2*peak_torque;
  double pos_range = 2*peak_pos;
  double speed_range = 2*peak_speed;

  double torque_int_range = (double)(pow(2,12) - 1);
  double pos_int_range = (double)(pow(2,16) - 1);
  double speed_int_range = (double)(pow(16,3) - 1);

  CAN_Frame answer_frame;
  CAN_Frame request_state_frame;

  byte position_byte[2];
  byte speed_high_byte;
  byte speed_low_current_high_byte;
  byte current_low_byte;
  byte motor_temperature_byte;
  byte error_code_byte;
  
  answer_format_int answer_values_int;

  // frame for requesting state to motor drive : {FF,FF,FF,FF,FF,FF,FF,FC} 
  request_state_frame.canID = COB_ID::MIT::COB_ID_MIT + _drive_ID;
  request_state_frame.frame_byte[0] = 255;
  request_state_frame.frame_byte[1] = 255;
  request_state_frame.frame_byte[2] = 255;
  request_state_frame.frame_byte[3] = 255;
  request_state_frame.frame_byte[4] = 255;
  request_state_frame.frame_byte[5] = 255;
  request_state_frame.frame_byte[6] = 255;
  request_state_frame.frame_byte[7] = 252;

  // Send_frame(request_state_frame,8);
  // Send_frame(request_state_frame,8);

  Read_frame(answer_frame,nbytes);
  while(answer_frame.Mit_Upload.drive_id != _drive_ID) 
  {
    Read_frame(answer_frame,nbytes);
    limiter ++;
  }
  if(limiter >= 6)
  {
    cout << "cannot recieve answer within 6 times" << endl;
  }

  position_byte[0] = answer_frame.Mit_Upload.position[0];
  position_byte[1] = answer_frame.Mit_Upload.position[1];
  speed_high_byte = answer_frame.Mit_Upload.speed_high;
  speed_low_current_high_byte = answer_frame.Mit_Upload.speed_low_current_high;
  current_low_byte = answer_frame.Mit_Upload.current_low;
  motor_temperature_byte = answer_frame.Mit_Upload.motor_temperature;
  error_code_byte = answer_frame.Mit_Upload.error_code;

  // cout << "position : " << (int)position_byte[0] << "," << (int)position_byte[1] << endl;
  // cout << (int)(speed_high_byte) << "," << (int)speed_low_current_high_byte << "," << (int)current_low_byte << endl;
  // position
  answer_values_int.position = (int)(position_byte[0] << 8) + (int)(position_byte[1]);
  // speed
  answer_values_int.speed = (int)(speed_high_byte << 4) + (int)((speed_low_current_high_byte - (speed_low_current_high_byte % 16)) >> 4); 
  // torque
  answer_values_int.current = (int)((speed_low_current_high_byte % 16) << 8) + (int)(current_low_byte); 
  // temperature
  answer_values_int.motor_temperature = motor_temperature_byte;
  // error code
  answer_values_int.error_code = error_code_byte;

  // cout << "position : " << answer_values_int[i].position << " temp : " << answer_values_int[i].motor_temperature << endl;

  answer_values.position = (((double)(answer_values_int.position))/pos_int_range*pos_range) - peak_pos;
  answer_values.speed = (((double)(answer_values_int.speed))/speed_int_range*speed_range) - peak_speed;
  answer_values.torque = (((double)(answer_values_int.current))/torque_int_range*torque_range) - peak_torque;
  answer_values.motor_temperature = answer_values_int.motor_temperature;
  answer_values.error_code = answer_values_int.error_code; 
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
      position_frame[i].Position_Loop.position[j] = data[i][3-j];   
    }
  }

  for(int i = 0; i < this->nDrives; i++){
    Send_frame(position_frame[i],4);
  }

}

int can_base::TargetTorque(double *target_torque)
{
  CAN_Frame current_frames[this->nDrives];

  for(int i = 0; i < this->nDrives; i++){
    TargetTorqueFrame(current_frames[i],target_torque[i],this->DriveIDs[i],this->MotorDriveInformations[i]);
  }

  for(int i = 0; i < this->nDrives; i++){
    Send_frame(current_frames[i],4*(this->MotorDriveInformations[i].drive_info.drive_mode + 1));
  }


}

int can_base::TargetTorqueFrame(CAN_Frame &current_frame, double target_torque, int _drive_ID, Motor_Drive_Info _MotorDriveInformations)
{
  double temp_target_torque;

  // Take target torque into the bound of the torque of the motor.
  if(target_torque > _MotorDriveInformations.motor_info.peak_torque)
  {
    temp_target_torque = _MotorDriveInformations.motor_info.peak_torque;
  }
  else if(target_torque < -_MotorDriveInformations.motor_info.peak_torque)
  {
    temp_target_torque = -_MotorDriveInformations.motor_info.peak_torque;
  }
  else
  {
    temp_target_torque = target_torque;
  }

  // SERVO mode
  if(_MotorDriveInformations.drive_info.drive_mode == Drive_Mode::SERVO_MODE)
  {
    byte data[4] = {0};

    // convert torque to current and convert double current to integer current. 2 which is located after temp_target_current is flag_type and means torque flag
    Convert_to_byte_data_SERVO(temp_target_torque, data, 2, _MotorDriveInformations);

    current_frame.Current_Loop.canID = COB_ID::SERVO::CURRENT_LOOP + _drive_ID;

    for(int j = 0; j < 4; j++)
    {
      current_frame.Current_Loop.current[j] = data[3-j];   
    }
  }

  // MIT mode
  else if(_MotorDriveInformations.drive_info.drive_mode == Drive_Mode::MIT_MODE)
  {
    byte data[8] = {0};
    Convert_to_byte_torque_MIT(data, temp_target_torque, _MotorDriveInformations);

    current_frame.Mit_Download.canID = COB_ID::MIT::COB_ID_MIT + _drive_ID;
    current_frame.Mit_Download.position[0] = data[0];
    current_frame.Mit_Download.position[1] = data[1];
    current_frame.Mit_Download.speed_high = data[2];
    current_frame.Mit_Download.speed_low_kp_high = data[3];
    current_frame.Mit_Download.kp_low = data[4];
    current_frame.Mit_Download.kd_high = data[5];
    current_frame.Mit_Download.kd_low_current_high = data[6];
    current_frame.Mit_Download.current_low = data[7];
  }

}


void can_base::Convert_to_byte_data_SERVO(double raw_data, byte *byte_data, int flag_type, Motor_Drive_Info _MotorDriveInformations)
{
  int int_data;
  int nbytes;

  if(flag_type == 1){ /* Position Mode */ /* output-deg */
    int_data = (int)(raw_data * this->PositionResolution);
  }
  else if(flag_type == 2){ /* Torque Mode */ /* Motor Current - A */
    int_data = (int)(raw_data/(_MotorDriveInformations.motor_info.reduction_ratio*_MotorDriveInformations.motor_info.torque_constant)*this->CurrentResolution);
  }
  // cout << " current int : " << int_data;
  Decimal_to_bytes(byte_data, nbytes, int_data);

}

void can_base::Convert_to_byte_torque_MIT(byte *frame, double torque, Motor_Drive_Info _MotorDriveInformations)
{
  double peak_torque = _MotorDriveInformations.motor_info.peak_torque;
  double range = 2*peak_torque;
  double torque_int_range = (double)(pow(2,12) - 1);
  int torque_int_val = (int)((torque + peak_torque)*torque_int_range/range);

  for(int i = 0; i < 6; i++){
    frame[i] = 0;
  }
  frame[6] = (torque_int_val - (torque_int_val % 256)) >> 8;
  frame[7] = (torque_int_val % 256); 

}

void can_base::Convert_to_byte_position_MIT(byte *frame, double kp, double kd, double ref_pos, double ref_speed, Motor_Drive_Info _MotorDriveInformations)
{
  double peak_pos = _MotorDriveInformations.motor_info.peak_position;
  double peak_speed = _MotorDriveInformations.motor_info.peak_speed;

  double pos_range = 2*peak_pos;
  double speed_range = 2*peak_speed;
  double kp_range = 500;
  double kd_range = 5;

  double pos_int_range = (double)(pow(2,16) - 1);
  double speed_int_range = (double)(pow(16,3) - 1);
  double kp_kd_int_range = (double)(pow(16,3) - 1);

  int pos_int_val = (int)((ref_pos + peak_pos)*pos_int_range/pos_range);
  int speed_int_val = (int)((ref_speed - peak_speed)*speed_int_range/speed_range);
  int kp_int_val = (int)(kp*kp_kd_int_range/kp_range);
  int kd_int_val = (int)(kd*kp_kd_int_range/kd_range);


  frame[0] = (pos_int_val - (pos_int_val % 256)) >> 8;
  frame[1] = pos_int_val % 256;
  frame[2] = (speed_int_val - (speed_int_val % 16)) >> 4;
  frame[3] = (speed_int_val % 16) << 4 + (kp_int_val - (kp_int_val % 256)) >> 8;  
  frame[4] = kp_int_val % 256;
  frame[5] = (kd_int_val - (kd_int_val % 16)) >> 4;
  frame[6] = (kd_int_val % 16) << 4;
  frame[7] = 0;
}


void can_base::Double_to_bytes_speed_MIT(byte *frame, double kd, double ref_speed, Motor_Drive_Info _MotorDriveInformations)
{
  double peak_speed = _MotorDriveInformations.motor_info.peak_speed;

  double speed_range = 2*peak_speed;
  double kd_range = 5;

  double speed_int_range = (double)(pow(16,3) - 1);
  double kd_int_range = (double)(pow(16,3) - 1);

  int speed_int_val = (int)((ref_speed - peak_speed)*speed_int_range/speed_range);
  int kd_int_val = (int)(kd*kd_int_range/kd_range);


  frame[0] = 0;
  frame[1] = 0;
  frame[2] = (speed_int_val - (speed_int_val % 16)) >> 4;
  frame[3] = (speed_int_val % 16) << 4;  
  frame[4] = 0;
  frame[5] = (kd_int_val - (kd_int_val % 16)) >> 4;
  frame[6] = (kd_int_val % 16) << 4;
  frame[7] = 0;
}

// void Double_to_bytes_MIT_torque(byte *frame, double torque, Motor_Drive_Info _MotorDriveInformations)
// void Double_to_bytes_MIT_position(byte *frame, double kp, double kd, double ref_pos, double ref_speed, Motor_Drive_Info _MotorDriveInformations)
// void Double_to_bytes_MIT_speed(byte *frame, double kd, double ref_speed, Motor_Drive_Info _MotorDriveInformations)


int can_base::utilities::Hex2number(char* word)
{
    std::string myhex(word);
    int x = strtoul(myhex.substr(0, 4).c_str(), NULL, 16);

    return x;
}