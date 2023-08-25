#ifndef CAN_OBJ_DICTIONARY_HPP
#define CAN_OBJ_DICTIONARY_HPP

namespace CAN_FAMILY
{
  /* COB-ID for T-motor CAN communication
  /* https://cubemars.com/images/file/20230721/1689907184190344.pdf */
  namespace SERVO
  {
    enum COB_ID
    {
      DUTY_CYCLE = 0x000,
      CURRENT_LOOP = 0x100,
      CURRENT_BRAKE = 0x200,
      VELOCITY = 0x300,
      POSITION = 0x400,
      SET_ORIGIN = 0x500,
      POSITION_VELOCITY_LOOP = 0x600
    }; // OBJECT
  } // SERVO

}

#endif