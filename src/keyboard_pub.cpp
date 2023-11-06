#include <sstream>
#include "ros/ros.h"
#include "cmath"
#include <iostream>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include <std_msgs/String.h>
#include <can_driver_t/control_param.h>

using namespace std;
 
 

int getch()
{
    int ch;
    struct termios oldt;
    struct termios newt;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_iflag |= IGNBRK;
    newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
    newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
    newt.c_cc[VMIN] = 1;
    newt.c_cc[VTIME] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &newt);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return ch;

}


int main(int argc, char **argv)
{
  ros::init(argc,argv,"keyboard_pub");
  ros::NodeHandle n;
  ros::Publisher msg_pub = n.advertise<can_driver_t::control_param>("/control_param_msg", 1000);

  double rate = 1;
  // ros::param::get("/ros_param/rate",rate);

  ros::Rate loop_rate(rate);


  // int number_drivers;


  // ros::param::get("/t_motor/n_drives",number_drivers);

  sleep(4);


  ROS_INFO("START");

  string str;
  char ch;
  char mode;
  int stop_flag = 0;
  int init_flag = 0;
  int control_mode = 0;
  // control mode
  // 1. ref pos
  // 2. ref vel
  // 3. ref pos and ref vel
  // 4. kp
  // 5. kd
  // 6. kp and kd
  float ref_pos = 0;
  float ref_vel = 0;
  float kp = 0;
  float kd = 0;

  int stof_error;

  can_driver_t::control_param msg;


  while(ros::ok()){
    ROS_INFO("\n\n\n");
    ROS_INFO("Enter mode you want enter");
    ROS_INFO("0. EXIT PROGRAM");
    ROS_INFO("1. pos_1+-    : +- reference posiiton by 1 degree");
    ROS_INFO("2. pos_10+-   : +- reference posiiton by 10 degree");
    ROS_INFO("3. pos_free   : directly enter reference posiiton by degree");
    ROS_INFO("4. vel_free   : directly enter reference velocity by degree");
    ROS_INFO("5. pos_vel    : directly enter reference posiiton and velocity by degree");
    ROS_INFO("6. kp_free    : directly enter kp");
    ROS_INFO("7. kd_free    : directly enter kd");
    ROS_INFO("8. kp_kd_free : directly enter kp and kd");
    ROS_INFO("Please enter mode number");

    mode = getch();

    switch(mode)
    {
      case '0':
        ROS_INFO("PROGRAM TERMINATED!");
        return 0;
      case '1':
        control_mode = 1;
        msg.control_mode = control_mode;

        while(1)
        {
          ROS_INFO("Entered pos_1+- mode");          
          ROS_INFO("1. pos_1+- : +- reference posiiton by 1 degree");
          ROS_INFO("If you want to stop, press 'Z'(capital z) key");
          ROS_INFO("Please enter '+' or '-'");

          ch = getch();
          if (ch == 'Z')
          {
            ROS_INFO("mode 1 terminated");
            break;
          }
          else if (ch == '+')
          {
            ref_pos += 1;
            msg.ref_pos = ref_pos; 
            ROS_INFO("reference position : %f",ref_pos);
          }
          else if (ch == '-')
          {
            ref_pos -= 1;
            msg.ref_pos = ref_pos;
            ROS_INFO("reference position : %f",ref_pos);
          }
          msg_pub.publish(msg);

        }
        break;

      case '2':
        control_mode = 1;
        msg.control_mode = control_mode;

        while(1)
        {
          ROS_INFO("Entered pos_10+- mode");
          ROS_INFO("2. pos_10+- : +- reference posiiton by 10 degree");
          ROS_INFO("If you want to stop, press 'Z'(capital z) key");
          ROS_INFO("Please enter '+' or '-'");

          ch = getch();
          if (ch == 'Z')
          {
            ROS_INFO("mode 2 terminated");
            break;
          }
          else if (ch == '+')
          {
            ref_pos += 10;
            msg.ref_pos = ref_pos; 
            ROS_INFO("reference position : %f",ref_pos);
          }
          else if (ch == '-')
          {
            ref_pos -= 10;
            msg.ref_pos = ref_pos;
            ROS_INFO("reference position : %f",ref_pos);
          }
          msg_pub.publish(msg);

        }        
        break;

      case '3':
        control_mode = 1;
        msg.control_mode = control_mode;

        while(1)
        {
          ROS_INFO("Entered pos_free mode");
          ROS_INFO("3. pos_free : directly enter reference posiiton by degree");
          ROS_INFO("If you want to stop, press 'Z'(capital z) key");
          ROS_INFO("Please enter reference position(degree)");

          getline(cin, str);
          if (str == "Z")
          {
            ROS_INFO("mode 3 terminated");
            break;
          }
          else
          {
            ref_pos = stof(str);
            msg.ref_pos = ref_pos; 
            ROS_INFO("reference position : %f",ref_pos);
          }
          msg_pub.publish(msg);

        }
        break;

      case '4':
        control_mode = 2;
        msg.control_mode = control_mode;

        while(1)
        {
          ROS_INFO("Entered vel_free mode");
          ROS_INFO("4. vel_free : directly enter reference velocity by degree");
          ROS_INFO("If you want to stop, press 'Z'(capital z) key");
          ROS_INFO("Please enter reference velocity(degree/s)");

          getline(cin, str);
          if (str == "Z")
          {
            ROS_INFO("mode 4 terminated");
            break;
          }
          else
          {
            ref_vel = stof(str);
            msg.ref_vel = ref_vel; 
            ROS_INFO("reference velocity : %f",ref_vel);
          }
          msg_pub.publish(msg);

        }
        break;

      case '5':
        control_mode = 3;
        msg.control_mode = control_mode;

        while(1)
        {
          ROS_INFO("Entered pos_vel mode");
          ROS_INFO("5. pos_vel : directly enter reference posiiton and velocity by degree");
          ROS_INFO("If you want to stop, press 'Z'(capital z) key");
          ROS_INFO("Please enter reference position(degree) and velocity(degree/s)");

          getline(cin, str);
          if (str == "Z")
          {
            ROS_INFO("mode 5 terminated");
            break;
          }
          else
          {
            ref_pos = stof(str);
            msg.ref_pos = ref_pos; 
            ROS_INFO("reference position : %f",ref_pos);
          }

          getline(cin, str);
          if (str == "Z")
          {
            ROS_INFO("mode 5 terminated");
            break;
          }
          else
          {
            ref_vel = stof(str);
            msg.ref_vel = ref_vel; 
            ROS_INFO("reference velocity : %f",ref_vel);
          }

          msg_pub.publish(msg);

        }
        break;

      case '6':    
        control_mode = 4;
        msg.control_mode = control_mode;

        while(1)
        {
          ROS_INFO("Entered kp_free mode");
          ROS_INFO("6. kp_free : directly enter kp");
          ROS_INFO("If you want to stop, press 'Z'(capital z) key");
          ROS_INFO("Please enter kp value");

          getline(cin, str);
          if (str == "Z")
          {
            ROS_INFO("mode 6 terminated");
            break;
          }
          else
          {
            kp = stof(str);
            msg.kp = kp; 
            ROS_INFO("kp value : %f",kp);
          }
          msg_pub.publish(msg);

        }
        break;

      case '7':
        control_mode = 5;
        msg.control_mode = control_mode;

        while(1)
        {
          ROS_INFO("Entered kd_free mode");
          ROS_INFO("7. kd_free : directly enter kd");
          ROS_INFO("If you want to stop, press 'Z'(capital z) key");
          ROS_INFO("Please enter kd value");

          getline(cin, str);
          if (str == "Z")
          {
            ROS_INFO("mode 7 terminated");
            break;
          }
          else
          {
            kd = stof(str);
            msg.kd = kd; 
            ROS_INFO("kd value : %f",kd);
          }
          msg_pub.publish(msg);

        }
        break;

      case '8':
        control_mode = 6;
        msg.control_mode = control_mode;

        while(1)
        {
          ROS_INFO("Entered kp_kd_free mode");
          ROS_INFO("8. kp_kd_free : directly enter kp and kd");
          ROS_INFO("If you want to stop, press 'Z'(capital z) key");
          ROS_INFO("Please enter kp and kd value");

          getline(cin, str);
          if (str == "Z")
          {
            ROS_INFO("mode 8 terminated");
            break;
          }
          else
          {
            kp = stof(str);
            msg.kp = kp; 
            ROS_INFO("kp value : %f",kp);
          }

          getline(cin, str);
          if (str == "Z")
          {
            ROS_INFO("mode 8 terminated");
            break;
          }
          else
          {
            kd = stof(str);
            msg.kd = kd; 
            ROS_INFO("kp value : %f",kd);
          }

          msg_pub.publish(msg);

        }
        break;
    }


    ros::spinOnce();
    loop_rate.sleep();

  }


  // while(1)
  // {

  //   ch = getch();

  //   ROS_INFO("Character %c",ch);

  
  // }

  return 0;
}
