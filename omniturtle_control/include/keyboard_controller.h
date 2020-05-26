#ifndef KEYBOARD_CONTROLLER_H
#define KEYBOARD_CONTROLLER_H

#include"controller.h"

// ROS
#include<ros/ros.h>
#include"std_msgs/Int32.h"
#include"geometry_msgs/Twist.h"

// C/C++
#include<string>
#include<iostream>
#include<ncurses.h>
#include<exception>
#include<stdexcept>

namespace OmniTurtle
{
  class KeyboardController : public InputController
  {
  private:
    ros::NodeHandle* n;
    ros::Publisher keyboard_pub;
    geometry_msgs::Twist ctrl_msg;
    bool is_running;
    double velocity;

  public:
    KeyboardController(ros::NodeHandle& node, const std::string& pub_topic_name);
    void getInput() override;
    void sendMessage() const override;
    bool isRunning() const { return is_running; }
    void quit() const;

    enum KEYMAP
    {
      ONE = 49,
      TWO,
      THREE,
      FOUR,
      FIVE,
      SIX,
      SEVEN,
      EIGHT,
      NINE,
      QL = 113,
      WL = 119,
      EL = 101,
      AL = 97,
      SL = 115,
      DL = 100,
      ZL = 122,
      CL = 99,
      OL = 111,
      PL = 112,
      QU = 81,
      WU = 87,
      EU = 69,
      AU = 65,
      SU = 83,
      DU = 68,
      ZU = 90,
      CU = 67,
      OU = 79,
      PU = 80,
      SPACE = 32,
      ESC   = 27,
      ENTER = 10,
      CTRLC = 3,
      PLUS  = 43,
      MINUS = 45
    };
  };
}




#endif // KEYBOARD_CONTROLLER_H
