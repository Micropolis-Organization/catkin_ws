#ifndef __ROS_NODE_HPP__
#define __ROS_NODE_HPP__

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "CAN_Interface.hpp"

class ROS_Node
{
private:
      ros::NodeHandle *_nh;

      ros::Subscriber velocity_sub,
          steering_sub;

      CAN_Interface *can_interface;

      std::string _USB_PORT;

      float _velocity = 50.0,
            _steering = 50.0;
      float _prev_velocity = 50,
            _prev_steering = 50;
      void getRosParam(std::string paramName, auto &paramValue);

public:
      ROS_Node(/* args */);
      ~ROS_Node();
      void update();
};

#endif //__ROS_NODE_HPP__