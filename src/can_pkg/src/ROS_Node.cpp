#include "ROS_Node.hpp"

ROS_Node::ROS_Node(/* args */)
{
   this->_nh = new ros::NodeHandle(); // Create a Node Handle for the ROS

   this->velocity_sub = this->_nh->subscribe<std_msgs::Float32>("velocity", 1, [&](const std_msgs::Float32::ConstPtr &velocityMsg)
                                                                { this->_velocity = velocityMsg->data; 
                                                                  std::cout<<"velocity: " <<this->_velocity<<std::endl; });

   this->steering_sub = this->_nh->subscribe<std_msgs::Float32>("steering_rad", 1, [&](const std_msgs::Float32::ConstPtr &steeringMsg)
                                                                { this->_steering = steeringMsg->data;
                                                                std::cout<<"steering: " <<this->_steering<<std::endl; });
   
   this->getRosParam("/can_node/port", this->_USB_PORT);

   can_interface = new CAN_Interface((char *)this->_USB_PORT.c_str());
}

void ROS_Node::update()
{
   // can_interface->update_loop(this->_velocity, this->_steering);
}

void ROS_Node::getRosParam(std::string paramName, auto &paramValue)
{
   if (this->_nh->getParam(paramName, paramValue))
   {
      std::stringstream strg;
      strg << paramValue;
      std::string s = strg.str();
      ROS_INFO("[IWebotsRosNode] [PARAM] %s = %s", paramName.c_str(), s.c_str());
   }
   else
   {
      ROS_WARN("[IWebotsRosNode] [PARAM] %s is not set", paramName.c_str());
   }
}

ROS_Node::~ROS_Node()
{
   this->_nh->shutdown();
}
