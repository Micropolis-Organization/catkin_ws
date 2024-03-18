#include "ROS_Node.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "CAN_node");
  ROS_Node ros_node;

  while (ros::ok())
  {
    
    ros_node.update();
    ros::spinOnce();
  }
}
