#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void rb_msgCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("RefereeBox Says: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "rb_sub_node");


  ros::NodeHandle n;


  ros::Subscriber sub = n.subscribe("rb_msg", 1000, rb_msgCallback);


  ros::spin();

  return 0;
}