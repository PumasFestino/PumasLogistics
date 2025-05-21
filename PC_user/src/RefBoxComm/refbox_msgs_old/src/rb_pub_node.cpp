#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "rb_pub_node");

  ros::NodeHandle n;

  ros::Publisher rb_msg_pub = n.advertise<std_msgs::String>("rb_msg", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {

    std_msgs::String msg;

    std::stringstream ss;
    ss << "Refbox Msg " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());


    rb_msg_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}