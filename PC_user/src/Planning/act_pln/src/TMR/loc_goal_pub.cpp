#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "loc_goal_pub");
  ros::NodeHandle n;
  ros::Publisher zone_goal_take_pub = n.advertise<std_msgs::String>("zone_goal_take", 1000);
  ros::Publisher zone_goal_del_pub = n.advertise<std_msgs::String>("zone_goal_del", 1000);

  ros::Rate loop_rate(1);

  while (ros::ok())
  {
    std_msgs::String msg_take;
    std_msgs::String msg_del;

    std::cout << "Take Goal:  " << argv[1] << "\tDeliver Goal:  " << argv[2]<< std::endl;
    msg_take.data = argv[1];
    msg_del.data = argv[2];

    zone_goal_take_pub.publish(msg_take);
    zone_goal_del_pub.publish(msg_del);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
