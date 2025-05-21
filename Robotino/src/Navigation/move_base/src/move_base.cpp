#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32MultiArray.h>
#include <cmath>

void move_base(double x, double y, double theta, double time_out, ros::Publisher& pubVel);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_forward");
    ros::NodeHandle nh;

    ros::Rate rate(1000);
    ros::Publisher pubVel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    
    move_base(0.2, 0.0, 0.0, 5.0, pubVel); //1 [m]
    //move_base(0.0, 0.0, -0.1, 15.7075, pubVel); // pi/2 [rad]
    //move_base(0.1, 0.0, 0.0, 8.0, pubVel); //1 [m]
    //move_base(0.0, 0.1, 0.0, 8.0, pubVel); // 0.8 [m]
    //move_base(0.1, 0.0, 0.0, 8.0, pubVel); // 0.8 [m]
    std::cout<<"Listooooo"<<std::endl;
    return 0;
}

void move_base(double x, double y, double theta, double time_out, ros::Publisher& pubVel)
{
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = x;
    vel_msg.linear.y = y;
    vel_msg.angular.z = theta;
    
    ros::Time init = ros::Time::now();
    while(ros::ok() && (ros::Time::now() - init).toSec() < time_out)
    {
        pubVel.publish(vel_msg);
        ros::Duration(0.01).sleep();
    }
}