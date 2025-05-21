#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <simple_move/MoveBase.h>

class MoveBaseService
{
    private:
        ros::NodeHandle nh_;
        ros::ServiceServer service_;
        ros::Publisher pubVel;

    public:
        MoveBaseService()
        {
            service_ = nh_.advertiseService("navigation/move_base", &MoveBaseService::executeCB, this);
            pubVel = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

            std::cout << "Move_base --- Soft by Joshua M" << std::endl;
        }

        bool executeCB(simple_move::MoveBase::Request &req, simple_move::MoveBase::Response &res)
        {
            ros::Rate rate(10000);
            geometry_msgs::Twist vel_msg;
            vel_msg.linear.x = req.x;
            vel_msg.linear.y = req.y;
            vel_msg.angular.z = req.theta;

            ros::Time init = ros::Time::now();
            while (ros::ok() && (ros::Time::now() - init).toSec() < req.time_out)
            {
                pubVel.publish(vel_msg);
                rate.sleep();
            }

            res.success = true;
            return true;
        }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_base_service");
    MoveBaseService move_base;
    ros::spin();
    return 0;
}
