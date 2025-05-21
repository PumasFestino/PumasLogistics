#include "refbox_comm/comm_twr.h"

#include <chrono>
#include <iomanip>
#include <sstream>
#include <memory>
#include <thread>

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "refbox_comm_node");
    ros::NodeHandle n;

    std::shared_ptr<google::protobuf::Message> beacon_msg;
    llsf_msgs::AgentTask task;
    ROS_INFO_STREAM("Refbox Comm Node");
    Peer pe("localhost", 4441, 4444);
    
    task.mutable_move()->set_waypoint("M_Z33");
    task.set_team_color(llsf_msgs::Team::CYAN);
    task.set_task_id(1);
    task.set_robot_id(1);

    Robot robot;
    
    ros::Duration(5.0).sleep();
    robot.sendTasktoRobot(task);
    ros::Rate r(10);
    while (ros::ok()) 
    {
        beacon_msg = robot.beacon2main();
        pe.sendBeaconSignal(robot);
        ros::spinOnce();
    }
    return 0;
}