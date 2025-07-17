#pragma once

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <festino_task/graspingAction.h>
#include <festino_task/navigate_to_zoneAction.h>
#include <festino_task/pose_robot_stationAction.h>

class FestinoTask
{
private:
    static ros::NodeHandle* nh_;

public:
    static bool setNodeHandle(ros::NodeHandle* nh);
    static bool grasp(const std::string& action);
    static bool navigate(const std::string& zones);
    static bool poseRobot(bool align, const std::string& mps_type, const std::string& mps_band);
};
