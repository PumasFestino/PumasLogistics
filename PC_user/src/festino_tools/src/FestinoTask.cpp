#include "festino_tools/FestinoTask.h"

ros::NodeHandle* FestinoTask::nh_ = nullptr;

bool FestinoTask::setNodeHandle(ros::NodeHandle* nh) {
    nh_ = nh;
    return nh_ != nullptr;
}

bool FestinoTask::grasp(const std::string& action) {
    actionlib::SimpleActionClient<festino_task::graspingAction> client("grasping", true);
    ROS_INFO("[FestinoTask] Esperando servidor grasping...");
    client.waitForServer();

    festino_task::graspingGoal goal;
    goal.action_manip = action;

    client.sendGoal(goal);
    client.waitForResult(ros::Duration(30.0));

    if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("[FestinoTask] Grasping completado");
        return client.getResult()->success;
    } else {
        ROS_WARN("[FestinoTask] Grasping falló o expiró");
        return false;
    }
}

bool FestinoTask::navigate(const std::string& zone) {
    actionlib::SimpleActionClient<festino_task::navigate_to_zoneAction> client("zone_navigation", true);
    ROS_INFO("[FestinoTask] Esperando servidor de navegación...");
    client.waitForServer();

    festino_task::navigate_to_zoneGoal goal;
    goal.target_zone = zone;

    client.sendGoal(goal);
    client.waitForResult(ros::Duration(60.0));

    if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("[FestinoTask] Navegación completada");
        return client.getResult()->success;
    } else {
        ROS_WARN("[FestinoTask] Navegación falló o expiró");
        return false;
    }
}

bool FestinoTask::poseRobot(bool align, const std::string& mps_type, const std::string& mps_band) {
    actionlib::SimpleActionClient<festino_task::pose_robot_stationAction> client("pose_robot_station", true);
    ROS_INFO("[FestinoTask] Esperando servidor pose_robot_station...");
    client.waitForServer();

    festino_task::pose_robot_stationGoal goal;
    goal.aling = align;
    goal.mps_type = mps_type;
    goal.mps_band = mps_band;

    client.sendGoal(goal);
    client.waitForResult(ros::Duration(30.0));

    if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("[FestinoTask] Pose completado");
        return client.getResult()->success;
    } else {
        ROS_WARN("[FestinoTask] Pose falló o expiró");
        return false;
    }
}
