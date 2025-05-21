#pragma once
#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "robotino_msgs/DigitalReadings.h"
#include <arms/MoveArm.h>
#include <kinect_move/MoveTilt.h>
#include <kinect_move/InitTilt.h>


class FestinoHardware
{
private:
    static bool is_node_set;
    static ros::Publisher pub_digital;
    static ros::Publisher pub_head_orientation;    
    static ros::Publisher pub_gripper;    
    static ros::ServiceClient arm_client;
    static ros::ServiceClient kinect_client;
    static ros::ServiceClient kinect_init;

public:
    //
    //The startSomething functions, only publish the goal pose or path and return inmediately after starting movement
    //The others, block until a goal-reached signal is received
    //
    
    static bool setNodeHandle(ros::NodeHandle* nh);
    //Methods for changes led color
    static void setColorLed(std::string colorName);
    static void setHeadOrientation(float yaw, float pitch);
    static void setArmPose(float x, float y, float z, float yaw, float pitch, float roll);
    static void setArmPose(std::string pose);
    static void setGripperPose(float gipper);
    static bool move_kinect(float theta, double time_out);
    static bool init_kinect();
};