#pragma once
// ----- ROS Libraties ----- //
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>

// ----- Msgs ----- //
#include "robotino_msgs/DigitalReadings.h"
#include <kinect_move/MoveTilt.h>
#include <kinect_move/InitTilt.h>

// ---- C++ Libraries ----- //
#include <iostream>
#include <cmath>

class FestinoHardware
{
private:
    static bool is_node_set;
    static ros::Publisher pub_digital;
    static ros::Publisher pub_gripper;    
    static ros::Publisher pub_move_manipulator;
    static ros::Publisher pub_move_manipulator_home;
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

    //Method for move the kinect
    static bool move_kinect(float theta, double time_out);
    static bool init_kinect();

    //Methods for control the manipulator
    static void move_manipulator(float x, float y, float z);
    static void move_manipulator_home(bool state);
    static void move_gripper(bool state);
};