#include "festino_tools/FestinoHardware.h"

bool FestinoHardware::is_node_set = false;

ros::Publisher FestinoHardware::pub_digital;
ros::Publisher FestinoHardware::pub_head_orientation;
ros::Publisher FestinoHardware::pub_gripper;
ros::ServiceClient FestinoHardware::arm_client;
ros::ServiceClient FestinoHardware::kinect_client;
ros::ServiceClient FestinoHardware::kinect_init;
    

//Aquí se configuran los nodos, el tipo de mensaje, buffer, el topico, etc.
bool FestinoHardware::setNodeHandle(ros::NodeHandle* nh)
{
    if(FestinoHardware::is_node_set)
        return true;
    if(nh == 0)
        return false;
    std::cout << "FestinoHardware.->Setting ros node..." << std::endl;
    
    FestinoHardware::pub_head_orientation   = nh -> advertise       <std_msgs::Float64MultiArray>("/hardware/head/goal_pose", 1000);
    FestinoHardware::pub_gripper            = nh -> advertise       <std_msgs::Float64>("/hardware/left_arm/goal_gripper", 1000);
    FestinoHardware::pub_digital            = nh -> advertise       <robotino_msgs::DigitalReadings>("/set_digital_values", 1000);
    FestinoHardware::arm_client             = nh -> serviceClient   <arms::MoveArm>("move_left_arm");
    FestinoHardware::kinect_client          = nh -> serviceClient   <kinect_move::MoveTilt> ("/kinect/move_tilt");
    FestinoHardware::kinect_init            = nh -> serviceClient   <kinect_move::InitTilt> ("/kinect/init_tilt");
    return true;
}

void FestinoHardware::setHeadOrientation(float yaw, float pitch)
{
    std_msgs::Float64MultiArray msg;
    msg.data.resize(2);
    msg.data[0] = yaw;
    msg.data[1] = pitch;
    FestinoHardware::pub_head_orientation.publish(msg);
}

void FestinoHardware::setGripperPose(float gripper)
{
    std_msgs::Float64 msg;
    msg.data = gripper;
    FestinoHardware::pub_gripper.publish(msg);
}

void FestinoHardware::setColorLed(std::string colorName)
{
    //Robotino Lights
    robotino_msgs::DigitalReadings arr_values;
    arr_values.stamp.sec = 0;
    arr_values.stamp.nsec = 0;
    arr_values.values = {0,0,0,0,0,0};

    if (colorName == "red")
    {
        arr_values.values = {0,0,0,1,0,0};
        pub_digital.publish(arr_values);
        ros::Duration(0.5, 0).sleep();
    }

    if (colorName == "green")
    {
        arr_values.values = {0,0,0,0,1,0};
        pub_digital.publish(arr_values);
        ros::Duration(0.5, 0).sleep();
    }

    if (colorName == "blue")
    {
        arr_values.values = {0,0,0,0,0,1};
        pub_digital.publish(arr_values);
        ros::Duration(0.5, 0).sleep();
    }

    if (colorName == "yellow")
    {
        arr_values.values = {0,0,0,1,1,0};
        pub_digital.publish(arr_values);
        ros::Duration(0.5, 0).sleep();
    }

    if (colorName == "magenta")
    {
        arr_values.values = {0,0,0,1,0,1};
        pub_digital.publish(arr_values);
        ros::Duration(0.5, 0).sleep();
    }

    if (colorName == "turquoise")
    {
        arr_values.values = {0,0,0,0,1,1};
        pub_digital.publish(arr_values);
        ros::Duration(0.5, 0).sleep();
    }

    if (colorName == "white")
    {
        arr_values.values = {0,0,0,1,1,1};
        pub_digital.publish(arr_values);
        ros::Duration(0.5, 0).sleep();
    }

}

void FestinoHardware::setArmPose(float x, float y, float z, float roll, float pitch, float yaw) 
{
    arms::MoveArm srv;
    srv.request.x = x;
    srv.request.y = y;
    srv.request.z = z;
    srv.request.roll = roll;
    srv.request.pitch = pitch;
    srv.request.yaw = yaw;
    srv.request.pose_name = "";  // String vacío para indicar modo cartesiano

    if (arm_client.call(srv)) {
        if (!srv.response.success) {
            ROS_ERROR("Failed to move arm to pose: %s", srv.response.message.c_str());
        }
    } else {
        ROS_ERROR("Failed to call move_left_arm service");
    }
}

void FestinoHardware::setArmPose(std::string pose_name)
{
    arms::MoveArm srv;
    srv.request.x = 0;
    srv.request.y = 0;
    srv.request.z = 0;
    srv.request.roll = 0;
    srv.request.pitch = 0;
    srv.request.yaw = 0;
    srv.request.pose_name = pose_name;

    if (arm_client.call(srv)) {
        if (!srv.response.success) {
            ROS_ERROR("Failed to move arm to '%s' pose: %s", 
                     pose_name.c_str(), srv.response.message.c_str());
        }
    } else {
        ROS_ERROR("Failed to call move_left_arm service");
    }
}

bool FestinoHardware::move_kinect(float theta, double time_out)
{   
    if (!kinect_client.isValid()) {
        std::cerr << "FestinoNavigation.->Service client not initialized!" << std::endl;
        return false;
    }
    
    kinect_move::MoveTilt::Request req;
    kinect_move::MoveTilt::Response res;

    req.angle = theta;
    req.time_out = time_out;

    if (kinect_client.call(req, res))
    {
        if (res.success)
        {
            std::cout << "FestinoHardware.-> move kinect already" << std::endl;
            return true;
        }
        else
        {
            std::cout << "FestinoHardware.-> move kinect failed. " << std::endl;
            return false;
        }
    } 
    else
    {
        std::cout << "FestinoHardware.-> move kinect service call failed" << std::endl;
        return false;
    }
}

bool FestinoHardware::init_kinect()
{   
    if (!kinect_init.isValid()) {
        std::cerr << "FestinoHardware.->Service client not initialized!" << std::endl;
        return false;
    }
    kinect_move::InitTilt::Request req;
    kinect_move::InitTilt::Response res;
    kinect_init.call(req, res);
    if (res.success)
        {
            std::cout << "FestinoHardware.-> init kinect " << res << std::endl;
        }
}