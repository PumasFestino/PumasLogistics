#include "festino_tools/FestinoHardware.h"

bool FestinoHardware::is_node_set = false;

ros::Publisher FestinoHardware::pub_digital;

ros::Publisher FestinoHardware::pub_move_manipulator;
ros::Publisher FestinoHardware::pub_move_manipulator_home;
ros::Publisher FestinoHardware::pub_gripper;

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

    //Publisher for color led
    FestinoHardware::pub_digital            = nh -> advertise       <robotino_msgs::DigitalReadings>("/set_digital_values", 1000);

    //Publishers for manipulator
    FestinoHardware::pub_move_manipulator       = nh -> advertise   <geometry_msgs::Point>("/manipulator_move", 1000);
    FestinoHardware::pub_move_manipulator_home  = nh -> advertise   <std_msgs::Bool>("/manipulator_home", 1000);
    FestinoHardware::pub_gripper                = nh -> advertise   <std_msgs::Bool>("/manipulator_gripper", 1000);
    
    //Service Client for move the yaw angle
    FestinoHardware::kinect_client          = nh -> serviceClient   <kinect_move::MoveTilt> ("/kinect/move_tilt");
    FestinoHardware::kinect_init            = nh -> serviceClient   <kinect_move::InitTilt> ("/kinect/init_tilt");
    return true;
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

    std::cout << "FestinoHardware -> set led color: " << colorName << "." << std::endl;

}

bool FestinoHardware::move_kinect(float theta, double time_out)
{   
    if (!kinect_client.isValid()) {
        std::cerr << "FestinoHardware -> Service client not initialized!" << std::endl;
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
            std::cout << "FestinoHardware -> move kinect already" << std::endl;
            return true;
        }
        else
        {
            std::cout << "FestinoHardware -> move kinect failed. " << std::endl;
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
        std::cerr << "FestinoHardware -> Service client not initialized!" << std::endl;
        return false;
    }
    kinect_move::InitTilt::Request req;
    kinect_move::InitTilt::Response res;
    kinect_init.call(req, res);
    if (res.success)
        {
            std::cout << "FestinoHardware -> init kinect " << res << std::endl;
        }
}

void FestinoHardware::move_manipulator(float x, float y, float z) 
{
    geometry_msgs::Point point;
    point.x = x;
    point.y = y;
    point.z = z;

    pub_move_manipulator.publish(point);
    ros::Duration(0.5, 0).sleep();
   
    std::cout << "FestinoHardware -> Move manipulator: x: " << point.x << " mm; y: " << point.y << " mm; z: " << point.z << " mm." << std::endl;
}

void FestinoHardware::move_manipulator_home(bool state) 
{
    std_msgs::Bool msgs;
    msgs.data = state;
    
    pub_move_manipulator_home.publish(msgs);
    ros::Duration(0.5, 0).sleep();
   
    std::cout << "FestinoHardware -> Move manipulator: Go Home." << std::endl;
}


void FestinoHardware::move_gripper(bool state)
{
    std_msgs::Bool msg;
    msg.data = state;
    
    pub_gripper.publish(msg);
    ros::Duration(0.5, 0).sleep();
    
    std::cout << "FestinoHardware -> move gripper: " << (state ? "open." : "close.") << std::endl;
}