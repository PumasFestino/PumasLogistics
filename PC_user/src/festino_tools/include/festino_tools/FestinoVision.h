#pragma once
#include <iostream>
#include <string>
#include <vector>
#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "vision_msgs/FaceRecogSrv.h"
#include "vision_msgs/FaceTrainSrv.h"
#include "img_proc/Tag_with_tf.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include <img_proc/ReadQRCode.h>

class FestinoVision
{
private:
    
    static ros::NodeHandle* nh;
    static bool is_node_set;
    
    //Pose Estimation
    static ros::Subscriber subPointingHand;
    static std::string _pointing_hand;

    //Face Recognition
    static std::vector<std::string> _nameRecog;
    static ros::ServiceClient cltFindPersons;
    static ros::ServiceClient cltTrainPersons;

    //Aruco detector
    static ros::ServiceClient cltArucoTf;
    static std::vector<std::string> _nameArUcoDet;
    static std::vector<geometry_msgs::PoseStamped> _posArUcoDet;

    //QR detector
    static ros::ServiceClient cltQRSrv;

public:
    
    static bool setNodeHandle(ros::NodeHandle* _nh);

    //Pose Estimation
    static void callbackPointingHand(const std_msgs::String::ConstPtr& msg);
    static std::string PointingHand();
    static void enablePoseEstimation(bool flag);

    //Face Recognition
    static std::vector<std::string> enableRecogFacesName(bool flag);
    static bool TrainingPerson(std::string person);

    //Aruco detector
    static void enableArucoDet(bool flag);
    
    //QR detector
    static std::string enableQRDetect(bool enabled);



private:
    //
    
    
};
