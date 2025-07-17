#pragma once
// ----- C++ Libraries -----//
#include <iostream>
#include <string>
#include <vector>

// ----- ROS Libraries ----- //
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

// ----- Custom msgs ----- //
#include <vision_msgs/FaceRecogSrv.h>
#include <vision_msgs/FaceTrainSrv.h>
#include <img_proc/Tag_with_tf.h>
#include <img_proc/ReadQRCode.h>
#include <vision_logistics/RunTask.h>

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

    //Logistics camera taks
    static ros::ServiceClient cltCameraTask;

    static ros::Subscriber subCentroidPiece;
    static geometry_msgs::Point _centroidPiece;

    static float _centroid_x;
    static float _centroid_y;


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
    static std::string getArucoTF(bool flag);    
    
    //QR detector
    static std::string enableQRDetect(bool enabled);

    //Logistics camera task
    static void callbackCentroid(const geometry_msgs::Point::ConstPtr& msg);
    static std::pair<double, double> findPiece();
    static float findBand();
    static float centerBand();

    static float findEndBand();

private:
    //
    
    
};
