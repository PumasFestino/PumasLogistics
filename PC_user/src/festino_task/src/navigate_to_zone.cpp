/*  
    ----------------------------------------------------------------------
    ----- This action node navigates to a specified zone on the map. -----
    ----------------------------------------------------------------------
*/

/*------------------------C++ Libraries-----------------*/
#include <iostream>
#include <cmath>
#include <vector> 
#include <string>
#include <sstream>
#include <algorithm>
#include <boost/bind.hpp>

/*------------------------ROS Libraries-----------------*/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_listener.h>
#include "robotino_msgs/DigitalReadings.h"
#include "ros/time.h"
#include <actionlib/server/simple_action_server.h>
#include "actionlib_msgs/GoalStatus.h"

/*------------------------Action Library---------------*/
#include <festino_task/navigate_to_zoneAction.h>

/*------------------------Festino Tools-----------------*/
#include "festino_tools/FestinoCommunication.h"

//Se puede cambiar, agregar o eliminar los estados
enum SMState {
    SM_INIT,
    SM_TRANSFORM_ZONES, // Positions are sent in order based on plan
    SM_NAV_TO_ZONE,
    SM_WAIT_AT_ZONE,
    SM_REPORT_POSE,
    SM_FINAL_STATE
};

class NavigateToZoneActionServer
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<festino_task::navigate_to_zoneAction> as_;
    std::string action_name_;
    
    // Action messages
    festino_task::navigate_to_zoneFeedback feedback_;
    festino_task::navigate_to_zoneResult result_;
    
    // Subscribers and Publishers
    ros::Subscriber sub_move_goal_status_;
    ros::Publisher pub_goal_;
    
    // State machine variables
    SMState state_;
    std::string target_zone_;
    geometry_msgs::PoseStamped tf_target_zone_;
    actionlib_msgs::GoalStatus simple_move_goal_status_;
    int simple_move_status_id_;
    
    // Transform listener
    tf::TransformListener tf_listener_;

public:
    NavigateToZoneActionServer(std::string name) :
        as_(nh_, name, boost::bind(&NavigateToZoneActionServer::executeCB, this, _1), false),
        action_name_(name),
        state_(SM_INIT),
        simple_move_status_id_(0)
    {
        // Initialize subscribers and publishers
        sub_move_goal_status_ = nh_.subscribe("/simple_move/goal_reached", 10, 
            &NavigateToZoneActionServer::callbackSimpleMoveGoalStatus, this);
        pub_goal_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000);
        
        // Initialize FestinoCommunication
        if(FestinoCommunication::setNodeHandle(&nh_) == false){
            ROS_ERROR("FestinoCommunication node was not set");
        }
        
        as_.start();
        ROS_INFO("Zone Navigation Action Server Started");
    }

    ~NavigateToZoneActionServer(void) {}

    void callbackSimpleMoveGoalStatus(const actionlib_msgs::GoalStatus::ConstPtr& msg)
    {
        simple_move_goal_status_ = *msg;
        std::stringstream ss;
        ss << msg->goal_id.id;
        ss >> simple_move_status_id_;
    }

    bool transformZones()
    {
        tf::StampedTransform transform;

        tf_target_zone_.pose.position.x = 0.0;
        tf_target_zone_.pose.position.y = 0.0;
        tf_target_zone_.pose.position.z = 0.0;
        tf_target_zone_.pose.orientation.x = 0.0;
        tf_target_zone_.pose.orientation.y = 0.0;
        tf_target_zone_.pose.orientation.z = 0.0;
        tf_target_zone_.pose.orientation.w = 0.0;

        try{
            tf_listener_.lookupTransform(target_zone_, "/map", ros::Time(0), transform);
            
            tf_target_zone_.header.frame_id = "/map";
            tf_target_zone_.pose.position.x = -transform.getOrigin().x();
            tf_target_zone_.pose.position.y = -transform.getOrigin().y();
            tf_target_zone_.pose.position.z = 0.0;
            tf_target_zone_.pose.orientation.x = 0.0;
            tf_target_zone_.pose.orientation.y = 0.0;
            tf_target_zone_.pose.orientation.z = 0.0;
            tf_target_zone_.pose.orientation.w = 1.0;
            
            ROS_INFO("Transformed zone %s to position (%.2f, %.2f)", 
                target_zone_.c_str(), 
                tf_target_zone_.pose.position.x, 
                tf_target_zone_.pose.position.y);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("Transform failed for zone %s: %s", target_zone_.c_str(), ex.what());
            return false;
        }
        return true;
    }

    geometry_msgs::Point getCurrentRobotPosition()
    {
        geometry_msgs::Point robot_pos;
        tf::StampedTransform transform_rob;
        
        try{
            tf_listener_.lookupTransform("/map", "/base_link", ros::Time(0), transform_rob);
            robot_pos.x = transform_rob.getOrigin().x();
            robot_pos.y = transform_rob.getOrigin().y();
            robot_pos.z = 0.0;
        }
        catch (tf::TransformException ex){
            ROS_ERROR("Failed to get robot position: %s", ex.what());
        }
        
        return robot_pos;
    }

    void updateFeedback(const std::string& current_state)
    {
        feedback_.current_state = current_state;
        
        as_.publishFeedback(feedback_);
    }

    void executeCB(const festino_task::navigate_to_zoneGoalConstPtr &goal)
    {
        ros::Rate r(10); // 10 Hz
        bool success = true;
        
        // Initialize from goal
        target_zone_ = goal->target_zone;
        state_ = SM_INIT;
        
        ROS_INFO("Starting zone navigation to %s", target_zone_.c_str());
        
        // Main state machine loop
        while(ros::ok() && success)
        {
            // Check for preemption
            if(as_.isPreemptRequested()){
                ROS_INFO("%s: Preempted", action_name_.c_str());
                as_.setPreempted();
                return;
            }
            
            switch(state_)
            {
                case SM_INIT:
                    ROS_INFO("State: SM_INIT");
                    updateFeedback("Initializing navigation");
                    
                    if(target_zone_ == ""){
                        ROS_ERROR("No target zones provided");
                        success = false;
                        break;
                    }
                    
                    state_ = SM_TRANSFORM_ZONES;
                    ros::Duration(1.0).sleep();
                    break;
                
                case SM_TRANSFORM_ZONES:
                    ROS_INFO("State: SM_TRANSFORM_ZONES");
                    updateFeedback("Transforming zone coordinates");
                    
                    if(!transformZones()){
                        ROS_ERROR("Failed to transform zones");
                        success = false;
                        break;
                    }
                    
                    state_ = SM_NAV_TO_ZONE;
                    break;
                
                case SM_NAV_TO_ZONE:
                    ROS_INFO("State: SM_NAV_TO_ZONE - Navigating to zone %s", 
                        target_zone_.c_str());
                    
                    updateFeedback("Navigating to zone: " + target_zone_);
                    
                    // Publish navigation goal
                    pub_goal_.publish(tf_target_zone_);
                    
                    // Wait for navigation to complete
                    if(simple_move_goal_status_.status == actionlib_msgs::GoalStatus::SUCCEEDED && 
                       simple_move_status_id_ == -1){
                        ROS_INFO("Reached zone %s", target_zone_.c_str());
                        state_ = SM_WAIT_AT_ZONE;
                    }
                    else if(simple_move_goal_status_.status == actionlib_msgs::GoalStatus::ABORTED){
                        ROS_ERROR("Navigation to zone %s failed", target_zone_.c_str());
                        success = false;
                    }
                    break;
                
                case SM_WAIT_AT_ZONE:
                    ROS_INFO("State: SM_WAIT_AT_ZONE");
                    updateFeedback("Waiting at zone: " + target_zone_);
                    
                    // Stay at zone for 5 seconds
                    ros::Duration(5.0).sleep();
                    state_ = SM_REPORT_POSE;
                    break;
                
                case SM_REPORT_POSE:{
                    ROS_INFO("State: SM_REPORT_POSE");
                    updateFeedback("Reporting pose for zone: " + target_zone_);
                    
                    // Report current position
                    geometry_msgs::Point current_pos = getCurrentRobotPosition();
                    if(FestinoCommunication::reportPose(current_pos.x, current_pos.y) == false){
                        ROS_WARN("Pose could not be reported for zone %s", target_zone_.c_str());
                    }
                    
                    state_ = SM_FINAL_STATE;
                    break;
                }
                case SM_FINAL_STATE:
                    ROS_INFO("State: SM_FINAL_STATE");
                    updateFeedback("Navigation sequence completed");
                    
                    // Set final result
                    result_.success = true;
                    
                    ROS_INFO("Zone navigation completed successfully");
                    as_.setSucceeded(result_);
                    return;
            }
            
            ros::spinOnce();
            r.sleep();
        }
        
        // If we exit the loop due to failure
        if(!success){
            result_.success = false;
            as_.setAborted(result_);
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "zone_navigation_action_server");
    
    NavigateToZoneActionServer server("zone_navigation");
    ros::spin();
    
    return 0;
}