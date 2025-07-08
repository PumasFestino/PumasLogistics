// --------------------------------------------------------------------------------------- //
// ----- This action put the robot in position for take or drop piece (pre-grasping) ----- //
// --------------------------------------------------------------------------------------- //

// ----- ROS Libraries ----- //
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>

// ----- Action Library ----- //
#include <festino_task/pose_robot_stationAction.h>

// ----- Festino tools ----- //
#include <festino_tools/FestinoNavigation.h>
#include <festino_tools/FestinoVision.h>

enum SMState
{
    SM_INIT,
    SM_FIND_TAG,
    SM_ALING_WITH_MPS,
    SM_ALING_FOR_PIECE,
    SM_PUT_MPS_ON_MAP,
    SM_FINISH
};

class PoseRobotStationAction
{
    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<festino_task::pose_robot_stationAction> as_;

        std::string action_name_;
        std::string name_mps_;
        std::string mps_type_;
        std::string mps_band_;
        
        festino_task::pose_robot_stationFeedback feedback_;
        festino_task::pose_robot_stationResult result_;
        
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        SMState state_;

        bool aling_;
        bool success_;

        //Variables for aling whit mps
        float forward_distance_     = 0.75;
        float platform_distance_    = 0.35;
        float move_left_distance_   = 0.20;
        float move_right_distance_  = 0.20;

        float move_base_vel_        = 0.15;

    public:
        PoseRobotStationAction(std::string name):
            as_(nh_, name, boost::bind(&PoseRobotStationAction::executeCB, this, _1), false),
            action_name_(name),
            
            tf_listener_(tf_buffer_)
            {
                state_ = SM_INIT;
                success_ = false;

                FestinoNavigation::setNodeHandle(&nh_);
                FestinoVision::setNodeHandle(&nh_);

                as_.start();
                ROS_INFO("ACTION SERVER [%s] started.", action_name_.c_str());
            }

            void executeCB(const festino_task::pose_robot_stationGoalConstPtr &goal)
            {
                aling_      = goal -> aling;
                mps_type_   = goal -> mps_type;
                mps_band_   = goal -> mps_band;

                ros::Rate rate(10);
                std::string current_state = "";

                while (ros::ok() && !success_ && !as_.isPreemptRequested())
                {
                    switch (state_)
                    {
                        case SM_INIT:
                            current_state = "SM_INIT";
                            state_ = SM_ALING_WITH_MPS;
                            break;

                        case SM_FIND_TAG:
                            current_state = "SM_FIND_TAG";
                            name_mps_ = FestinoVision::getArucoTF(true);
                            std::cout << "Aruco Marker: " << name_mps_ << std::endl;
                            state_ = SM_FINISH;
                            break;

                        case SM_ALING_WITH_MPS:
                            current_state = "SM_ALING_WITH_MPS";
                            
                            //Aling with MPS before
                            FestinoNavigation::alingWithLine(true);

                            //Move base forward 0.75m
                            FestinoNavigation::move_base(1, 0, move_base_vel_, forward_distance_);

                            //Aling again
                            FestinoNavigation::alingWithLine(true);
                            
                            state_ = SM_ALING_FOR_PIECE;
                            break;

                        case SM_ALING_FOR_PIECE:
                            if(mps_type_ == "platform")
                            {
                                current_state = "SM_ALING_FOR_PIECE --- Platform";
                                FestinoNavigation::move_base(0, -1, move_base_vel_, platform_distance_);
                            }
                         
                            //Si estamos en la CS, ya sea entrada o salida que se mueva uno a la izquierda
                            else if((mps_type_ == "CS") || (mps_type_ == "BS" && mps_band_ == "output"))
                            {
                                current_state = "SM_ALING_FOR_PIECE --- " + mps_type_ + " -> " + mps_band_;
                                FestinoNavigation::move_base(0, 1, move_base_vel_, move_left_distance_);
                            }
                            
                            //si estamos en la BS y vamos a la entrance entonces que se mueva uno a la derecha
                            else if(mps_type_ == "BS" && mps_band_ == "entrance")
                            {
                                current_state = "SM_ALING_FOR_PIECE --- " + mps_type_ + "-> " + mps_band_;
                                FestinoNavigation::move_base(0, -1, move_base_vel_, move_right_distance_);
                                
                            }
                            /*else if(tokens[1] == "RS" && tokens[4] == "output"){
                               //Mueve uno a la derecha de la banda
                               //Negativo a la derecha
                               vel.linear.y = -2;
                               std::cout << "Publico en vel para quedar a la derecha de la banda" << std::endl;
                               pubVel.publish(vel);
			                   ros::Duration(1, 0).sleep();
                            }*/
                                                     				
			                state_ = SM_FINISH;	
			        
                            break;
                        case SM_PUT_MPS_ON_MAP:
                            current_state = "SM_PUT_MPS_ON_MAP";
                            state_ = SM_FINISH;
                            break;

                        case SM_FINISH:
                            current_state = "SM_FINISH";
                            success_ = true;
                            break;
                    }

                    feedback_.state = current_state;
                    as_.publishFeedback(feedback_);
                    rate.sleep();
                }

                result_.success = success_;
                as_.setSucceeded(result_);
                success_ = false;
                state_ = SM_INIT;
            }
    };  

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_robot_station_server");
    PoseRobotStationAction action_server("pose_robot_station");
    ros::spin();
    return 0;
}