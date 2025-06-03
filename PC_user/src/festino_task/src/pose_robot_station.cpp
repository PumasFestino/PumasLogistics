// ----- ROS Libraries ----- //
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>

// ----- Festino tools ----- //
#include <festino_tools/FestinoNavigation.h>
#include <festino_tools/FestinoVision.h>

// ----- Action Library ----- //
#include <festino_task/pose_robot_stationAction.h>


enum SMState
{
    SM_INIT,
    SM_FIND_TAG,
    SM_ALING_WITH_MPS,
    SM_PUT_MPS_ON_MAP,
    SM_FINISH
};

class PoseRobotStationAction
{
    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<festino_task::pose_robot_stationAction> as_;
        std::string action_name_;
        festino_task::pose_robot_stationFeedback feedback_;
        festino_task::pose_robot_stationResult result_;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        SMState state_;
        bool success_;
        std::string name_mps_;
        float error_x_;
        float error_y_;
        bool aling_;
        int aux_;

    public:
        PoseRobotStationAction(std::string name):
            as_(nh_, name, boost::bind(&PoseRobotStationAction::executeCB, this, _1), false),
            action_name_(name),
            
            tf_listener_(tf_buffer_)
            {
                state_ = SM_INIT;
                success_ = false;
                error_x_ = error_y_ = 0.0;
                aux_ = 1;

                FestinoNavigation::setNodeHandle(&nh_);
                FestinoVision::setNodeHandle(&nh_);

                as_.start();
                ROS_INFO("ACTION SERVER [%s] started.", action_name_.c_str());
            }

            void executeCB(const festino_task::pose_robot_stationGoalConstPtr &goal)
            {
                aling_ = goal->aling;
                ros::Rate rate(10);
                std::string current_state = "";

                while (ros::ok() && !success_ && !as_.isPreemptRequested())
                {
                    switch (state_)
                    {
                        case SM_INIT:
                            current_state = "SM_INIT";
                            state_ = SM_FIND_TAG;
                            break;

                        case SM_FIND_TAG:
                            current_state = "SM_FIND_TAG";
                            name_mps_ = FestinoVision::getArucoTF(true);
                            state_ = (name_mps_ != "") ? (aling_ ? SM_ALING_WITH_MPS : SM_PUT_MPS_ON_MAP) : SM_FIND_TAG;
                            break;

                        case SM_ALING_WITH_MPS:
                            current_state = "SM_ALING_WITH_MPS";
                            while (aux_)
                            {
                                //USAR LIDAAAAR    
                                try
                                {
                                    if (!tf_buffer_.canTransform("map", name_mps_, ros::Time(0), ros::Duration(0.2)) ||
                                        !tf_buffer_.canTransform("map", "camera_link", ros::Time(0), ros::Duration(0.2)))
                                    {
                                        ROS_WARN("TF not available.");
                                        continue;
                                    }

                                    geometry_msgs::TransformStamped coord_Aruco = tf_buffer_.lookupTransform("map", name_mps_, ros::Time(0));
                                    geometry_msgs::TransformStamped coord_cam_robot = tf_buffer_.lookupTransform("map", "camera_link", ros::Time(0));

                                    error_x_ = coord_Aruco.transform.translation.x - coord_cam_robot.transform.translation.x;
                                    error_y_ = coord_Aruco.transform.translation.y - coord_cam_robot.transform.translation.y;

                                    FestinoNavigation::move_base(error_x_, error_y_, 0.0, 0.001);

                                    if (fabs(error_x_) < 0.18 && fabs(error_y_) < 0.1)
                                    {
                                        FestinoNavigation::move_base(0.0, 0.0, 0.0, 0.01);
                                        aux_ = 0;
                                    }

                                }
                                catch (tf2::TransformException &ex)
                                {
                                    ROS_WARN("%s", ex.what());
                                    ros::Duration(0.1).sleep();
                                    continue;
                                }
                            }
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
                feedback_.state = "FINISHED";
                as_.setSucceeded(result_);
            }
    };  

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_robot_station_server");
    PoseRobotStationAction action_server("pose_robot_station");
    ros::spin();
    return 0;
}