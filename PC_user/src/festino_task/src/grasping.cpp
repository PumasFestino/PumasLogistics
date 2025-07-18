// -------------------------------------------------------------------------------------------- //
// ----- This action handles the manipulator to pick or drop pieces using vision and control --- //
// -------------------------------------------------------------------------------------------- //

// ----- ROS Libraries ----- //
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>

// ----- Action Library ----- //
#include <festino_task/graspingAction.h>

// ----- Festino tools ----- //
#include <festino_tools/FestinoNavigation.h>
#include <festino_tools/FestinoVision.h>
#include <festino_tools/FestinoHardware.h>

enum SMState
{
    SM_INIT,
    SM_MOVE_TO_PLATFORM,
    SM_NOT_MOVE_TO_PLATFORM,

    SM_FIND_PLATFORM,
    SM_ALIGN_WITH_PLATFORM,

    SM_FIND_PIECE,
    SM_ALIGN_WITH_PIECE,

    SM_FIND_END_BAND,
    SM_ALIGN_WITH_END_BAND,

    SM_PLACE_OVER_PIECE,
    SM_DOWN_GRIPPER,
    SM_GRIPPER_ACTION,
    SM_UP_GRIPPER,

    SM_GO_HOME,
    SM_FINISH
};

class GraspingAction
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<festino_task::graspingAction> as_;
    
    festino_task::graspingFeedback feedback_;
    festino_task::graspingResult result_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    SMState state_;
    bool success_;
    bool gripper_status = true;

    std::string action_name_;
    std::string action_manip_;

    // Initial manipulator positioning
    float move_x_out = 5.0;
    float move_y_out = 60.0;

    // For aligning with the piece
    float move_x_piece = 35.0;
    float move_y_piece = 55.0;
    float move_z_piece = 85.0;

    // Proportional control
    float error_x = 0.0;
    float error_y = 0.0;

    // Vision centering
    float centroid_x = 0.0;
    float centroid_y = 0.0;
    const int target_x = 640;
    const int target_y = 360;
    const float tolerance = 0.05f;
    const int min_x = target_x - static_cast<int>(target_x * tolerance);
    const int max_x = target_x + static_cast<int>(target_x * tolerance);
    const int min_y = target_y - static_cast<int>(target_y * tolerance);
    const int max_y = target_y + static_cast<int>(target_y * tolerance);

    // Instructions
    std::string take   = "take";
    std::string takep  = "takep";
    std::string drop   = "drop";
    std::string dropp  = "dropp";

public:
    GraspingAction(std::string name)
        : as_(nh_, name, boost::bind(&GraspingAction::executeCB, this, _1), false),
          action_name_(name),
          tf_listener_(tf_buffer_),
          success_(false),
          state_(SM_INIT)
    {
        FestinoNavigation::setNodeHandle(&nh_);
        FestinoVision::setNodeHandle(&nh_);
        FestinoHardware::setNodeHandle(&nh_);
        as_.start();
        ROS_INFO("ACTION SERVER [%s] started.", action_name_.c_str());
    }

    void executeCB(const festino_task::graspingGoalConstPtr &goal)
    {
        action_manip_ = goal->action_manip;
        ros::Rate rate(10);
        std::string current_state = "";

        while (ros::ok() && !success_ && !as_.isPreemptRequested())
        {
            switch (state_)
            {
                case SM_INIT:
                    current_state = "SM_INIT";

                    FestinoHardware::move_gripper((action_manip_ == "take" || action_manip_ == "takep") ? !gripper_status : gripper_status);
                    
                    state_ = (action_manip_ == "takep" || action_manip_ == "dropp") ? SM_MOVE_TO_PLATFORM : SM_NOT_MOVE_TO_PLATFORM;
                    break;

                case SM_MOVE_TO_PLATFORM:
                    current_state = "SM_MOVE_TO_PLATFORM --- Init";
                    FestinoNavigation::move_base(0, -1, 0.1, 0.15);
                    FestinoHardware::move_manipulator(move_x_out, move_y_out, 0.0);
                    state_ = SM_FIND_PLATFORM;
                    break;

                case SM_FIND_PLATFORM:
                    current_state = "SM_FIND_PLATFORM --- Init";
                    std::tie(centroid_x, centroid_y) = FestinoVision::find("platform");
                    std::cout << "FestinoGrasping -> centroid x: " << centroid_x << "; centroid_y: " << centroid_y << std::endl;
                    state_ = (action_manip_ == "takep") ? SM_FIND_PIECE : SM_ALIGN_WITH_PLATFORM;
                    break;

                case SM_ALIGN_WITH_PLATFORM:
                    current_state = "SM_ALIGN_WITH_PLATFORM --- Centering";
                    if (centroid_x >= min_x && centroid_x <= max_x && centroid_y >= min_y && centroid_y <= max_y)
                    {
                        std::cout << "FestinoGrasping -> Centered done" << std::endl;
                        ros::Duration(0.7).sleep();
                        state_ = SM_PLACE_OVER_PIECE;
                    }
                    else
                    {
                        error_x = -0.15f * (centroid_x - target_x);
                        error_y = 0.15f * (target_y - centroid_y);
                        std::cout << "FestinoGrasping -> error x: " << error_x << "; error_y: " << error_y << std::endl;
                        FestinoHardware::move_manipulator(error_x, error_y, 0.0);
                        ros::Duration(0.7).sleep();
                        state_ = SM_FIND_PLATFORM;
                    }
                    break;

                case SM_NOT_MOVE_TO_PLATFORM:
                    current_state = "SM_NOT_MOVE_TO_PLATFORM --- Init";  
                    FestinoHardware::move_manipulator(move_x_out, move_y_out, 0.0);
                    ros::Duration(0.7).sleep();
                    state_ = (action_manip_ == "take") ? SM_FIND_PIECE : SM_FIND_END_BAND;
                    break;

                case SM_FIND_PIECE:
                    current_state = "SM_FIND_PIECE --- Init";    
                    std::tie(centroid_x, centroid_y) = FestinoVision::find("piece");
                    std::cout << "FestinoGrasping -> centroid x: " << centroid_x << "; centroid_y: " << centroid_y << std::endl;
                    state_ = SM_ALIGN_WITH_PIECE;
                    break;

                case SM_ALIGN_WITH_PIECE:
                    current_state = "SM_ALIGN_WITH_PIECE --- Centering";
                    if (centroid_x >= min_x && centroid_x <= max_x && centroid_y >= min_y && centroid_y <= max_y)
                    {
                        std::cout << "FestinoGrasping -> Centered done" << std::endl;
                        ros::Duration(0.7).sleep();
                        state_ = SM_PLACE_OVER_PIECE;
                    }
                    else
                    {
                        error_x = -0.15f * (centroid_x - target_x);
                        error_y = 0.15f * (target_y - centroid_y);
                        std::cout << "FestinoGrasping -> error x: " << error_x << "; error_y: " << error_y << std::endl;
                        FestinoHardware::move_manipulator(error_x, error_y, 0.0);
                        ros::Duration(0.7).sleep();
                        state_ = SM_FIND_PIECE;
                    }
                    break;

                case SM_FIND_END_BAND:
                    current_state = "SM_FIND_END_BAND --- Searching";
                    std::tie(centroid_x, centroid_y) = FestinoVision::find("band");
                    std::cout << "FestinoGrasping -> centroid x: " << centroid_x << "; centroid_y: " << centroid_y << std::endl;
                    state_ = SM_ALIGN_WITH_END_BAND;
                    break;

                case SM_ALIGN_WITH_END_BAND:
                    current_state = "SM_ALIGN_WITH_END_BAND --- Centering";
                    if (centroid_x >= min_x && centroid_x <= max_x && centroid_y >= min_y && centroid_y <= max_y)
                    {
                        std::cout << "FestinoGrasping -> Centered done" << std::endl;
                        ros::Duration(0.7).sleep();
                        state_ = SM_PLACE_OVER_PIECE;
                    }
                    else
                    {
                        error_x = -0.15f * (centroid_x - target_x);
                        error_y = 0.15f * (target_y - centroid_y);
                        std::cout << "FestinoGrasping -> error x: " << error_x << "; error_y: " << error_y << std::endl;
                        FestinoHardware::move_manipulator(error_x, error_y, 0.0);
                        ros::Duration(0.7).sleep();
                        state_ = SM_FIND_END_BAND;
                    }
                    break;

                case SM_PLACE_OVER_PIECE:
                    current_state = "SM_PLACE_OVER_PIECE --- Aligning";
                    FestinoHardware::move_manipulator(move_x_piece, move_y_piece, 0.0);
                    ros::Duration(0.7).sleep();
                    state_ = SM_DOWN_GRIPPER;
                    break;

                case SM_DOWN_GRIPPER:
                    current_state = "SM_DOWN_GRIPPER --- Lowering";
                    FestinoHardware::move_manipulator(0.0, 0.0, move_z_piece);
                    ros::Duration(0.7).sleep();
                    state_ = SM_GRIPPER_ACTION;
                    break;

                case SM_GRIPPER_ACTION:
                    current_state = std::string("SM_GRIPPER_ACTION --- ") +
                        ((action_manip_ == "take" || action_manip_ == "takep") ? "close" : "open");
                    FestinoHardware::move_gripper((action_manip_ == "take" || action_manip_ == "takep") ? gripper_status : !gripper_status);
                    ros::Duration(1.0).sleep();
                    state_ = SM_UP_GRIPPER;
                    break;

                case SM_UP_GRIPPER:
                    current_state = "SM_UP_GRIPPER --- Lifting";
                    FestinoHardware::move_manipulator(0.0, 0.0, -move_z_piece);
                    ros::Duration(0.7).sleep();
                    state_ = SM_GO_HOME;
                    break;

                case SM_GO_HOME:
                    current_state = "SM_GO_HOME --- Returning home";
                    FestinoHardware::move_manipulator_home(true);
                    ros::Duration(0.7).sleep();
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
    ros::init(argc, argv, "grasping_server");
    GraspingAction action_server("grasping");
    ros::spin();
    return 0;
}
