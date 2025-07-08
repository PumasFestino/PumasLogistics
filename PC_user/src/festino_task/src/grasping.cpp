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
    SM_INIT,                // Initial state.
    
    // ---- BAND/PLATFORM DETECTION ----
    SM_FIND_PLATFORM,       // Moves manipulator forward to locate the platform.
    SM_FIND_BAND,           // Scans to detect the conveyor band's position.
    SM_ALIGN_WITH_BAND,     // Centers the manipulator over the band.

    // ---- PIECE HANDLING ----
    SM_FIND_PIECE,          // Identifies the target piece.
    SM_FIND_END_BAND,       // Detects the end of the band to avoid out-of-bounds errors.
    SM_ALIGN_WITH_PIECE,    // Positions manipulator directly above the piece.

    // ---- GRIPPER ACTIONS ----
    SM_DOWN_GRIPPER,        // Lowers the manipulator to picking/dropping height.
    SM_GRIPPER_ACTION,      // Opens or closes gripper to grasp the piece.
    SM_UP_GRIPPER,          // Lifts the manipulator after picking/dropping.

    // ---- FINALIZATION ----
    SM_GO_HOME,             // Returns manipulator to home position.
    SM_FINISH               // End of cycle.
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

    bool success_;

    SMState state_;
    std::string action_manip_;
    std::string action_name_;

    // For finding platform
    float move_x_center = 25.0;
    float move_y_center = 60.0;

    // For proportional control
    float error_x = 0.0;
    float error_y = 0.0;
    float threshold = 100;
    float threshold_1 = 50;

    // For aligning with the piece
    float move_x_piece = 15.0;
    float move_y_piece = 15.0;
    float move_z_piece = 25.0;

    // For end of band alignment
    float move_x_band = 15.0;
    float move_y_band = 15.0;

    // Gripper control
    bool gripper_open = true;

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
                    state_ = (action_manip_ == "takep" || action_manip_ == "dropp") ? SM_FIND_PLATFORM : SM_FIND_BAND;
                    break;

                case SM_FIND_PLATFORM:
                    current_state = "SM_FIND_PLATFORM --- Init";
                    FestinoHardware::move_manipulator(move_x_center, move_y_center, 0.0);
                    do {
                        current_state = "SM_FIND_PLATFORM --- Move manipulator";
                        std::tie(error_x, error_y) = FestinoVision::findPlatform();
                        FestinoHardware::move_manipulator(error_x, error_y, 0.0);
                    } while (error_y < threshold);
                    state_ = SM_FIND_PIECE;
                    break;

                case SM_FIND_BAND:
                    current_state = "SM_FIND_BAND --- Init";
                    FestinoHardware::move_manipulator(move_x_center, move_y_center, 0.0);
                    do {
                        current_state = "SM_FIND_BAND --- Move manipulator";
                        
                        error_x = FestinoVision::findBand();
                        std::cout << "Holiiis:" << error_x <<std::endl;
                        FestinoHardware::move_manipulator(-error_x*0.1, 0.0, 0.0);
                        ros::Duration(0.3).sleep();
                    } while (abs(error_x) > threshold);
                    state_ = SM_ALIGN_WITH_BAND;
                    break;

                case SM_ALIGN_WITH_BAND:
                    current_state = "SM_ALIGN_WITH_BAND --- Centering";
                    do {
                        error_y = FestinoVision::centerBand();
                        FestinoHardware::move_manipulator(0.0, error_y*0.1, 0.0);
                        ros::Duration(0.3).sleep();
                        
                    } while (abs(error_x) > threshold_1);
                    state_ = (action_manip_ == "drop") ? SM_FIND_END_BAND : SM_FIND_PIECE;
                    break;

                case SM_FIND_PIECE:
                    current_state = "SM_FIND_PIECE --- Locating";
                    do {
                        error_y = FestinoVision::findPiece();
                        FestinoHardware::move_manipulator(0.0, error_y*0.1, 0.0);
                        ros::Duration(0.3).sleep();
                    } while (abs(error_y) > threshold_1);
                    state_ = SM_ALIGN_WITH_PIECE;
                    break;

                case SM_FIND_END_BAND:
                    current_state = "SM_FIND_END_BAND --- Searching";
                    do {
                        error_y = FestinoVision::findEndBand();
                        FestinoHardware::move_manipulator(0.0, error_y*0.1, 0.0);
                        ros::Duration(0.3).sleep();
                    } while (abs(error_y) > threshold);
                    FestinoHardware::move_manipulator(move_x_band, move_y_band, 0.0);
                    state_ = SM_DOWN_GRIPPER;
                    break;

                case SM_ALIGN_WITH_PIECE:
                    current_state = "SM_ALIGN_WITH_PIECE --- Aligning";
                    FestinoHardware::move_manipulator(move_x_piece, move_y_piece, 0.0);
                    state_ = SM_DOWN_GRIPPER;
                    break;

                case SM_DOWN_GRIPPER:
                    current_state = "SM_DOWN_GRIPPER --- Lowering";
                    FestinoHardware::move_manipulator(0.0, 0.0, move_z_piece);
                    state_ = SM_GRIPPER_ACTION;
                    break;

                case SM_GRIPPER_ACTION:
                    current_state = std::string("SM_GRIPPER_ACTION --- ") +
                        ((action_manip_ == "take" || action_manip_ == "takep") ? "close" : "open");
                    FestinoHardware::move_gripper((action_manip_ == "take" || action_manip_ == "takep") ? gripper_open : !gripper_open);
                    state_ = SM_UP_GRIPPER;
                    break;

                case SM_UP_GRIPPER:
                    current_state = "SM_UP_GRIPPER --- Lifting";
                    FestinoHardware::move_manipulator(0.0, 0.0, -move_z_piece);
                    state_ = SM_GO_HOME;
                    break;

                case SM_GO_HOME:
                    current_state = "SM_GO_HOME --- Returning home";
                    FestinoHardware::move_manipulator_home(true);
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
