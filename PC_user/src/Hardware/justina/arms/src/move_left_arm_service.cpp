#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <arms/MoveArm.h>  // ¡Ahora incluye el nuevo campo pose_name!

class ArmMover {
public:
    ArmMover() : move_group("left_arm") {
        move_group.setPlanningTime(10.0);
    }

    // Mover a una pose cartesiana (como antes)
    bool moveToPose(double x, double y, double z, double roll, double pitch, double yaw) {
        geometry_msgs::Pose target_pose;
        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;
        
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        q.normalize();
        target_pose.orientation.x = q.x();
        target_pose.orientation.y = q.y();
        target_pose.orientation.z = q.z();
        target_pose.orientation.w = q.w();
        
        move_group.setPoseTarget(target_pose);
        
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
        if (success) {
            move_group.move();
        }
        
        move_group.clearPoseTargets();
        return success;
    }

    // Mover a una pose predefinida (nuevo)
    bool moveToNamedPose(const std::string& pose_name) {
        move_group.setNamedTarget(pose_name);
        
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
        if (success) {
            move_group.move();
        }
        
        return success;
    }

private:
    moveit::planning_interface::MoveGroupInterface move_group;
};

bool handle_move_request(arms::MoveArm::Request &req,
                         arms::MoveArm::Response &res,
                         ArmMover* mover) {
    // Si se especificó pose_name, usamos la pose predefinida
    if (!req.pose_name.empty()) {
        ROS_INFO("Movimiento a pose predefinida: %s", req.pose_name.c_str());
        bool success = mover->moveToNamedPose(req.pose_name);
        res.success = success;
        res.message = success ? "Pose predefinida ejecutada." : "Error en pose predefinida.";
    } 
    // Si no, movimiento cartesiano normal
    else {
        ROS_INFO("Movimiento cartesiano: x=%.2f, y=%.2f, z=%.2f", req.x, req.y, req.z);
        bool success = mover->moveToPose(req.x, req.y, req.z, req.roll, req.pitch, req.yaw);
        res.success = success;
        res.message = success ? "Movimiento exitoso." : "Error en movimiento.";
    }
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "move_left_arm_service_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);
    spinner.start();

    ArmMover mover;
    ros::ServiceServer service = nh.advertiseService<arms::MoveArm::Request, arms::MoveArm::Response>(
        "move_left_arm",
        boost::bind(&handle_move_request, _1, _2, &mover));

    ROS_INFO("Servicio listo (ahora soporta poses predefinidas y cartesianas).");
    ros::waitForShutdown();
    return 0;
}