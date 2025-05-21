#include <ros/ros.h>
#include <moveit_msgs/DisplayTrajectory.h>  // ¡Nuevo mensaje!
#include <std_msgs/Float64MultiArray.h>

class MoveItToHardwareBridge {
public:
    MoveItToHardwareBridge() {
        ros::NodeHandle nh;
        hardware_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/hardware/left_arm/goal_pose", 10);
        trajectory_sub_ = nh.subscribe("/move_group/display_planned_path", 1, 
                                     &MoveItToHardwareBridge::trajectoryCallback, this);
        ROS_INFO("Puente iniciado. Suscrito a %s", trajectory_sub_.getTopic().c_str());
    }

    void trajectoryCallback(const moveit_msgs::DisplayTrajectory::ConstPtr& msg) {
        if (msg->trajectory.empty() || msg->trajectory[0].joint_trajectory.points.empty()) {
            ROS_WARN("Trayectoria vacía. Ignorando.");
            return;
        }
    
        const auto& joint_trajectory = msg->trajectory[0].joint_trajectory;
        ROS_INFO("=== Trayectoria recibida (%lu puntos) ===", joint_trajectory.points.size());
    
        ros::Time start_time = ros::Time::now();
        ros::Duration prev_time(0.0);
    
        ros::Rate loop_rate(40);  // 100 Hz, o ajusta según lo que soporte tu hardware

for (const auto& point : joint_trajectory.points) {
    std_msgs::Float64MultiArray array_msg;
    array_msg.data = point.positions;
    hardware_pub_.publish(array_msg);
    loop_rate.sleep();
}

        ROS_INFO("Ejecutada trayectoria completa.");
    }
    
    

private:
    ros::Publisher hardware_pub_;
    ros::Subscriber trajectory_sub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "moveit_to_hardware_bridge_cpp");
    MoveItToHardwareBridge bridge;
    ros::spin();
    return 0;
}