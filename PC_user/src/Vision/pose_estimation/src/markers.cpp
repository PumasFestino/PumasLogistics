#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pose_estimation/PersonPose3D.h>
#include <pose_estimation/Keypoint3D.h>

class Pose3DVisualizer {
public:
    Pose3DVisualizer() {
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");
        
        // Obtener parámetro frame_id, con valor por defecto
        private_nh.param<std::string>("frame_id", frame_id_, "camera_depth_optical_frame");
        
        // Inicializar suscriptor y publicador
        sub_ = nh.subscribe("/pose_3d", 10, &Pose3DVisualizer::poseCallback, this);
        pub_ = nh.advertise<visualization_msgs::MarkerArray>("/pose_3d/markers", 10);
        
        ROS_INFO("Nodo visualizador 3D iniciado");
    }

    void poseCallback(const pose_estimation::PersonPose3D::ConstPtr& msg) {
        visualization_msgs::MarkerArray marker_array;
        ros::Time timestamp = ros::Time::now();

        for (size_t i = 0; i < msg->keypoints.size(); ++i) {
            const auto& kp = msg->keypoints[i];
            
            visualization_msgs::Marker marker;
            marker.header.frame_id = frame_id_;
            marker.header.stamp = timestamp;
            marker.ns = "person_" + std::to_string(msg->id);
            marker.id = i;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            
            marker.pose.position.x = kp.x;
            marker.pose.position.y = kp.y;
            marker.pose.position.z = kp.z;
            
            marker.scale.x = 0.05;  // 5cm
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;
            
            marker.color.r = 1.0;
            marker.color.g = 0.5;
            marker.color.b = 0.0;
            marker.color.a = 0.8;
            
            marker.lifetime = ros::Duration(0.2);
            
            marker_array.markers.push_back(marker);
        }

        pub_.publish(marker_array);
    }

private:
    ros::Subscriber sub_;
    ros::Publisher pub_;
    std::string frame_id_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_3d_visualizer");
    
    Pose3DVisualizer visualizer;
    
    ros::spin();
    
    return 0;
}