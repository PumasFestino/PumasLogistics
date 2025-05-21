#include <ros/ros.h>
#include <std_msgs/String.h>
#include <pose_estimation/PersonPose3D.h>
#include <pose_estimation/Keypoint3D.h>
#include <unordered_map>
#include <Eigen/Dense>

class PointingDetector
{
public:
    PointingDetector()
    {
        sub_ = nh.subscribe("/vision/pose_3d", 10, &PointingDetector::poseCallback, this);
        pub_ = nh.advertise<std_msgs::String>("/vision/pointing_direction", 10);
        dist_threshold_ = 0.15;
        ROS_INFO("[pointing_detector_node] Init node.");
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    double dist_threshold_;

    void poseCallback(const pose_estimation::PersonPose3D::ConstPtr& msg)
    {
        std_msgs::String out_msg;
        out_msg.data = detectPointing(msg);
        pub_.publish(out_msg);
    }

    std::string detectPointing(const pose_estimation::PersonPose3D::ConstPtr& person_pose)
    {
        bool enabled = true;
        nh.getParam("/pointing_detector_enabled", enabled);
        if (!enabled)
                 return "none";
        // Mapear keypoints por nombre
        std::unordered_map<std::string, pose_estimation::Keypoint3D> keypoints;
        for (const auto& kp : person_pose->keypoints)
        {
            keypoints[kp.name] = kp;
        }

        // Funciones auxiliares
        auto toVec = [](const pose_estimation::Keypoint3D& a, const pose_estimation::Keypoint3D& b) {
            return Eigen::Vector3f(b.x - a.x, b.y - a.y, b.z - a.z);
        };

        auto norm = [](const Eigen::Vector3f& v) {
            return v.norm();
        };

        auto isPointing = [&](const pose_estimation::Keypoint3D& shoulder,
                              const pose_estimation::Keypoint3D& elbow,
                              const pose_estimation::Keypoint3D& wrist,
                              const pose_estimation::Keypoint3D& hip)
        {
            Eigen::Vector3f v1 = toVec(shoulder, elbow);
            Eigen::Vector3f v2 = toVec(elbow, wrist);

            float cos_angle = v1.normalized().dot(v2.normalized());
            float dist_hand_hip = norm(toVec(hip, wrist));

            bool arm_straight = (cos_angle > 0.95);
            bool hand_far = (dist_hand_hip > dist_threshold_);

            return arm_straight && hand_far;
        };

        try
        {
            bool left = isPointing(keypoints.at("shoulder_left"),
                                   keypoints.at("elbow_left"),
                                   keypoints.at("wrist_left"),
                                   keypoints.at("hip_left"));

            bool right = isPointing(keypoints.at("shoulder_right"),
                                    keypoints.at("elbow_right"),
                                    keypoints.at("wrist_right"),
                                    keypoints.at("hip_right"));

            if (left && !right)
            {
                std::cout << "Pointing with left hand" << std::endl;
                return "left";
            }
            else if (right && !left)
            {
                std::cout << "Pointing with right hand" << std::endl;
                return "right";
            }
            else
            {
                return "none";
            }
        }
        catch (const std::out_of_range& e)
        {
            //ROS_WARN("Faltan keypoints: %s", e.what());
            return "none";
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointing_detector_node");
    PointingDetector pd;
    ros::spin();
    return 0;
}
