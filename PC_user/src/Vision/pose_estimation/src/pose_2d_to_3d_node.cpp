#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <pose_estimation/PersonPose2D.h>
#include <pose_estimation/PersonPose3D.h>
#include <pose_estimation/Keypoint2D.h>
#include <pose_estimation/Keypoint3D.h>

class Pose2Dto3D 
{
    public:
        Pose2Dto3D()
        {
            pose_sub_ = nh.subscribe("/vision/pose_2d", 10, &Pose2Dto3D::poseCallback, this);
            depth_sub_ = nh.subscribe("/camera/depth/image_raw", 10, &Pose2Dto3D::depthCallback, this);
            info_sub_ = nh.subscribe("/camera/depth/camera_info", 10, &Pose2Dto3D::infoCallback, this);
            
            pose_pub_ = nh.advertise<pose_estimation::PersonPose3D>("/vision/pose_3d", 10);
            
            std::cout <<"Keypoint 2D to 3D node --- Soft by Joshua M" << std::endl;
        }

    private:
        ros::NodeHandle nh;
        ros::Subscriber pose_sub_, depth_sub_, info_sub_;
        ros::Publisher pose_pub_;
        cv::Mat camera_matrix_;
        cv::Mat depth_image_;
        cv_bridge::CvImagePtr cv_ptr_;
        bool has_camera_matrix_ = false;

        void infoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
        {
            if (!has_camera_matrix_) {
                camera_matrix_ = cv::Mat(3, 3, CV_64F);
                for (int i = 0; i < 9; ++i) {
                    camera_matrix_.at<double>(i/3, i%3) = msg->K[i];
                }
                has_camera_matrix_ = true;
            }
        }

        void depthCallback(const sensor_msgs::Image::ConstPtr& msg)
        {
            try
            {
                cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
                depth_image_ = cv_ptr_->image;
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("Error to convert deep image: %s", e.what());
            }
        }

        void poseCallback(const pose_estimation::PersonPose2D::ConstPtr& msg)
        {
            if (!has_camera_matrix_ || depth_image_.empty())
            {
                return;
            }

            bool enabled = true;
            nh.getParam("/pose_2d_to_3d_enabled", enabled);
            if (!enabled)
                 return;

            pose_estimation::PersonPose3D person_3d;
            person_3d.id = msg->id;

            double fx = camera_matrix_.at<double>(0, 0);
            double fy = camera_matrix_.at<double>(1, 1);
            double cx = camera_matrix_.at<double>(0, 2);
            double cy = camera_matrix_.at<double>(1, 2);

            for (const auto& kp : msg->keypoints)
            {
                int x2d = static_cast<int>(std::round(kp.x));
                int y2d = static_cast<int>(std::round(kp.y));

                if (y2d >= 0 && y2d < depth_image_.rows && x2d >= 0 && x2d < depth_image_.cols)
                {
                    
                    float depth = depth_image_.at<unsigned short>(y2d, x2d) / 1000.0f; // mm -> metros

                    if (depth > 0)
                    {
                        pose_estimation::Keypoint3D kp3d;
                        kp3d.name = kp.name;
                        kp3d.x = (x2d - cx) * depth / fx;
                        kp3d.y = (y2d - cy) * depth / fy;
                        kp3d.z = depth;
                        kp3d.confidence = kp.confidence;

                        person_3d.keypoints.push_back(kp3d);
                    }
                }
            }

            if (!person_3d.keypoints.empty())
            {
                pose_pub_.publish(person_3d);
            }
        }
    };

    int main(int argc, char** argv)
    {
        ros::init(argc, argv, "pose_2d_to_3d_node");

        try
        {
            Pose2Dto3D converter;
            ros::spin();
        }
        catch (const std::exception& e)
        {
            ROS_ERROR("Node error: %s", e.what());
        }
    
    return 0;
}