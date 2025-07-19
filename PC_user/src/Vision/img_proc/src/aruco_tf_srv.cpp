#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <Eigen/Dense>
#include <queue>
#include "img_proc/Tag_with_tf.h"

class ArucoDistanceTF
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Subscriber pointcloud_sub_, scan_sub_;
    ros::ServiceServer service_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::StaticTransformBroadcaster static_broadcaster_;

    cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
    cv::Ptr<cv::aruco::DetectorParameters> aruco_params_;

    sensor_msgs::PointCloud2::ConstPtr latest_pointcloud_;
    cv::Mat latest_image_;
    std::vector<std::pair<double, double>> laser_points_;

    std::vector<std::string> mps_names;
    bool success;

public:
    ArucoDistanceTF() : nh_("~"), it_(nh_), tf_listener_(tf_buffer_), success(false)
    {
        aruco_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
        aruco_params_ = cv::aruco::DetectorParameters::create();

        image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &ArucoDistanceTF::imageCallback, this);
        pointcloud_sub_ = nh_.subscribe("/camera/depth/points", 1, &ArucoDistanceTF::pointCloudCallback, this);
        scan_sub_ = nh_.subscribe("/scan", 1, &ArucoDistanceTF::scanCallback, this);
        service_ = nh_.advertiseService("/vision/find_tag", &ArucoDistanceTF::getArucoTFService, this);

        ROS_INFO("ArucoDistanceTF node initialized.");
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        try {
            latest_image_ = cv_bridge::toCvShare(msg, "bgr8")->image;
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("Image conversion failed: %s", e.what());
        }
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        latest_pointcloud_ = msg;
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        laser_points_.clear();
        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            float range = msg->ranges[i];
            if (range > msg->range_min && range < msg->range_max)
            {
                double angle = msg->angle_min + i * msg->angle_increment;
                double x = range * cos(angle);
                double y = range * sin(angle);
                laser_points_.emplace_back(x, y);
            }
        }
    }

    bool getArucoTFService(img_proc::Tag_with_tf::Request &req, img_proc::Tag_with_tf::Response &res)
    {
        res.success = false;
        if (req.is_find_tag_enabled)
        {
            process();
            res.success = success;
            res.mps_name = mps_names.empty() ? "" : mps_names[0];
        }
        return true;
    }

    geometry_msgs::Point getPointFromCloud(float u, float v)
    {
        geometry_msgs::Point point;
        if (!latest_pointcloud_) return point;

        int width = latest_pointcloud_->width;
        int height = latest_pointcloud_->height;

        if (u < 0 || v < 0 || u >= width || v >= height) return point;

        int index = static_cast<int>(v) * width + static_cast<int>(u);

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*latest_pointcloud_, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*latest_pointcloud_, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*latest_pointcloud_, "z");

        for (int i = 0; i < index; ++i) {
            ++iter_x; ++iter_y; ++iter_z;
        }

        point.x = *iter_z;
        point.y = *iter_x;
        point.z = *iter_y;
        
        point.y = -point.y;
        point.x =  point.x;
        point.z = -point.z;
        return point;
    }

    geometry_msgs::Point transformPointToLaser(const geometry_msgs::Point& p)
    {
        geometry_msgs::PointStamped in, out;
        in.header.frame_id = "camera_link";
        in.header.stamp = ros::Time(0);
        in.point = p;
        try {
            tf_buffer_.transform(in, out, "laser_link", ros::Duration(1.0));
            return out.point;
        } catch (tf2::TransformException& ex) {
            ROS_WARN("TF transform failed: %s", ex.what());
            geometry_msgs::Point invalid;
            invalid.x = invalid.y = invalid.z = std::numeric_limits<double>::quiet_NaN();
            return invalid;
        }
    }

    std::vector<std::vector<std::pair<double, double>>> clusterLaserPoints(double dist = 0.05, int min_pts = 5)
    {
        std::vector<std::vector<std::pair<double, double>>> clusters;
        std::vector<bool> visited(laser_points_.size(), false);

        for (size_t i = 0; i < laser_points_.size(); ++i)
        {
            if (visited[i]) continue;
            std::vector<std::pair<double, double>> cluster;
            std::queue<size_t> q;
            q.push(i);
            visited[i] = true;

            while (!q.empty())
            {
                size_t idx = q.front(); q.pop();
                cluster.push_back(laser_points_[idx]);

                for (size_t j = 0; j < laser_points_.size(); ++j)
                {
                    if (!visited[j])
                    {
                        double dx = laser_points_[idx].first - laser_points_[j].first;
                        double dy = laser_points_[idx].second - laser_points_[j].second;
                        if (std::hypot(dx, dy) < dist)
                        {
                            q.push(j);
                            visited[j] = true;
                        }
                    }
                }
            }

            if (cluster.size() >= min_pts)
                clusters.push_back(cluster);
        }

        return clusters;
    }

    bool findBestLineFromClusters(const geometry_msgs::Point& ref, double& angle_out)
    {
        auto clusters = clusterLaserPoints();
        double min_dist = std::numeric_limits<double>::max();
        int best_inliers = 0;
        double best_angle = 0.0;

        for (const auto& cluster : clusters)
        {
            if (cluster.size() < 10) continue;

            const int iterations = 50;
            const double threshold = 0.02;
            int best_cluster_inliers = 0;
            double cluster_angle = 0.0;

            for (int i = 0; i < iterations; ++i)
            {
                int a = rand() % cluster.size();
                int b = rand() % cluster.size();
                if (a == b) continue;

                auto [x1, y1] = cluster[a];
                auto [x2, y2] = cluster[b];
                double dx = x2 - x1, dy = y2 - y1, norm = std::hypot(dx, dy);
                if (norm == 0) continue;

                double A = dy, B = -dx, C = dx * y1 - dy * x1;
                int inliers = 0;
                for (const auto& [x, y] : cluster)
                {
                    double d = std::fabs(A * x + B * y + C) / norm;
                    if (d < threshold) inliers++;
                }

                if (inliers > best_cluster_inliers)
                {
                    best_cluster_inliers = inliers;
                    cluster_angle = atan2(dy, dx);
                }
            }

            double cx = 0, cy = 0;
            for (const auto& [x, y] : cluster) { cx += x; cy += y; }
            cx /= cluster.size(); cy /= cluster.size();
            double dist = std::hypot(ref.x - cx, ref.y - cy);

            if (best_cluster_inliers > 10 && dist < min_dist)
            {
                min_dist = dist;
                best_inliers = best_cluster_inliers;
                best_angle = cluster_angle;
            }
        }

        if (best_inliers > 10) {
            angle_out = best_angle;
            return true;
        }
        return false;
    }

    void publishTF(const geometry_msgs::Point& point, const tf2::Quaternion& q, int marker_id)
    {
        try {
            geometry_msgs::TransformStamped tf = tf_buffer_.lookupTransform("map", "camera_link", ros::Time(0), ros::Duration(1.0));

            tf2::Vector3 pos(point.x, point.y, point.z);
            tf2::Transform T_cam;
            tf2::fromMsg(tf.transform, T_cam);
            tf2::Vector3 pos_map = T_cam * pos;

            geometry_msgs::TransformStamped tf_out;
            tf_out.header.stamp = ros::Time::now();
            tf_out.header.frame_id = "map";
            tf_out.child_frame_id = "aruco_marker_" + std::to_string(marker_id);
            tf_out.transform.translation.x = pos_map.x();
            tf_out.transform.translation.y = pos_map.y();
            tf_out.transform.translation.z = pos_map.z();
            tf_out.transform.rotation = tf2::toMsg(q);

            static_broadcaster_.sendTransform(tf_out);

            ROS_INFO("Published static TF for marker %d", marker_id);
        } catch (tf2::TransformException& ex) {
            ROS_WARN("TF error in publishTF: %s", ex.what());
        }
    }

    void process()
    {
        success = false;
        if (latest_image_.empty() || !latest_pointcloud_) return;

        cv::Mat gray;
        cv::cvtColor(latest_image_, gray, cv::COLOR_BGR2GRAY);
        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<int> ids;
        cv::aruco::detectMarkers(gray, aruco_dict_, corners, ids, aruco_params_);

        if (ids.empty()) {
            ROS_WARN("No ArUco markers detected.");
            return;
        }

        geometry_msgs::Point closest;
        tf2::Quaternion orientation;
        double min_dist = std::numeric_limits<double>::max();
        int closest_id = -1;

        for (size_t i = 0; i < ids.size(); ++i)
        {
            std::vector<geometry_msgs::Point> pts;
            for (const auto& c : corners[i])
                pts.push_back(getPointFromCloud(c.x, c.y));

            bool valid = std::all_of(pts.begin(), pts.end(), [](const geometry_msgs::Point& p){
                return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
            });
            if (!valid) continue;

            geometry_msgs::Point centroid;
            for (const auto& p : pts) {
                centroid.x += p.x; centroid.y += p.y; centroid.z += p.z;
            }
            centroid.x /= 4.0; centroid.y /= 4.0; centroid.z /= 4.0;

            double dist = std::sqrt(centroid.x*centroid.x + centroid.y*centroid.y + centroid.z*centroid.z);
            if (dist < min_dist) {
                closest = centroid;
                closest_id = ids[i];
                min_dist = dist;
            }
        }

        if (closest_id == -1) return;

        orientation.setRPY(0, 0, 0); // fallback

        geometry_msgs::Point closest_laser = transformPointToLaser(closest);
        double corrected_yaw;
        if (std::isfinite(closest_laser.x) && findBestLineFromClusters(closest_laser, corrected_yaw))
        {
            orientation.setRPY(0, 0, corrected_yaw);
            ROS_INFO("Yaw corrected with LaserScan: %.3f", corrected_yaw);
        }

        publishTF(closest, orientation, closest_id);
        mps_names = {std::to_string(closest_id)};
        success = true;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "aruco_distance_tf_service");
    ArucoDistanceTF node;
    ros::spin();
    return 0;
}