#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <tf2_ros/static_transform_broadcaster.h> 
#include <tf2_ros/transform_broadcaster.h> 
#include <tf2_ros/transform_listener.h>          
#include <tf2_ros/buffer.h>                      
#include <tf2/transform_datatypes.h>   
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "img_proc/Tag_with_tf.h"
#include <Eigen/Dense>

class ArucoDistanceTF
{
    private:
            ros::NodeHandle nh_;
            image_transport::ImageTransport it_;
            image_transport::Subscriber image_sub_;
            ros::Subscriber pointcloud_sub_;
            ros::ServiceServer service_;
            tf2_ros::TransformBroadcaster tf_broadcaster_;

            cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
            cv::Ptr<cv::aruco::DetectorParameters> aruco_params_;

            sensor_msgs::PointCloud2::ConstPtr latest_pointcloud_;
            cv::Mat latest_image_;
            
            std::vector<std::string> mps_names;
            bool success;


    public:
            ArucoDistanceTF() : nh_("~"), it_(nh_), success(false)
            {
                aruco_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
                aruco_params_ = cv::aruco::DetectorParameters::create();

                image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &ArucoDistanceTF::imageCallback, this);
                pointcloud_sub_ = nh_.subscribe("/camera/depth/points", 1, &ArucoDistanceTF::pointCloudCallback, this);
                service_ = nh_.advertiseService("/vision/find_tag", &ArucoDistanceTF::getArucoTFService, this);;
                
                std::cout << "Aruco with TF Service --- Soft by Joshua M" << std::endl;
            }

            void imageCallback(const sensor_msgs::ImageConstPtr& msg)
            {
                try
                {
                    latest_image_ = cv_bridge::toCvShare(msg, "bgr8")->image;
                }
                catch (cv_bridge::Exception& e)
                {
                    ROS_ERROR("Error to convert image: %s", e.what());
                }
            }

            void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
            {
                latest_pointcloud_ = msg;
            }


            bool getArucoTFService(img_proc::Tag_with_tf::Request &req, img_proc::Tag_with_tf::Response &res)
            {
                
                res.success = false; 
                if (req.is_find_tag_enabled)
                {
                    process();
                    res.success = success;
                    res.mps_name = mps_names;
                }
                return true;
            }

            void process()
            {
                success = false;
                if (latest_image_.empty() || !latest_pointcloud_)
                {
                    ROS_INFO("Waiting image data and point cloud...");
                    
                    return;
                }

                cv::Mat gray;
                cv::cvtColor(latest_image_, gray, cv::COLOR_BGR2GRAY);

                std::vector<std::vector<cv::Point2f>> corners;
                std::vector<int> ids;
                cv::aruco::detectMarkers(gray, aruco_dict_, corners, ids, aruco_params_);

                std::vector<geometry_msgs::Point> plane_x;

                if (!ids.empty())
                {
                    mps_names.clear();
                    for (size_t i = 0; i < ids.size(); ++i)
                    {
                        plane_x.clear();

                        for (const auto& corner : corners[i])
                        {
                            plane_x.push_back(getPointFromCloud(corner.x, corner.y));
                        }

                        if (plane_x.size() == 4) 
                        {
                            
                            geometry_msgs::Point centroid_3d;
                            centroid_3d.x = (plane_x[0].x + plane_x[1].x + plane_x[2].x + plane_x[3].x) / 4.0;
                            centroid_3d.y = (plane_x[0].y + plane_x[1].y + plane_x[2].y + plane_x[3].y) / 4.0;
                            centroid_3d.z = (plane_x[0].z + plane_x[1].z + plane_x[2].z + plane_x[3].z) / 4.0;

                        
                            Eigen::Vector3d v1(plane_x[1].x - plane_x[0].x, plane_x[1].y - plane_x[0].y, plane_x[1].z - plane_x[0].z);
                            Eigen::Vector3d v2(plane_x[3].x - plane_x[0].x, plane_x[3].y - plane_x[0].y, plane_x[3].z - plane_x[0].z);

                            
                            Eigen::Vector3d normal = v1.cross(v2);
                            normal.normalize();

                            
                            double yaw = atan2(normal.y(), normal.x());

                            
                            tf2::Quaternion q;
                            q.setRPY(0, 0, yaw);

                            std::cout << "Coord 3D from marker " << ids[i] << ": x=" << centroid_3d.x 
                                    << ", y=" << centroid_3d.y << ", z=" << centroid_3d.z 
                                    << ", yaw=" << yaw << std::endl;

                            mps_names.push_back(std::to_string(ids[i]));
                            publishTF(centroid_3d.x, centroid_3d.y, centroid_3d.z, q, ids[i]);
                        }
                        else
                        {
                            ROS_WARN("Not enough corner points detected.");
                            success = false;
                        }
                    }   
                }
                else
                {
                    std::cout << "Aruco with TF Service --- No tag detected" << std::endl;
                    success = false;
                }
            }

            geometry_msgs::Point getPointFromCloud(float u, float v)
            {
                geometry_msgs::Point point;

                if (!latest_pointcloud_)
                {
                    point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN();
                    success = false;
                    return point;
                }

                int width = latest_pointcloud_->width;
                int height = latest_pointcloud_->height;

                if (u < 0 || v < 0 || u >= width || v >= height)
                {
                    ROS_WARN("Coordinates (%f, %f) are outside of point cloud", u, v);
                    point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN();
                    success= false;
                    return point;
                }

                int index = static_cast<int>(v) * width + static_cast<int>(u);

                sensor_msgs::PointCloud2ConstIterator<float> iter_x(*latest_pointcloud_, "x");
                sensor_msgs::PointCloud2ConstIterator<float> iter_y(*latest_pointcloud_, "y");
                sensor_msgs::PointCloud2ConstIterator<float> iter_z(*latest_pointcloud_, "z");

                for (int i = 0; i < index; ++i) {
                    ++iter_x;
                    ++iter_y;
                    ++iter_z;
                }

                point.x = *iter_z;
                point.y = *iter_x;
                point.z = *iter_y;

                point.y = -point.y;
                point.x = point.x;
                point.z = -point.z;
                
                return point;
            }


            void publishTF_without_map(float x, float y, float z, const tf2::Quaternion& q, int marker_id)
            {
                static tf2_ros::StaticTransformBroadcaster static_broadcaster_;
                
                geometry_msgs::TransformStamped transform;
                transform.header.stamp = ros::Time::now();
                transform.header.frame_id = "camera_link";
                transform.child_frame_id = "aruco_marker_" + std::to_string(marker_id);

                transform.transform.translation.x = x;
                transform.transform.translation.y = y;
                transform.transform.translation.z = z;

                transform.transform.rotation.x = q.x();
                transform.transform.rotation.y = q.y();
                transform.transform.rotation.z = q.z();
                transform.transform.rotation.w = q.w();


                //tf_broadcaster_.sendTransform(transform);
                static_broadcaster_.sendTransform(transform);
                
                std::cout << "Publish TF for marker "<< marker_id <<" @ x: " << x << "; y: " << y << "; z:" << z << std::endl;
            }

            void publishTF(float x, float y, float z, const tf2::Quaternion& q, int marker_id)
            {
                static tf2_ros::StaticTransformBroadcaster static_broadcaster_;
                tf2_ros::Buffer tf_buffer;
                tf2_ros::TransformListener tf_listener(tf_buffer);

                try
                {
                    // Obtener la transformada de "map" a "camera_link"
                    geometry_msgs::TransformStamped map_to_camera;
                    map_to_camera = tf_buffer.lookupTransform("map", "camera_link", ros::Time(0), ros::Duration(1.0));

                    // Aplicar la transformación a las coordenadas
                    tf2::Vector3 marker_in_camera(x, y, z);
                    tf2::Transform transform_map_to_camera;
                    tf2::fromMsg(map_to_camera.transform, transform_map_to_camera);

                    tf2::Vector3 marker_in_map = transform_map_to_camera * marker_in_camera;

                    // Crear el mensaje de transformada estática
                    geometry_msgs::TransformStamped transform;
                    transform.header.stamp = ros::Time::now();
                    transform.header.frame_id = "map";
                    transform.child_frame_id = "aruco_marker_" + std::to_string(marker_id);

                    transform.transform.translation.x = marker_in_map.x();
                    transform.transform.translation.y = marker_in_map.y();
                    transform.transform.translation.z = marker_in_map.z();

                    transform.transform.rotation.x = q.x();
                    transform.transform.rotation.y = q.y();
                    transform.transform.rotation.z = q.z();
                    transform.transform.rotation.w = q.w();
                    
                    // Publicar la transformada estática
                    static_broadcaster_.sendTransform(transform);

                    std::cout << "Publish static TF for marker " << marker_id 
                            << " @ x: " << marker_in_map.x()
                            << "; y: " << marker_in_map.y()
                            << "; z: " << marker_in_map.z()
                            << " referenced to map." << std::endl;
                    success = true;
                }
                catch (tf2::TransformException& ex)
                {
                    ROS_WARN("Could not transform from 'map' to 'camera_link': %s", ex.what());
                    success = false;
                }
            }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "aruco_distance_tf_service");
    ArucoDistanceTF node;
    ros::Rate rate(100000);

    ros::spin();
    return 0;
}