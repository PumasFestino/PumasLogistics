#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <tf2_ros/transform_broadcaster.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>


class ArucoDistanceTF
{
    private:
            ros::NodeHandle nh_;
            image_transport::ImageTransport it_;
            image_transport::Subscriber image_sub_;
            ros::Subscriber pointcloud_sub_;
            tf2_ros::TransformBroadcaster tf_broadcaster_;

            cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
            cv::Ptr<cv::aruco::DetectorParameters> aruco_params_;

            sensor_msgs::PointCloud2::ConstPtr latest_pointcloud_;
            cv::Mat latest_image_;

    public:
            ArucoDistanceTF() : nh_("~"), it_(nh_)
            {
                // Inicializar el diccionario y los parámetros de ArUco
                aruco_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
                aruco_params_ = cv::aruco::DetectorParameters::create();

                // Suscriptores
                image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &ArucoDistanceTF::imageCallback, this);
                pointcloud_sub_ = nh_.subscribe("/camera/depth/points", 1, &ArucoDistanceTF::pointCloudCallback, this);

                ROS_INFO("ArUco Node with TF... start");
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

            void process()
            {
                if (latest_image_.empty() || !latest_pointcloud_)
                {
                    ROS_INFO("Waiting image data and point cloud...");
                    
                    return;
                }

                // Convertir imagen a escala de grises
                cv::Mat gray;
                cv::cvtColor(latest_image_, gray, cv::COLOR_BGR2GRAY);

                // Detectar marcadores ArUco
                std::vector<std::vector<cv::Point2f>> corners;
                std::vector<int> ids;
                cv::aruco::detectMarkers(gray, aruco_dict_, corners, ids, aruco_params_);

                if (!ids.empty())
                {
                    for (size_t i = 0; i < ids.size(); ++i)
                    {
                        // Calcular centroide del marcador
                        cv::Point2f centroid(0, 0);
                        for (const auto& corner : corners[i])
                        {
                            centroid += corner;
                        }
                        centroid.x /= 4.0;
                        centroid.y /= 4.0;

                        std::cout<<"Centroide del marcador "<<ids[i]<<": ("<<centroid.x<<", "<<centroid.y<<std::endl;

                        // Obtener coordenadas 3D del centroide
                        geometry_msgs::Point point_3d = getPointFromCloud(centroid.x, centroid.y);

                        if (!std::isnan(point_3d.x) && !std::isnan(point_3d.y) && !std::isnan(point_3d.z))
                        {
                            std::cout<<"Coordenadas 3D del marcador "<< ids[i]<<": x="<< point_3d.x<<", y="<<point_3d.y<<", z="<<point_3d.z<<std::endl;
                            publishTF(point_3d.x, point_3d.y, point_3d.z, ids[i]);
                        }
                        else
                        {
                            ROS_WARN("No se encontro un punto valido en la nube para el marcador %d.", ids[i]);
                        }
                    }
                }
            }

            geometry_msgs::Point getPointFromCloud(float u, float v)
            {
                geometry_msgs::Point point;

                if (!latest_pointcloud_) {
                    point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN();
                    return point;
                }

                int width = latest_pointcloud_->width;
                int height = latest_pointcloud_->height;

                if (u < 0 || v < 0 || u >= width || v >= height) {
                    ROS_WARN("Coordenadas (%f, %f) estan fuera de los limites de la nube de puntos.", u, v);
                    point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN();
                    return point;
                }

                // Índice lineal en la nube de puntos
                int index = static_cast<int>(v) * width + static_cast<int>(u);

                // Iterar sobre la nube de puntos
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


            void publishTF(float x, float y, float z, int marker_id)
            {
                geometry_msgs::TransformStamped transform;
                transform.header.stamp = ros::Time::now();
                transform.header.frame_id = "camera_link";
                transform.child_frame_id = "aruco_marker_" + std::to_string(marker_id);

                transform.transform.translation.x = x;
                transform.transform.translation.y = y;
                transform.transform.translation.z = z;

                transform.transform.rotation.x = 0.0;
                transform.transform.rotation.y = 0.0;
                transform.transform.rotation.z = 0.0;
                transform.transform.rotation.w = 1.0;

                tf_broadcaster_.sendTransform(transform);
                ROS_INFO("Publicado TF para marcador %d en (%f, %f, %f)", marker_id, x, y, z);
            }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "aruco_distance_tf");
    ArucoDistanceTF node;

    ros::Rate rate(10000); 
    
    while (ros::ok())
    {
        node.process();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}