#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32MultiArray.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <festino_tools/FestinoNavigation.h>
#include <festino_tools/FestinoVision.h>
#include <festino_tools/FestinoKnowledge.h>
#include <cmath>

int main(int argc, char **argv)
{
    geometry_msgs::TransformStamped coord_Aruco;
    geometry_msgs::TransformStamped coord_cam_robot;
    float error_x = 0.0;
    float error_y = 0.0;
    int aux = 1;
    std::vector<float> goal_vec(3);
    
    std::cout << "ACTION - Aling with MPS... Soft by Joshua M" << std::endl;
    ros::init(argc, argv, "SM");
    ros::NodeHandle nh;
    ros::Rate rate(100000);

    FestinoNavigation::setNodeHandle(&nh);
    FestinoVision::setNodeHandle(&nh);
    FestinoKnowledge::setNodeHandle(&nh);

    /*goal_vec = FestinoKnowledge::CoordenatesLocSrv("station-test");
    std::cout <<"Coordenates of inspection_point:"<<std::endl;
    std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
    if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
    if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2], 120000))
    std::cout << "Cannot move to inspection point" << std::endl;*/

    FestinoVision::enableArucoDet(true);

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);


    do
    {
        try
        {
            if (tf_buffer.canTransform("map", "aruco_marker_107", ros::Time(0), ros::Duration(0.1)))
            {
                coord_Aruco = tf_buffer.lookupTransform("map", "aruco_marker_107", ros::Time(0), ros::Duration(0.1));
            }else
            {
                ROS_WARN("Transform for aruco_marker_107 not available");
                continue;
            }
            if (tf_buffer.canTransform("map", "camera_link", ros::Time(0), ros::Duration(0.1)))
            {
                coord_cam_robot = tf_buffer.lookupTransform("map", "camera_link", ros::Time(0), ros::Duration(0.1));
            }else
            {
                ROS_WARN("Transform for camera_link not available");
                continue;
            }
            std::cout << "Aruco x:" << coord_Aruco.transform.translation.x << " -- Robot x:" << coord_cam_robot.transform.translation.x << std::endl;
            std::cout << "Aruco y:" << coord_Aruco.transform.translation.y << " -- Robot y:" << coord_cam_robot.transform.translation.y << std::endl;
            std::cout << "Aruco - robot:" << abs(coord_Aruco.transform.translation.y - coord_cam_robot.transform.translation.y) << std::endl;
            ros::spinOnce();

            error_y = (coord_Aruco.transform.translation.y - coord_cam_robot.transform.translation.y);
            error_x = (coord_Aruco.transform.translation.x - coord_cam_robot.transform.translation.x);
            //error_x = abs(coord_cam_robot.transform.translation.x - coord_Aruco.transform.translation.x);
            
            //error_x = (error_x > 0.25) ? 0.5 : error_x;
            //error_y = (error_y > 0.25) ? 0.5 : error_y;

            FestinoNavigation::move_base(error_x, error_y, 0.0, 0.001);
            ros::spinOnce();

            if ( error_x < 0.18 && abs(error_y) < 0.1)
            {
                FestinoNavigation::move_base(0.0, 0.0, 0.0, 0.01);
                aux = 0;
            }
            
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(0.1).sleep();
            continue;
        }   
    }while(aux);
    std::cout<<"End"<<std::endl;
    rate.sleep();
    return 0;
}