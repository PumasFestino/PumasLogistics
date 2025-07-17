#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <movement_functions/ModifyMap.h>
#include <cmath>
#include <string>
#include <vector>
#include <signal.h>
#include <cstdlib>

ros::Publisher map_pub;
nav_msgs::OccupancyGrid modified_map;

void publishTransform(const std::string& parent_frame, const std::string& child_frame, float x, float y, float theta_rad)
{
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = parent_frame;
    transformStamped.child_frame_id = child_frame;
    transformStamped.transform.translation.x = x;
    transformStamped.transform.translation.y = y;
    transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_rad);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    static_broadcaster.sendTransform(transformStamped);
}

bool staticMapCallback(nav_msgs::GetMap::Request& req, nav_msgs::GetMap::Response& res)
{
    res.map = modified_map;
    return true;
}

bool isPointInRotatedRectangle(float px, float py, float cx, float cy, float width, float length, float theta_rad)
{
    float dx = px - cx;
    float dy = py - cy;
    float cos_theta = cos(-theta_rad);
    float sin_theta = sin(-theta_rad);
    float rotated_x = dx * cos_theta - dy * sin_theta;
    float rotated_y = dx * sin_theta + dy * cos_theta;
    return (rotated_x >= -width/2 && rotated_x <= width/2 &&
            rotated_y >= -length/2 && rotated_y <= length/2);
}

bool modifyMapCallback(movement_functions::ModifyMap::Request &req, movement_functions::ModifyMap::Response &res)
{
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    ros::Duration(1.0).sleep();

    geometry_msgs::TransformStamped tf_zone;
    try
    {
        tf_zone = tf_buffer.lookupTransform("map", req.zone_frame, ros::Time(0), ros::Duration(2.0));
    }
    catch(tf2::TransformException &ex)
    {
        ROS_ERROR("modify_map (service) --- TF error: %s", ex.what());
        res.success = false;
        res.message = "Failed to obtain TF for the specified zone.";
        return true;
    }

    double width = 0.3;
    double length = 0.7;
    int rounded_angle = static_cast<int>(std::round(req.orientation / 45.0)) * 45;
    double theta_rad = rounded_angle * M_PI / 180.0;
    float cx = tf_zone.transform.translation.x;
    float cy = tf_zone.transform.translation.y;

    float resolution = modified_map.info.resolution;
    int map_width = modified_map.info.width;
    int map_height = modified_map.info.height;
    float origin_x = modified_map.info.origin.position.x;
    float origin_y = modified_map.info.origin.position.y;

    for (int y = 0; y < map_height; ++y)
    {
        for (int x = 0; x < map_width; ++x)
        {
            float world_x = origin_x + (x + 0.5) * resolution;
            float world_y = origin_y + (y + 0.5) * resolution;
            if (isPointInRotatedRectangle(world_x, world_y, cx, cy, width, length, theta_rad))
            {
                modified_map.data[y * map_width + x] = 100;
            }
        }
    }

    modified_map.header.stamp = ros::Time::now();
    map_pub.publish(modified_map);

    float offset = 0.750;
    float in_x = cx + offset * cos(theta_rad);
    float in_y = cy + offset * sin(theta_rad);
    float out_x = cx - offset * cos(theta_rad);
    float out_y = cy - offset * sin(theta_rad);
    float angle_in = atan2(cy - in_y, cx - in_x);
    float angle_out = atan2(cy - out_y, cx - out_x);

    publishTransform("map", req.zone_frame + "_input", in_x, in_y, angle_in);
    publishTransform("map", req.zone_frame + "_output", out_x, out_y, angle_out);

    res.success = true;
    res.message = "Station printed and TFs '_IN' and '_OUT' published successfully.";

    // Obtener posición actual del robot en el mapa
    geometry_msgs::TransformStamped tf_robot;
    try {
        tf_robot = tf_buffer.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(2.0));
    } catch(tf2::TransformException &ex) {
        ROS_ERROR("modify_map (service) --- TF error al obtener base_link: %s", ex.what());
        res.success = false;
        res.message = "No se pudo obtener la posición actual del robot.";
        return true;
    }

    // Publicar en /initialpose
    ros::NodeHandle nh;
    ros::Publisher initial_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, true);
    geometry_msgs::PoseWithCovarianceStamped init_pose;
    init_pose.header.stamp = ros::Time::now();
    init_pose.header.frame_id = "map";
    init_pose.pose.pose.position.x = tf_robot.transform.translation.x;
    init_pose.pose.pose.position.y = tf_robot.transform.translation.y;
    init_pose.pose.pose.orientation = tf_robot.transform.rotation;

    for (int i = 0; i < 36; ++i)
        init_pose.pose.covariance[i] = 0.0;
    init_pose.pose.covariance[0] = 0.5 * 0.5;
    init_pose.pose.covariance[7] = 0.5 * 0.5;
    init_pose.pose.covariance[35] = (M_PI / 12.0) * (M_PI / 12.0); // 15°

    initial_pose_pub.publish(init_pose);
    ros::Duration(1.0).sleep();

        // Reiniciar AMCL
    int kill_result = system("rosnode kill /amcl");
    if (kill_result != 0)
    {
        ROS_WARN("modify_map (service) --- Could not kill /amcl or it was already dead.");
    }

    ros::Duration(1.0).sleep();

    int launch_result = system("roslaunch config_files amcl_reload.launch &");
    if (launch_result != 0)
    {
        ROS_ERROR("modify_map (service) --- Failed to relaunch AMCL.");
        res.success = false;
        res.message = "Map modified but could not reload AMCL.";
        return true;
    }

    // Esperar a que /amcl esté activo (máx 10 intentos)
    ROS_INFO("modify_map (service) --- Esperando a que /amcl esté activo...");
    bool amcl_up = false;
    for (int i = 0; i < 10; ++i)
    {
        if (system("rosnode list | grep -w /amcl > /dev/null") == 0)
        {
            amcl_up = true;
            break;
        }
        ros::Duration(1.0).sleep();
    }

    if (!amcl_up)
    {
        ROS_WARN("modify_map (service) --- /amcl no se levantó a tiempo.");
        res.success = false;
        res.message = "AMCL no se levantó a tiempo para recibir /initialpose.";
        return true;
    }

    ROS_INFO("modify_map (service) --- /amcl detectado, publicando /initialpose...");

    // Obtener posición actual del robot en el mapa (ya estaba definida antes)
    try {
        tf_robot = tf_buffer.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(2.0));
    } catch(tf2::TransformException &ex) {
        ROS_ERROR("modify_map (service) --- TF error al obtener base_link: %s", ex.what());
        res.success = false;
        res.message = "No se pudo obtener la posición actual del robot.";
        return true;
    }

    init_pose.header.stamp = ros::Time::now();
    init_pose.header.frame_id = "map";
    init_pose.pose.pose.position.x = tf_robot.transform.translation.x;
    init_pose.pose.pose.position.y = tf_robot.transform.translation.y;
    init_pose.pose.pose.orientation = tf_robot.transform.rotation;

    for (int i = 0; i < 36; ++i)
        init_pose.pose.covariance[i] = 0.0;
    init_pose.pose.covariance[0] = 0.5 * 0.5;
    init_pose.pose.covariance[7] = 0.5 * 0.5;
    init_pose.pose.covariance[35] = (M_PI / 12.0) * (M_PI / 12.0);

    ros::Duration(1.0).sleep();
    initial_pose_pub.publish(init_pose);
    ros::Duration(0.5).sleep();

    ROS_INFO("modify_map (service) --- /initialpose publicado con éxito.");
    return true;

}

int main(int argc, char** argv) {
    std::cout << "modify_map (service) --- Soft by Joshua M" << std::endl;
    ros::init(argc, argv, "modify_map_server");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("modify_map", modifyMapCallback);
    map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
    ros::ServiceServer static_map_srv = nh.advertiseService("/static_map", staticMapCallback);
    nav_msgs::OccupancyGrid::ConstPtr base_map = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map", nh);
    if (!base_map)
    {
        ROS_ERROR("Failed to obtain the base map from topic '/map'.");
        return 1;
    }
    modified_map = *base_map;

    ROS_INFO("modify_map (service) --- 'modify_map' is ready.");
    ros::spin();
    return 0;
}
