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
#include <fstream>
#include <ros/package.h>

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

void paintStationAndTFs(tf2_ros::Buffer& tf_buffer, const std::string& zone_tf, double theta_rad)
{
    geometry_msgs::TransformStamped tf_zone;
    try {
        tf_zone = tf_buffer.lookupTransform("map", zone_tf, ros::Time(0), ros::Duration(2.0));
    } catch(tf2::TransformException &ex) {
        ROS_WARN("modify_map --- TF error con %s: %s", zone_tf.c_str(), ex.what());
        return;
    }

    float cx = tf_zone.transform.translation.x;
    float cy = tf_zone.transform.translation.y;

    float width = 0.3;
    float length = 0.7;
    float resolution = modified_map.info.resolution;
    int map_width = modified_map.info.width;
    int map_height = modified_map.info.height;
    float origin_x = modified_map.info.origin.position.x;
    float origin_y = modified_map.info.origin.position.y;

    for (int y = 0; y < map_height; ++y)
    {
        for (int x = 0; x < map_width; ++x)
        {
            float world_x = origin_x + (x + 0.5f) * resolution;
            float world_y = origin_y + (y + 0.5f) * resolution;
            if (isPointInRotatedRectangle(world_x, world_y, cx, cy, width, length, theta_rad))
            {
                modified_map.data[y * map_width + x] = 100;
            }
        }
    }

    float offset = 0.750;
    float in_x = cx + offset * cos(theta_rad);
    float in_y = cy + offset * sin(theta_rad);
    float out_x = cx - offset * cos(theta_rad);
    float out_y = cy - offset * sin(theta_rad);
    float angle_in = atan2(cy - in_y, cx - in_x);
    float angle_out = atan2(cy - out_y, cx - out_x);

    publishTransform("map", zone_tf + "_input", in_x, in_y, angle_in);
    publishTransform("map", zone_tf + "_output", out_x, out_y, angle_out);
}

void saveModifiedMap(const nav_msgs::OccupancyGrid& map, const std::string& name)
{
    int width = map.info.width;
    int height = map.info.height;
    float resolution = map.info.resolution;

    std::ofstream pgm_file(name + ".pgm", std::ios::binary);
    pgm_file << "P5\n" << width << " " << height << "\n255\n";
    for (int y = height - 1; y >= 0; --y)
    {
        for (int x = 0; x < width; ++x)
        {
            int8_t val = map.data[y * width + x];
            uint8_t out;
            if (val == -1) out = 205;
            else if (val == 0) out = 254;
            else out = 0;
            pgm_file.write(reinterpret_cast<char*>(&out), 1);
        }
    }
    pgm_file.close();

    std::ofstream yaml_file(name + ".yaml");
    yaml_file << "image: " << name << ".pgm\n";
    yaml_file << "resolution: " << resolution << "\n";
    yaml_file << "origin: [" << map.info.origin.position.x << ", " << map.info.origin.position.y << ", 0.0]\n";
    yaml_file << "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
    yaml_file.close();

    ROS_INFO("Mapa guardado: %s.[pgm|yaml]", name.c_str());
}

bool staticMapCallback(nav_msgs::GetMap::Request& req, nav_msgs::GetMap::Response& res)
{
    res.map = modified_map;
    return true;
}

bool modifyMapCallback(movement_functions::ModifyMap::Request &req, movement_functions::ModifyMap::Response &res)
{
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    ros::Duration(1.0).sleep();

    // Convertir orientación y preparar nombres de TFs
int rounded_angle = static_cast<int>(std::round(req.orientation / 45.0)) * 45;
double theta_rad_c = rounded_angle * M_PI / 180.0;

double theta_rad_m;
if (rounded_angle % 90 != 0) {
    int mirrored_angle = (180 - rounded_angle + 360) % 360;
    theta_rad_m = mirrored_angle * M_PI / 180.0;
} else {
    theta_rad_m = theta_rad_c;
}

std::string tf_c = "C_" + req.zone_frame;
std::string tf_m = "M_" + req.zone_frame;

// Pintar ambas estaciones
paintStationAndTFs(tf_buffer, tf_c, theta_rad_c);
paintStationAndTFs(tf_buffer, tf_m, theta_rad_m);


    modified_map.header.stamp = ros::Time::now();
    map_pub.publish(modified_map);


    std::string package_path = ros::package::getPath("config_files");
    std::string full_path = package_path + "/maps/logistics-2025-mod";
    saveModifiedMap(modified_map, full_path);


    res.success = true;
    res.message = "Estaciones pintadas y mapa guardado correctamente.";
    return true;
}

int main(int argc, char** argv)
{
    std::cout << "modify_map (service) --- Soft by Joshua M" << std::endl;
    ros::init(argc, argv, "modify_map_server");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("modify_map", modifyMapCallback);
    map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
    ros::ServiceServer static_map_srv = nh.advertiseService("/static_map", staticMapCallback);
    nav_msgs::OccupancyGrid::ConstPtr base_map = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map", nh);
    if (!base_map)
    {
        ROS_ERROR("No se pudo obtener el mapa base del tópico '/map'.");
        return 1;
    }
    modified_map = *base_map;

    ROS_INFO("modify_map (service) --- 'modify_map' listo.");
    ros::spin();
    return 0;
}
