#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/TransformStamped.h>
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

bool isPointInRotatedRectangle(float px, float py, float cx, float cy, float width, float length, float theta_rad)
{
    // Translate point to rectangle coordinate system
    float dx = px - cx;
    float dy = py - cy;
    
    // Rotate point back to align with rectangle's axes
    float cos_theta = cos(-theta_rad);
    float sin_theta = sin(-theta_rad);
    float rotated_x = dx * cos_theta - dy * sin_theta;
    float rotated_y = dx * sin_theta + dy * cos_theta;
    
    // Check if point is within the non-rotated rectangle
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

    //For orientation to the station
    float angle_in = atan2(cy - in_y, cx - in_x);
    float angle_out = atan2(cy - out_y, cx - out_x);

    publishTransform("map", req.zone_frame + "_IN", in_x, in_y, angle_in);
    publishTransform("map", req.zone_frame + "_OUT", out_x, out_y, angle_out);

    res.success = true;
    res.message = "Station printed and TFs '_IN' and '_OUT' published successfully.";

    // Kill AMCL
    int kill_result = system("rosnode kill /amcl");
    if (kill_result != 0)
    {
        ROS_WARN("modify_map (service) --- Could not kill /amcl or it was already dead.");
    }

    // Wait
    ros::Duration(1.0).sleep();

    // Reload AMCL with the same parameters
    int launch_result = system("roslaunch config_files amcl_reload.launch");
    if (launch_result != 0)
    {
        ROS_ERROR("modify_map (service) --- Failed to relaunch AMCL.");
        res.success = false;
        res.message = "Map modified but could not reload AMCL.";
        return true;
    }

    return true;
}

int main(int argc, char** argv) {
    std::cout << "modify_map (service) --- Soft by Joshua M" << std::endl;
    ros::init(argc, argv, "modify_map_server");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("modify_map", modifyMapCallback);
    map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);

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