 #include "ros/ros.h"
#include "robot_to_main_communication/InstructionService.h"
#include "std_msgs/String.h"

#include <vector>
#include <string>
#include <boost/algorithm/string.hpp>

// Variables globales para almacenamiento de zonas
std::vector<std::string> zone_queue;
size_t current_zone_index = 0;

// Callback para recibir las zonas desde /pub_zone
void zoneCallback(const std_msgs::String::ConstPtr& msg)
{
    zone_queue.clear();
    current_zone_index = 0;

    boost::split(zone_queue, msg->data, boost::is_any_of(" "), boost::token_compress_on);
    ROS_INFO("Zonas recibidas (%lu):", zone_queue.size());
    for (const auto& z : zone_queue) {
        ROS_INFO(" - %s", z.c_str());
    }
}

// Servicio que devuelve una zona por cada solicitud "true"
bool handle_instruction(robot_to_main_communication::InstructionService::Request &req,
robot_to_main_communication::InstructionService::Response &res)
{
    if (req.request == "true") {
        if (current_zone_index < zone_queue.size()) {
            res.instruction = zone_queue[current_zone_index++];
            ROS_INFO("Enviando zona: %s", res.instruction.c_str());
        } else {
            res.instruction = "DONE"; // Ya no hay más zonas
            ROS_INFO("Todas las zonas ya fueron enviadas.");
        }
        return true;
    }

    res.instruction = "INVALID_REQUEST"; // Solicitud no válida
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_to_main_communication_node");
    ros::NodeHandle nh;

    ros::Subscriber zone_sub = nh.subscribe("/pub_zone", 10, zoneCallback);
    ros::ServiceServer service = nh.advertiseService("/instruction_msg", handle_instruction);

    ROS_INFO("Servicio /instruction_msg listo");
    ros::spin();

    return 0;
}