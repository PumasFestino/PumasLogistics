#include "ros/ros.h"
#include "robot_to_main_communication/InstructionService.h"
#include "robot_to_main_communication/ZoneService.h"
#include "std_msgs/String.h"

#include <vector>
#include <string>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp> 

#include<arpa/inet.h> 

#define TCPPORT 9002
#define SERVER_IP "192.168.0.101"

int client_fd;
char buffer[50];

// Variables globales para almacenamiento de zonas
std::vector<std::string> zone_queue;
std::vector<std::string> tokens;
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

// Servicio que devuelve una instruccion por cada solicitud
bool handle_instruction(robot_to_main_communication::InstructionService::Request &req,
robot_to_main_communication::InstructionService::Response &res)
{
    int valread = 0;
    std::stringstream ss;

    write(client_fd, "n", 1);
    valread = read(client_fd, buffer, sizeof(buffer));

    ss << buffer;
    res.instruction = ss.str();

    memset(&buffer, 0, sizeof(buffer));         // Limpia el buffer
    return true;
}

// Servicio que devuelve una zona por cada solicitud "true"
bool handle_zone(robot_to_main_communication::ZoneService::Request &req,
robot_to_main_communication::ZoneService::Response &res)
{
    if (req.request == "true") {
        if (current_zone_index < zone_queue.size()) {
            res.zone = zone_queue[current_zone_index++];
            ROS_INFO("Enviando zona: %s", res.zone.c_str());
        } else {
            res.zone = "DONE"; // Ya no hay más zonas
            ROS_INFO("Todas las zonas ya fueron enviadas.");
        }
        return true;
    } else {
        res.zone = "INVALID_REQUEST"; // Solicitud no válida
        return false;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_to_main_communication_node");
    ros::NodeHandle nh;

    ros::Subscriber zone_sub = nh.subscribe("/pub_zone", 10, zoneCallback);
    ros::ServiceServer instruction_srv = nh.advertiseService("/instruction_msg", handle_instruction);
    ros::ServiceServer zone_srv = nh.advertiseService("/zone_msg", handle_zone);

    ROS_INFO("Servicio /instruction_msg listo");
    ROS_INFO("Servicio /zone_msg listo");
    ros::spin();

    return 0;
}