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

// Instrucciones para probar
std::vector<std::string> demo_instructions = {
    // FIRST MACHINE
    "move CS M_Z45 0 output",
    "retrieve CS M_Z45 0 output",
    "move CS M_Z45 0 input",
    "deliver CS M_Z45 0 input",
    "move CS M_Z45 0 output",
    "retrieve CS M_Z45 0 output",
    "move CS M_Z45 0 input",
    "deliver CS M_Z45 0 input",
    "move CS M_Z45 0 output",
    "retrieve CS M_Z45 0 output",
    "move CS M_Z45 0 input",
    "deliver CS M_Z45 0 input",
    // SECOND MACHINE
    "move CS M_Z53 0 output",
    "retrieve CS M_Z53 0 output",
    "move CS M_Z53 0 input",
    "deliver CS M_Z53 0 input",
    "move CS M_Z53 0 output",
    "retrieve CS M_Z53 0 output",
    "move CS M_Z53 0 input",
    "deliver CS M_Z53 0 input",
    "move CS M_Z53 0 output",
    "retrieve CS M_Z53 0 output",
    "move CS M_Z53 0 input",
    "deliver CS M_Z53 0 input",
    // THIRD MACHINE
    "move CS M_Z12 0 output",
    "retrieve CS M_Z12 0 output",
    "move CS M_Z12 0 input",
    "deliver CS M_Z12 0 input",
    "move CS M_Z12 0 output",
    "retrieve CS M_Z12 0 output",
    "move CS M_Z12 0 input",
    "deliver CS M_Z12 0 input",
    "move CS M_Z12 0 output",
    "retrieve CS M_Z12 0 output",
    "move CS M_Z12 0 input",
    "deliver CS M_Z12 0 input",
    // INITIAL POSITION
    "move CS M_Z55 0"
};

std::string demo_zones = "Z45 0 Z53 90 Z12 90";

int instruction_index = 0;

size_t current_zone_index = 20;

// Callback para recibir las zonas desde /zone_msg
void zoneCallback(const std_msgs::String::ConstPtr& msg)
{
    int ret;
    zone_queue.clear();
    current_zone_index = 0;

    boost::split(zone_queue, msg->data, boost::is_any_of(" "), boost::token_compress_on);
    ROS_INFO("Zonas recibidas (%lu):", zone_queue.size());
    for (const auto& z : zone_queue) {
        ROS_INFO(" - %s", z.c_str());
    }

    //ret = system("rosrun act_pln main_track &");

    if (ret == 0) {
        ROS_INFO("Main track launched successfully.");
    } else {
        ROS_ERROR("Failed to launch main_track.");
    }
}

// Servicio que devuelve una instruccion por cada solicitud
bool handle_instruction(robot_to_main_communication::InstructionService::Request &req,
robot_to_main_communication::InstructionService::Response &res)
{
    /*int valread = 0;
    std::string command = req.request;

    std::stringstream ss;

    if (command == "z" || command == "n")
    {
        write(client_fd, command.c_str(), command.length());
        valread = read(client_fd, buffer, sizeof(buffer));

        ss << buffer;

        res.instruction = ss.str();

        memset(&buffer, 0, sizeof(buffer));         // Limpia el buffer
    }
    else
    {
        std::cout << "INVALID COMMAND: " << command << std::endl;
    }

    return true;
    */

    /*****************  TESTING / GRASPING CHALLENGE **************************/
    std::string command = req.request;

    if (command == "n" && instruction_index < demo_instructions.size()){
        res.instruction = demo_instructions[instruction_index];
        ++instruction_index;
    }
    else if(command == "z"){
        res.instruction = demo_zones;
    }

    return true;
    /**************************************************************************/
}

// Servicio que devuelve una zona por cada solicitud "true"
bool handle_zone(robot_to_main_communication::ZoneService::Request &req,
robot_to_main_communication::ZoneService::Response &res)
{ 
    if (req.request == "true") {
        if (current_zone_index < zone_queue.size()) {
            res.zone = "move CS " + zone_queue[current_zone_index++] + " 0 " + "";
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
/*
    int status;
    struct sockaddr_in serv_addr;
    if ((client_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        printf("\n Socket creation error \n");
        return -1;
    }
 
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(TCPPORT);
 
    // Convert IPv4 and IPv6 addresses from text to binary
    // form
    if (inet_pton(AF_INET, SERVER_IP, &serv_addr.sin_addr)
        <= 0) {
        printf(
            "\nInvalid address/ Address not supported \n");
        return -1;
    }
 
    if ((status
         = connect(client_fd, (struct sockaddr*)&serv_addr,
                   sizeof(serv_addr)))
        < 0) {
        printf("\nConnection Failed \n");
        return -1;
    }
    printf("Connected \n");*/
    ros::init(argc, argv, "robot_to_main_communication_node");
    ros::NodeHandle nh;

    //ros::Subscriber zone_sub = nh.subscribe("/zone_msg", 10, zoneCallback);
    ros::ServiceServer instruction_srv = nh.advertiseService("/instruction_msg", handle_instruction);
    ros::ServiceServer zone_srv = nh.advertiseService("/nav_zones", handle_zone);

    ROS_INFO("Servicio /instruction_msg listo");
    ROS_INFO("Servicio /nav_zones listo");
    zone_queue.clear();
    ros::spin();


    return 0;
}