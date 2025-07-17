#include "ros/ros.h"
#include "robot_to_main_communication/InstructionService.h"

#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>

#define TCPPORT 9002	
#define SERVER_IP "192.168.100.111"

int client_fd;
char buffer[50];

bool handle_instruction(robot_to_main_communication::InstructionService::Request &req,
                        robot_to_main_communication::InstructionService::Response &res)
{
    std::string command = req.request;
    if (command.empty())
        command = "request_instruction";

    // Enviar al servidor TCP
    write(client_fd, command.c_str(), command.length());

    // Leer respuesta del servidor
    int valread = read(client_fd, buffer, sizeof(buffer));
    if (valread > 0) {
        buffer[valread] = '\0';
        res.instruction = std::string(buffer);
        memset(buffer, 0, sizeof(buffer));
        ROS_INFO("Response: %s", res.instruction.c_str());
        return true;
    } else {
        ROS_ERROR("No data received from server");
        return false;
    }
}


int main(int argc, char **argv)
{
    struct sockaddr_in serv_addr;

    if ((client_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        ROS_ERROR("Socket creation error");
        return -1;
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(TCPPORT);

    if (inet_pton(AF_INET, SERVER_IP, &serv_addr.sin_addr) <= 0) {
        ROS_ERROR("Invalid address/ Address not supported");
        return -1;
    }

    if (connect(client_fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        ROS_ERROR("Connection Failed");
        return -1;
    }

    ros::init(argc, argv, "robot_to_main_communication_node");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("/instruction_msg", handle_instruction);
    ROS_INFO("Instruction service ready");

    ros::spin();
    close(client_fd);
    return 0;
}
