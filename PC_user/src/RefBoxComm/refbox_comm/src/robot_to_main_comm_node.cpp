#include <ros/ros.h>
#include "std_msgs/String.h"

#include<arpa/inet.h> 

#define TCPPORT 9002
#define SERVER_IP "192.168.1.100"
//#define SERVER_IP "192.168.1.106"
//#define SERVER_IP "192.168.1.163"

using namespace std;

ros::Publisher pub_instruction;
int client_fd;

char buffer[50];

void request_next_instruction(const std_msgs::String::ConstPtr& msg_to_server) {
printf("request received \n");
   //ROS_INFO("I heard: [%s]", msg->data.c_str());

    /*
        string msg_to_server;
        std::ostringstream oss;
        std::cout << "write a message to the server";
        cin >> msg_to_server;
        oss << msg_to_server;
        write(client_fd, oss.str().c_str(), oss.str().size());
    */

        int valread = 0;

        //std::ostringstream instruction;

        //instruction << "next" << '\0';

        //write(client_fd, instruction.str(), sizeof(instruction));//ask server for next instruction
        //std::cout << " ask next instruction " << std::endl;
        write(client_fd, "n", 1);
        valread = read(client_fd, buffer, sizeof(buffer));

        printf("Server instruction: %s\n", buffer);

        std::stringstream ss;
        ss << buffer;

        std_msgs::String msg_to_robot;
        msg_to_robot.data = ss.str();

        pub_instruction.publish(msg_to_robot);

        memset( &buffer, 0, sizeof(buffer));

}

//callback next_instruction
/*
    {//recv msg
        write //ask server for next instruction
        wait_for_server_response //thread????
        publish instruction to ros
    }

 */

int main(int argc, char** argv) {

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
printf("Connected \n");
    ros::init(argc, argv, "robot_to_main_comm_node");
    ros::NodeHandle n;

    //CREATE SUBSCRIBERS AND PUBLISHERS TO COMUNICATE WITH ROBOTS

    ros::Subscriber sub = n.subscribe("/request_instruction", 1000, request_next_instruction);
    pub_instruction = n.advertise<std_msgs::String>("/instruction_msg", 1000);

    ros::Rate r(10);
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }

    // closing the connected socket
    close(client_fd);
    return 0;
}
