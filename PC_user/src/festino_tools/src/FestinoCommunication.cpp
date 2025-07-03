#include "festino_tools/FestinoCommunication.h"

bool FestinoHardware::is_node_set = false;

ros::ServiceClient FestinoCommunication::instruction_client; 

//Aquí se configuran los nodos, el tipo de mensaje, buffer, el topico, etc.
bool FestinoCommunication::setNodeHandle(ros::NodeHandle* nh)
{
    if(FestinoHardware::is_node_set)
        return true;
    if(nh == 0)
        return false;

    std::cout << "FestinoCommunication.->Setting ros node..." << std::endl;

    instruction_client = nh->serviceClient<robot_to_main_communication::InstructionService>("/instruction_msg");
    FestinoCommunication::is_node_set = true;
    return true;
}

std::string FestinoCommunication::getInstruction()
{
    robot_to_main_communication::InstructionService srv;
    if (instruction_client.call(srv)) {
        return srv.response.instruction;
    } else {
        ROS_ERROR("FestinoCommunication.->Failed to call /instruction_msg");
        return "";
    }
}