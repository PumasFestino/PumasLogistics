#include "festino_tools/FestinoCommunication.h"

bool FestinoCommunication::is_node_set = false;

ros::ServiceClient FestinoCommunication::instruction_client; 

//Aquí se configuran los nodos, el tipo de mensaje, buffer, el topico, etc.
bool FestinoCommunication::setNodeHandle(ros::NodeHandle* nh)
{
    if(FestinoCommunication::is_node_set)
        return true;
    if(nh == 0)
        return false;

    std::cout << "FestinoCommunication.->Setting ros node..." << std::endl;

    instruction_client = nh->serviceClient<robot_to_main_communication::InstructionService>("/instruction_msg");
    FestinoCommunication::is_node_set = true;
    return true;
}

bool FestinoCommunication::getInstruction(std::vector<std::string>* tokens)
{
    robot_to_main_communication::InstructionService srv;
    srv.request.request = "request_instruction";
    std::vector<std::string> Tokens = *tokens;

    if(instruction_client.call(srv)){
        Tokens.clear();
        boost::algorithm::split(Tokens, srv.response.instruction, boost::algorithm::is_any_of(" "));
        return true;
    }else{
        ROS_ERROR("FestinoCommunication.->Failed to call /instruction_msg");
        return false;
    }
}

bool FestinoCommunication::reportPose(float x, float y)
{
    robot_to_main_communication::InstructionService srv;
    std::ostringstream oss;
    oss << "report_pose:" << x << "," << y;
    srv.request.request = oss.str();

    if (instruction_client.call(srv)) {
        return srv.response.instruction == "ACK";
    } else {
        ROS_ERROR("FestinoCommunication.->Failed to report pose");
        return false;
    }
}

bool FestinoCommunication::reportMachine(const std::string& zone, const std::string& orientation)
{
    robot_to_main_communication::InstructionService srv;
    srv.request.request = "report_machine:" + zone + "," + orientation;

    if (instruction_client.call(srv)) {
        return srv.response.instruction == "ACK";
    } else {
        ROS_ERROR("FestinoCommunication.->Failed to report machine info");
        return false;
    }
}
