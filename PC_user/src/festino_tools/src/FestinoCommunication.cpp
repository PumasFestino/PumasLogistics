#include "festino_tools/FestinoCommunication.h"

bool FestinoCommunication::is_node_set = false;

ros::ServiceClient FestinoCommunication::instruction_client; 
ros::ServiceClient FestinoCommunication::zone_client; 

//Aquí se configuran los nodos, el tipo de mensaje, buffer, el topico, etc.
bool FestinoCommunication::setNodeHandle(ros::NodeHandle* nh)
{
    if(FestinoCommunication::is_node_set)
        return true;
    if(nh == 0)
        return false;

    std::cout << "FestinoCommunication.->Setting ros node..." << std::endl;

    instruction_client = nh->serviceClient<robot_to_main_communication::InstructionService>("/instruction_msg");
    zone_client = nh->serviceClient<robot_to_main_communication::ZoneService>("/zone_msg");
    FestinoCommunication::is_node_set = true;
    return true;
}

bool FestinoCommunication::getInstruction(std::vector<std::string>* tokens, string command)
{
    robot_to_main_communication::InstructionService srv;
    srv.request.request = command;

    if(instruction_client.call(srv)){
        
        std::string instruction_str = srv.response.instruction;        

         // Elimina espacios al inicio y final
        boost::algorithm::trim(instruction_str);

        if (instruction_str.empty()) {
            return false;
        }

        std::vector<std::string> temp_tokens;

        if (command == "n"){ // "n" es para instrucciones, "z" es para zonas

            // Divide la cadena en tokens
            boost::algorithm::split(temp_tokens, srv.response.instruction, boost::is_any_of(" "), boost::token_compress_on);

            // Rellena o recorta a 5 elementos
            while (temp_tokens.size() < 5)
                temp_tokens.push_back("");
            
            if (temp_tokens.size() > 5)
                temp_tokens.resize(5);
        }

        else if (command == "z"){

            // Solo agrega la cadena completa como un único elemento
            temp_tokens.push_back(instruction_str);
        }

        *tokens = temp_tokens;
        return true;
    } else {
        ROS_ERROR("FestinoCommunication.->Failed to call /instruction_msg");
        return false;
    }
}

bool FestinoCommunication::getZone(std::vector<std::string>* tokens)
{
    robot_to_main_communication::ZoneService srv;
    srv.request.request = "true";

    if(zone_client.call(srv)){
        tokens->clear();
        boost::algorithm::split(*tokens, srv.response.zone, boost::is_any_of(" "), boost::token_compress_on);
        return true;
    } else {
        ROS_ERROR("FestinoCommunication.->Failed to call /zone_msg");
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
