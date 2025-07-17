#include "ros/ros.h"
#include "robot_to_main_communication/InstructionService.h"
#include "robot_to_main_communication/ZoneService.h"
#include <string>

// Tokenization libraries
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>

class FestinoCommunication
{
public:
    static bool setNodeHandle(ros::NodeHandle* nh);
    static bool getInstruction(std::vector<std::string>* tokens);
    static bool getZone(std::vector<std::string>* tokens);
    static bool reportPose(float x, float y);
    static bool reportMachine(const std::string& zone, const std::string& orientation);
private:
    static bool is_node_set;
    static ros::ServiceClient instruction_client;
    static ros::ServiceClient zone_client;
};