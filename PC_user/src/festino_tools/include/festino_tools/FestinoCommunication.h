#include "ros/ros.h"
#include "robot_to_main/InstructionService.h"
#include <string>

class FestinoCommunication
{
public:
    static bool setNodeHandle(ros::NodeHandle* nh);
    static std::string getInstruction();

private:
    static bool is_node_set;
    static ros::ServiceClient instruction_client;
};