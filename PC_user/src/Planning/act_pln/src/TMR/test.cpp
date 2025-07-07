#include <stdlib.h>
#include <iostream>
#include "ros/ros.h"

#include "festino_tools/FestinoHRI.h"
#include "festino_tools/FestinoNavigation.h"
#include "festino_tools/FestinoKnowledge.h"

//#include "justina_tools/JustinaVision.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseStamped.h"


#define GRAMMAR_POCKET_NAMES "grammars/people_names.jsgf"

std::string grammarCommandsID = "receptionistNames";

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING ACT_PLN BY PAREJITASOFT Inc..." << std::endl; //cout
    ros::init(argc, argv, "act_pln");
    ros::NodeHandle n;

    FestinoHRI::setNodeHandle(&n);
    FestinoNavigation::setNodeHandle(&n);
    FestinoKnowledge::setNodeHandle(&n);
    //FestinoVision::setNodeHandle(&n);
    ros::Rate loop(10);
    std::string grammarNamesID = "receptionistNames";

    FestinoHRI::loadGrammarSpeechRecognized(grammarCommandsID, GRAMMAR_POCKET_NAMES);
    sleep(2);
    while(1)
    {
    	FestinoHRI::enableSpeechRecognized(true);
    	sleep(2);
    	std::cout << "Name: " << FestinoHRI::lastRecogSpeech()<<std::endl;
    }

    ros::spinOnce();
    loop.sleep();
    return 1;
}