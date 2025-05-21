#include <stdlib.h>
#include <iostream>
#include "ros/ros.h"

#include "festino_tools/FestinoHRI.h"
#include "festino_tools/FestinoNavigation.h"
#include "festino_tools/FestinoKnowledge.h"

#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseStamped.h"

enum SMState
{
    SM_INIT,
    SM_WAIT_FOR_DOOR,
    SM_NAVIGATE_TO_INSPECTION,
    SM_WAITING_FOR_KITCHEN,
    SM_WAIT_FOR_COMMAND,
    SM_REPEAT_COMMAND,
    SM_PARSE_SPOKEN_COMMAND,
    SM_FINAL_STATE,
    SM_WAIT_FOR_CONFIRMATION,
    SM_PARSE_SPOKEN_CONFIRMATION,
    SM_WAIT_FOR_INSPECTION,
    SM_NAVIGATE_TO_EXIT
};

SMState state = SM_INIT;
std::vector<float> goal_vec(3);
std::string location = "nuevito";
std::string recog = " ";
bool flag_door = true;


int main(int argc, char** argv)
{
    std::cout << "INITIALIZING ACT_PLN BY PAREJITASOFT Inc..." << std::endl; //cout
    ros::init(argc, argv, "act_pln");
    ros::NodeHandle n;

    FestinoHRI::setNodeHandle(&n);
    FestinoNavigation::setNodeHandle(&n);
    FestinoKnowledge::setNodeHandle(&n);
    ros::Rate loop(10);

    //Flags
    bool fail = false;
    bool success = false;

    FestinoHRI::say(" ",1);

    while(ros::ok() && !fail && !success)
    {
        switch(state)
        {
            case SM_INIT:
                std::cout << "State machine: SM_INIT" << std::endl;
                
                FestinoHRI::say("I am ready for robot inspection",3);
                state = SM_NAVIGATE_TO_INSPECTION;
                break;

            /*case SM_WAIT_FOR_DOOR:
                std::cout << "State machine: SM_WAIT_FOR_DOOR" << std::endl;
                arr_values.values = {0,0,0,0,1,1};
                pub_digital.publish(arr_values);
                ros::Duration(0.5, 0).sleep();
                arr_values.values = {0,0,0,0,0,0};
                pub_digital.publish(arr_values);
                ros::Duration(0.5, 0).sleep();
                arr_values.values = {0,0,0,0,1,1};
                pub_digital.publish(arr_values);
                FestinoHRI::say("I am waiting for the door to be open",3);
                //flag_door = false;
                if(!flag_door)
                    state = SM_WAIT_FOR_DOOR;
                else
                    state = SM_NAVIGATE_TO_INSPECTION;
                break;*/

            case SM_NAVIGATE_TO_INSPECTION:
                std::cout << "State machine: SM_NAVIGATE_TO_INSPECTION" << std::endl;
                FestinoHRI::say("I can see that the door is open, I am going to inspection point",3);
                sleep(3);
                
                goal_vec = FestinoKnowledge::CoordenatesLocSrv("Test-start");
                std::cout <<"Coordenates of Test-start:"<<std::endl;
                std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;

                if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2], 120000))
                    std::cout << "Cannot move to inspection point" << std::endl; 

                FestinoHRI::say("I have arrived to inspection point",1);    
                sleep(2);
                FestinoHRI::say("Please, tell me continue to the door",3);
                sleep(2);

                state=SM_WAIT_FOR_COMMAND;
                break;

            case SM_WAIT_FOR_COMMAND:                
                std::cout << "State machine: SM_WAIT_FOR_COMMAND" << std::endl;
                recog = FestinoHRI::lastRecogSpeech("test.json");
                if(recog != "continue")
                {
                    std::cout << "Listen: " << recog << std::endl;
                    state = SM_WAIT_FOR_COMMAND;
                }
                else
                {
                    std::cout << "I heard: " << recog << std::endl;
                    state = SM_NAVIGATE_TO_EXIT;
                }
                break;
            
            case SM_NAVIGATE_TO_EXIT:
                std::cout << "State machine: SM_NAVIGATE_TO_EXIT" << std::endl;
                
                FestinoHRI::say("I am going to the exit point",3);
                sleep(0.5);

                goal_vec = FestinoKnowledge::CoordenatesLocSrv("Test");
                std::cout <<"Coordenates of exit:"<<std::endl;
                std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
               
                if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2], 180000))
                // if(!FestinoNavigation::getClose(-3.0, 4.0, goal_vec[2], 180000))
                {
                    FestinoHRI::say("Cannot move to exit point",3);
                    state = SM_NAVIGATE_TO_EXIT;
                }

                //FestinoNavigation::moveDist(0.75, 5000);
                state = SM_FINAL_STATE;
                break;

            case SM_FINAL_STATE:
                ros::Duration(1, 0).sleep();

                std::cout << "State machine: SM_FINAL_STATE" << std::endl;
                FestinoHRI::say("i have finish the test",3);    
                ros::Duration(1, 0).sleep();
                success = true;
                break;
        }
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}