//State Machine for main track
#include <iostream>
#include <cmath>
#include <math.h>
#include "ros/ros.h"
#include <vector> 
#include <string>
#include "std_msgs/String.h"
#include <std_msgs/Int32.h>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include <tf/transform_listener.h>
#include "robotino_msgs/DigitalReadings.h"
#include <sstream>
#include "ros/time.h"
#include "actionlib_msgs/GoalStatus.h"
#include <algorithm>

//Para encontrar aruco
#include "img_proc/Find_tag_Srv.h"

//Festino Tools
#include "festino_tools/FestinoHRI.h"
#include "festino_tools/FestinoVision.h"
#include "festino_tools/FestinoNavigation.h"
#include "festino_tools/FestinoKnowledge.h"
#include "festino_tools/FestinoCommunication.h"

#include "festino_tools/FestinoTask.h"

std::string instructions[] = {"move CS C_Z56 270 platform",
                              "move CS C_Z42 0 entrance",
                              "move CS C_Z42 0 output",
                              "move CS M_Z61 0 entrance"};

int cont_instructions = 0;

//Se puede cambiar, agregar o eliminar los estados
enum SMState {
    SM_INIT,
    SM_WAIT_FOR_ZONES,
	SM_WAIT_FOR_INSTRUCTION,
	SM_MOVE,
    SM_ALIGN,
    SM_RETRIEVE,
    SM_DELIVER,
    SM_FINAL_STATE
};

bool fail = false;
bool success = false;
SMState state = SM_INIT;

// Request flag for new instructions to the planner
bool request = false;

// String to storage instruction tokens
std::vector<std::string> instructionTokens;

// Receive instructions from the planner
void request_instruction(){
    std::cout << "Request a new instruction" << std::endl;	
    instructionTokens.clear();

    if (!FestinoCommunication::getZone(&instructionTokens)) {
        if(FestinoCommunication::getInstruction(&instructionTokens,"n")) {
            std::cout << "The instruction is: " <<  instructionTokens[0] << std::endl;	
            request = true;

            if(instructionTokens[0] == "move"){
                state = SM_MOVE;
                return;
            }else if(instructionTokens[0] == "retrieve"){
                state = SM_RETRIEVE;
                return;
            }else if(instructionTokens[0] == "deliver"){
                state = SM_DELIVER;
                return;
            }else{
                request = false;
                return;
            }
        }
    } else {
        std::cout << "The instruction is: " <<  instructionTokens[0] << std::endl;  
        request = true;

        if(instructionTokens[0] == "move"){
            state = SM_MOVE;
            return;
        } else {
            request = false;
            return;
        }
    }
}

float angulo_rad;
//Variable que guarda el angulo en formato entero (se convierte de string a entero)
int angulo_int = 0;
//TF que va guardando la zona objetivo
geometry_msgs::PoseStamped tf_target_zone;

//Funcion que obtiene las coordenadas de las zonas con el lookupTransform
void transform_zone(std::string zone)
{
    
	tf::TransformListener listener;
    tf::StampedTransform transform;

    //TF related stuff 
    std::cout << instructionTokens[2] << std::endl;
    tf_target_zone.header.frame_id = "/map";
    tf_target_zone.pose.position.x = 0.0;
    tf_target_zone.pose.position.y = 0.0;
    tf_target_zone.pose.position.z = 0.0;
    tf_target_zone.pose.orientation.x = 0.0;
    tf_target_zone.pose.orientation.y = 0.0;
    tf_target_zone.pose.orientation.z = 0.0;
    tf_target_zone.pose.orientation.w = 0.0;

    std::cout << "entró al transform zones" << std::endl;

    try{
        std::cout << "entró al try" << std::endl;
        listener.waitForTransform("/map", instructionTokens.at(2), ros::Time(0), ros::Duration(100.0));
        listener.lookupTransform("/map", instructionTokens.at(2), ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    tf_target_zone.pose.position.x = transform.getOrigin().x();
    tf_target_zone.pose.position.y = transform.getOrigin().y();
	tf_target_zone.pose.position.z = transform.getOrigin().z();
	tf_target_zone.pose.orientation.x = transform.getRotation().x();
	tf_target_zone.pose.orientation.y = transform.getRotation().y();
	tf_target_zone.pose.orientation.z = transform.getRotation().z();
	tf_target_zone.pose.orientation.w = transform.getRotation().w();

    std::cout << "salió del try name:" << instructionTokens.at(2) << " tf x:" << tf_target_zone.pose.position.x << " y:" << tf_target_zone.pose.position.y << std::endl;
}


void navigate_to_location(geometry_msgs::PoseStamped location)
{
    std::cout << "Navigate to location x:"<< location.pose.position.x << " y:" << location.pose.position.y << std::endl;
    if(!FestinoNavigation::getClose(location.pose.position.x, location.pose.position.y, location.pose.orientation.x,60000)){
        if(!FestinoNavigation::getClose(location.pose.position.x, location.pose.position.y, location.pose.orientation.x, 60000)){
         	std::cout << "Cannot move to " << std::endl;
                FestinoHRI::say("Just let me go. Cries in robot iiiiii",3);
        }
    }
}

sensor_msgs::LaserScan laserScan;
bool flag_wall = false;
float move_to_machine = 0.0f;



int main(int argc, char** argv){
	ros::Time::init();
	bool latch;
	std::cout << "INITIALIZING PLANNING NODE... " << std::endl;
    ros::init(argc, argv, "SM_MAIN_TRACK");
    ros::Rate loop(30);
    ros::NodeHandle n;
	
	FestinoNavigation::setNodeHandle(&n);
	FestinoHRI::setNodeHandle(&n);
    FestinoCommunication::setNodeHandle(&n);
    FestinoTask::setNodeHandle(&n);

	while(ros::ok() && !fail && !success){
	    switch(state){
			case SM_INIT:
	    		std::cout << "State machine: SM_INIT" << std::endl;	
	            std::cout << "I am ready for the main track challenge" << std::endl;
                
	    	    //state = SM_WAIT_FOR_ZONES;
	    	    state = SM_WAIT_FOR_INSTRUCTION;
	    		break;

            case SM_WAIT_FOR_ZONES:
                std::cout << "State machine: SM_WAIT_FOR_ZONES" << std::endl;
                instructionTokens.clear();
                if (FestinoCommunication::getInstruction(&instructionTokens,"z")){
                    FestinoNavigation::modifyMap(instructionTokens[0]);
                    ros::Duration(35, 0).sleep();
                    state = SM_WAIT_FOR_INSTRUCTION;
                }
                else
                {
                    ros::Duration(10, 0).sleep();
                }

			case SM_WAIT_FOR_INSTRUCTION:
	    		std::cout << "State machine: SM_WAIT_FOR_INSTRUCTION" << std::endl;	
                if(!request){
                    request_instruction();
                }
	    		break;

	    	case SM_MOVE:
	    		std::cout << "State machine: SM_MOVE" << std::endl;
                request = false;
                std::cout << "HDP -- " << instructionTokens[4] << std::endl;

                if (instructionTokens[4] != "")
                { 
                    instructionTokens.at(2) = instructionTokens.at(2) + "_" + instructionTokens[4];
                    state = SM_ALIGN;
                }
                else
                {
                    state = SM_WAIT_FOR_INSTRUCTION;
                }

                FestinoTask::navigate(instructionTokens[2]);
	    		break;

            case SM_ALIGN:
                std::cout << "State machine: SM_ALIGN" << std::endl;
                request = false;

                FestinoNavigation::alingWithLine(true);
                FestinoNavigation::move_base(1, 0, 0.1, 1.0);
                FestinoNavigation::alingWithLine(true);
                state = SM_WAIT_FOR_INSTRUCTION;
                break;

			case SM_RETRIEVE:
	    		std::cout << "State machine: SM_RETRIEVE" << std::endl;	
                request = false;
	            
                if(instructionTokens[4] == "shelf"){
		            std::cout << "State machine -> retrive shelf" << std::endl;
                    FestinoTask::grasp("takep");
                }
                else
                {
                    std::cout << "State machine -> retrive" << std::endl;
                    FestinoTask::grasp("take");
                }

                //Delay para que pueda tomar la pieza
	    		ros::Duration(30, 0).sleep();
                std::cout << "Ya pasaron los 30 seg" << std::endl;

                state = SM_WAIT_FOR_INSTRUCTION;
	    		break;
			case SM_DELIVER:
	    		std::cout << "State machine: SM_DELIVER" << std::endl;	
                request = false;	          
	    		
                if(instructionTokens[4] == "shelf"){
		            std::cout << "State machine -> deliver shelf" << std::endl;
                    FestinoTask::grasp("dropp");
                }
                else
                {
                    std::cout << "State machine -> deliver" << std::endl;
                    FestinoTask::grasp("drop");
                }

                //Delay para que pueda dejar la pieza
	    		ros::Duration(30, 0).sleep();
                std::cout << "Ya pasaron los 30 seg" << std::endl;

                state = SM_WAIT_FOR_INSTRUCTION;
	    		break;

	    	case SM_FINAL_STATE:
	    		//Navigate case
	    		std::cout << "State machine: SM_FINAL_STATE" << std::endl;	

                state = SM_FINAL_STATE;
	    		break;
		}
        ros::Duration(1, 0).sleep();
	    ros::spinOnce();
	    loop.sleep();
	}
	return 0;
}