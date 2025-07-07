//https://github.com/RobotJustina/JUSTINA/blob/develop/catkin_ws/src/planning/act_pln/src/carry_my_luggage.cpp
#include<iostream>
#include <sstream>
#include <algorithm>

#include "ros/ros.h"
#include "ros/time.h"

#include <cmath>
#include <vector> 
#include <string>
#include <tuple>

#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "robotino_msgs/DigitalReadings.h"
#include "actionlib_msgs/GoalStatus.h"


//Festino Tools
#include "festino_tools/FestinoHRI.h"
#include "festino_tools/FestinoVision.h"
#include "festino_tools/FestinoNavigation.h"

//Digital readings
#include "robotino_msgs/DigitalReadings.h"

//TF
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_listener.h"

//Nav
#include "actionlib_msgs/GoalStatus.h"


#include <stdlib.h>
#include <iostream>
#include "ros/ros.h"

#include "festino_tools/FestinoKnowledge.h"

//#include "justina_tools/JustinaVision.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseStamped.h"

#include "robotino_msgs/DigitalReadings.h"
#include "sensor_msgs/LaserScan.h"






#define GRAMMAR_POCKET_COMMANDS "grammars/carryMyLuggage.jsgf"

enum SMState {
	SM_INIT,
	SM_FIND_BAG,
	SM_WAIT_FOR_BAG,
	SM_CONF_POINTING_HAND,
	SM_FIND_PERSON,
	SM_FOLLOW_OPERATOR,
	SM_WAIT_CONF_CAR,
	SM_LEAVE_BAG,
	SM_FIND_QUEUE,
	SM_NAV_QUEUE,
	SM_FOLLOW_QUEUE,
	SM_BACK_TO_ARENA,
    SM_FINAL_STATE
};

bool fail = false;
bool success = false;
bool human_detector = false;
geometry_msgs::Pose human_coordinates;
geometry_msgs::PoseStamped tf_human_coordinates;
SMState state = SM_INIT;
std::string grammarCommandsID = "carryMyLuggageCommands";
actionlib_msgs::GoalStatus simple_move_goal_status;
int simple_move_status_id = 0;
bool stop = false;
int counter = 0;

std::vector<float> goal_vec(3);


bool HumanDetector()
{
    return human_detector;
}

void humanBoolCallback(const std_msgs::Bool::ConstPtr& msg)
{
    human_detector = msg -> data; 
}

void humanCoordinatesCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    human_coordinates = *msg; 
}

void transform_human_coordinates()
{

	tf::TransformListener listener;
    tf::StampedTransform transform;

	try{
        std::cout << "Waiting for transform: " << std::endl;
        listener.waitForTransform("/hokuyo_laser_link", "/camera_link", ros::Time(0), ros::Duration(1000.0));
        listener.lookupTransform("/hokuyo_laser_link", "/camera_link", ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    tf_human_coordinates.pose.position.x = -transform.getOrigin().x();
	tf_human_coordinates.pose.position.y = -transform.getOrigin().y();

	tf_human_coordinates.pose.position.x = tf_human_coordinates.pose.position.x*human_coordinates.position.x;
	tf_human_coordinates.pose.position.y = tf_human_coordinates.pose.position.y*human_coordinates.position.y;

	/*tf_listener->waitForTransform("map", base_link_name, ros::Time(0), ros::Duration(1000.0));
	tf::TransformListener listener;
    tf::StampedTransform transform;

	//Obtaining destination point from string 
	try{
		//listener.lookupTransform(human_coordinates, "/map", ros::Time(0), transform);
		listener.lookupTransform("map", "camera_link", ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    tf_human_coordinates.pose.position.x = -transform.getOrigin().x();
	tf_human_coordinates.pose.position.y = -transform.getOrigin().y();

	tf_human_coordinates.pose.position.x = tf_human_coordinates.pose.position.x*human_coordinates.position.x;
	tf_human_coordinates.pose.position.y = tf_human_coordinates.pose.position.y*human_coordinates.position.y;*/
}

void callback_simple_move_goal_status(const actionlib_msgs::GoalStatus::ConstPtr& msg)
{
    simple_move_goal_status = *msg;
    std::stringstream ss;
    ss << msg->goal_id.id;
    ss >> simple_move_status_id;
    //std::cout << simple_move_goal_status.status << std::endl;

}

int main(int argc, char** argv){
	ros::Time::init();
	bool latch;
	bool confirm_car_again = false;
	std::cout << "INITIALIZING CARRY MY LUGGAGE NODE... " << std::endl;
    ros::init(argc, argv, "SM");
    ros::NodeHandle n;
    FestinoHRI::setNodeHandle(&n);
    FestinoVision::setNodeHandle(&n);
    FestinoNavigation::setNodeHandle(&n);
	FestinoKnowledge::setNodeHandle(&n);

    ros::Publisher pub_digital = n.advertise<robotino_msgs::DigitalReadings>("/set_digital_values", 1000);
    ros::Subscriber sub_human_bool = n.subscribe("human_detector_bool", 1000, humanBoolCallback);
    ros::Subscriber sub_human_coordinates = n.subscribe("human_detector_coordinates", 1000, humanCoordinatesCallback);
    ros::Publisher pub_goal = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000); 
    ros::Subscriber sub_move_goal_status   = n.subscribe("/simple_move/goal_reached", 10, callback_simple_move_goal_status);

    ros::Rate loop(30);

    //Speaker
    std::string voice;

    //Open Pose variable
    //True - Left
    //False - Right
    bool pointing_hand;
    bool human_detector_bool;

    //Robotino Lights
    robotino_msgs::DigitalReadings arr_values;
    arr_values.stamp.sec = 0;
    arr_values.stamp.nsec = 0;
    arr_values.values = {0,0,0,0,0,0};
    //This delay is necessary
    FestinoHRI::say(" ",3);

    //TF related stuff 
	tf_human_coordinates.header.frame_id = "/map";
    tf_human_coordinates.pose.position.x = 0.0;
    tf_human_coordinates.pose.position.y = 0.0;
    tf_human_coordinates.pose.position.z = 0.0;
    tf_human_coordinates.pose.orientation.x = 0.0;
    tf_human_coordinates.pose.orientation.y = 0.0;
    tf_human_coordinates.pose.orientation.z = 0.0;
    tf_human_coordinates.pose.orientation.w = 0.0;

    //Dict with coords pose, 
    std::map<int, std::tuple<float, float, float>> poses;
    float currentX, currentY, currentTheta;
    
    float goalX, goalY, goalTheta;
    std::vector<float> goal_vec(3);

    // Variable para contar las poses
	int pose_counter = 0;



	while(ros::ok() && !fail && !success){
	    switch(state){
			case SM_INIT:
	    		//Init case
	    		std::cout << "State machine: SM_INIT" << std::endl;	
	    		FestinoHRI::loadGrammarSpeechRecognized(grammarCommandsID, GRAMMAR_POCKET_COMMANDS);
	    		
	    		//White light
	    		arr_values.values = {0,0,0,1,1,1};
                pub_digital.publish(arr_values);
                ros::Duration(0.5, 0).sleep();
                
	            ros::Duration(2, 0).sleep();
	            FestinoHRI::enableSpeechRecognized(false);
	            voice = "I am ready for the carry my luggage test";
	            FestinoHRI::say(voice, 5);
	            voice = "Please point at the bag that you want me to carry";
	    		FestinoHRI::say(voice, 5);
	    		state = SM_FIND_BAG;
	    		//state = SM_LEAVE_BAG;
	    		break;
	    	case SM_FIND_BAG:{
	    		std::cout << "State machine: SM_FIND_BAG" << std::endl;
	    		
				voice = "Tell me, robot yes, when you are pointing at the bag";
				FestinoHRI::say(voice, 5);

				//Waiting for command (red light)
				arr_values.values = {0,0,0,1,0,0};
                pub_digital.publish(arr_values);
                ros::Duration(0.5, 0).sleep();

				//Enable speech recognition 
				FestinoHRI::enableSpeechRecognized(true);
	    		ros::Duration(2, 0).sleep();

	    		//Waiting for the operator to confirm
	    		if(FestinoHRI::waitForSpecificSentence("robot yes", 5000)){
	    		
	    			//Disable speech recognition 
	    			FestinoHRI::enableSpeechRecognized(false);
					ros::Duration(2, 0).sleep();

	    			//Command recognized (green light)
					arr_values.values = {0,0,0,0,1,0};
                	pub_digital.publish(arr_values);
                	ros::Duration(0.5, 0).sleep();

	    			state = SM_CONF_POINTING_HAND;
	    		}
	    		
	    		break;
	        }

	    	case SM_CONF_POINTING_HAND:
	    		std::cout << "State machine: SM_CONF_POINTING_HAND" << std::endl;

	    		pointing_hand = FestinoVision::PointingHand();
	    		
	    		if (pointing_hand){
	    		
    				voice = "Are you pointing at the left bag? tell me Robot yes or Robot no";
    				FestinoHRI::say(voice, 5);

    				//Waiting for command (red light)
					arr_values.values = {0,0,0,1,0,0};
	                pub_digital.publish(arr_values);
	                ros::Duration(0.5, 0).sleep();

	                //Enable speech recognition 
					FestinoHRI::enableSpeechRecognized(true);
		    		ros::Duration(2, 0).sleep();

    				if(FestinoHRI::waitForSpecificSentence("robot yes", 5000)){
    					FestinoHRI::enableSpeechRecognized(false);

    					//Command recognized (green light)
						arr_values.values = {0,0,0,0,1,0};
	                	pub_digital.publish(arr_values);
	                	ros::Duration(0.5, 0).sleep();

    					FestinoNavigation::moveDistAngle(0.0, -0.2853, 10000);
		    			voice = "Please hang the left bag on my green hook";
		            	FestinoHRI::say(voice, 10);
		            	FestinoHRI::enableLegFinder(true);
		    			state = SM_WAIT_FOR_BAG;
    				}
	    		}
	    		else{
    				voice = "Are you pointing at the right bag? tell me Robot yes or Robot no";
    				FestinoHRI::say(voice, 5);

					//Waiting for command (red light)
					arr_values.values = {0,0,0,1,0,0};
	                pub_digital.publish(arr_values);
	                ros::Duration(0.5, 0).sleep();

	                //Enable speech recognition 
					FestinoHRI::enableSpeechRecognized(true);
		    		ros::Duration(2, 0).sleep();

    				if(FestinoHRI::waitForSpecificSentence("robot yes", 5000)){
    					FestinoHRI::enableSpeechRecognized(false);
    					//Command recognized (green light)
						arr_values.values = {0,0,0,0,1,0};
	                	pub_digital.publish(arr_values);
	                	ros::Duration(0.5, 0).sleep();

    					FestinoNavigation::moveDistAngle(0.0, 0.2853, 10000);
		    			voice = "Please hang the right bag on my green hook";
		            	FestinoHRI::say(voice, 10);
		            	FestinoHRI::enableLegFinder(true);
		    			state = SM_WAIT_FOR_BAG;
    				}
	    		}
	    		break;
	    	case SM_WAIT_FOR_BAG:
	    		std::cout << "State machine: SM_WAIT_FOR_BAG" << std::endl;	

	    		voice = "Tell me, robot yes, when you put the bag";
				FestinoHRI::say(voice, 5);

				//Waiting for command (red light)
				arr_values.values = {0,0,0,1,0,0};
                pub_digital.publish(arr_values);
                ros::Duration(0.5, 0).sleep();

				//Enable speech recognition
				FestinoHRI::enableSpeechRecognized(true);
				ros::Duration(2, 0).sleep();

				//Waiting for the operator to confirm
	    		if(FestinoHRI::waitForSpecificSentence("robot yes", 5000)){
	    			FestinoHRI::enableSpeechRecognized(false);
	    			//Save the point or coord of point
	    			//Command recognized (green light)
					arr_values.values = {0,0,0,0,1,0};
	                pub_digital.publish(arr_values);
	                ros::Duration(0.5, 0).sleep();

	                if(pointing_hand){
						FestinoNavigation::moveDistAngle(0.0, 0.2853, 10000);
					}
					else{
						FestinoNavigation::moveDistAngle(0.0, -0.2853, 10000);
					}

					state = SM_FIND_PERSON;
	    		}
	    		break;
	    	case SM_FIND_PERSON:
				std::cout << "State machine: SM_FIND_PERSON" << std::endl;	

				
    			if(!FestinoHRI::frontalLegsFound()){
	    			std::cout << "Not found legs" << std::endl;

	    			voice = "I can't found you, please stand in front of me";
					FestinoHRI::say(voice, 5);

	    			FestinoHRI::enableHumanFollower(false);
	    		}
    			else{
    				human_detector_bool = HumanDetector();

    				std::cout << human_detector_bool << std::endl;

    				if(human_detector_bool){
    					voice = "Say follow me when you are ready";
						FestinoHRI::say(voice, 2);

						//Waiting for command (red light)
						arr_values.values = {0,0,0,1,0,0};
	                	pub_digital.publish(arr_values);
	                	ros::Duration(0.5, 0).sleep();

						//Enable speech recognition
						FestinoHRI::enableSpeechRecognized(true);
						ros::Duration(2, 0).sleep();

						//Waiting for the operator to confirm
		    			if(FestinoHRI::waitForSpecificSentence("follow me", 5000)){
		    				FestinoHRI::enableSpeechRecognized(false);

		    				//Command recognized (green light)
							arr_values.values = {0,0,0,0,1,0};
			                pub_digital.publish(arr_values);
			                ros::Duration(0.5, 0).sleep();

		    				voice = "I'm going to follow you, please say here is the car if we reached the final destination";
							FestinoHRI::say(voice, 5);
		    				state = SM_FOLLOW_OPERATOR;
		    			}
    				}
    				else 
    				{
    					std::cout << "Not found person" << std::endl;
		    			voice = "I can't found you, please stand in front of me";
						FestinoHRI::say(voice, 5);
    				}
    				
	    		}
	    		break;
	    	case SM_FOLLOW_OPERATOR:
	    	    std::cout << "State machine: SM_FOLLOW_OPERATOR" << std::endl;	
    
			    //Following (turquoise light)
			    arr_values.values = {0,0,0,0,1,1};
			    pub_digital.publish(arr_values);
			    ros::Duration(0.5, 0).sleep();

			    if(confirm_car_again){
			        voice = "Please say here is the car if we reached the final destination";
			        FestinoHRI::say(voice, 5);
			        confirm_car_again = false;
			    }

			    FestinoHRI::enableHumanFollower(true);
			    FestinoHRI::enableSpeechRecognized(true);
			    //Storage the ultimate pose with Map
			    FestinoNavigation::getRobotPoseWrtMap(currentX, currentY, currentTheta);
			    poses[pose_counter] = std::make_tuple(currentX, currentY, currentTheta);

			    // Bucle para capturar las poses
			    while (!stop) {
			      
			        FestinoNavigation::getRobotPoseWrtMap(currentX, currentY, currentTheta);
					poses[pose_counter] = std::make_tuple(currentX, currentY, currentTheta);
			        // Guardar la pose cada 10 poses
			        //if (pose_counter % 3 == 0) {
			        //    poses[pose_counter] = std::make_tuple(currentX, currentY, currentTheta);
			        //}
			        //printf("%f\n",currentX );
			        // Incrementar el contador de poses
			        pose_counter++;

			        // Comprobar si se ha perdido al operador
			        if (!FestinoHRI::frontalLegsFound()) {
			            std::cout << "Lost operator" << std::endl;
			            voice = "I lost you";
			            FestinoHRI::say(voice, 5);
			            FestinoHRI::enableHumanFollower(false);
			            state = SM_FIND_PERSON;
			            break;
			        }

			        // Esperar a que se diga "here is the car"
			        if (FestinoHRI::waitForSpecificSentence("here is the car", 5000)) {

			       
			            FestinoHRI::enableSpeechRecognized(false);

			            // Command recognized (green light)
			            arr_values.values = {0,0,0,0,1,0};
			            pub_digital.publish(arr_values);
			            ros::Duration(0.5, 0).sleep();		        
			            stop = true;
			      
			       
			        }
			     }
			     FestinoHRI::enableHumanFollower(false);
			     state = SM_WAIT_CONF_CAR;
    			
	    		break;
	    	case SM_WAIT_CONF_CAR:
				voice = "Is this the car?, say Robot yes or Robot no";
				FestinoHRI::say(voice, 6);

				//Waiting for command (red light)
				arr_values.values = {0,0,0,1,0,0};
                pub_digital.publish(arr_values);
                ros::Duration(0.5, 0).sleep();

				//FestinoHRI::enableSpeechRecognized(true);
				ros::Duration(2, 0).sleep();

				if(FestinoHRI::waitForSpecificSentence("robot yes", 5000)){
					FestinoHRI::enableSpeechRecognized(false);
					//Command recognized (green light)
					arr_values.values = {0,0,0,0,1,0};
	                pub_digital.publish(arr_values);
	                ros::Duration(0.5, 0).sleep();

					std::cout << "Here is the car" << std::endl;
					FestinoHRI::enableLegFinder(false);
					FestinoHRI::enableHumanFollower(false);
					state = SM_LEAVE_BAG;
				}
				else{
					confirm_car_again = true;
					FestinoHRI::enableSpeechRecognized(false);
					state = SM_LEAVE_BAG;
				}

				break;
	    	case SM_LEAVE_BAG:
	    		std::cout << "State machine: SM_LEAVE_BAG" << std::endl;	
	    		voice = "Please take the bag";
	    		FestinoHRI::say(voice, 2);
	    		voice = "Tell me, robot yes, when you take the bag";
				FestinoHRI::say(voice, 5);
				FestinoHRI::enableHumanFollower(false);
				//Waiting for command (red light)
				arr_values.values = {0,0,0,1,0,0};
                pub_digital.publish(arr_values);
                ros::Duration(0.5, 0).sleep();

				FestinoHRI::enableSpeechRecognized(true);
				ros::Duration(2, 0).sleep();

				if(FestinoHRI::waitForSpecificSentence("robot yes", 5000)){
					FestinoHRI::enableSpeechRecognized(false);
					
					//Command recognized (green light)
					arr_values.values = {0,0,0,0,1,0};
	                pub_digital.publish(arr_values);
	                ros::Duration(0.5, 0).sleep();


	    			//Turn 180 degree
	    			//FestinoNavigation::moveDistAngle(0.0, 3.12, 20000);
	    			//FestinoNavigation::moveDist(3, 30000);

					//state = SM_FIND_QUEUE;
					state = SM_BACK_TO_ARENA;
	    		 }
	    		break;
	    	case SM_BACK_TO_ARENA:

	    		// Use the dictionary poses for local goals
	    		std::cout<<"State: Back to arena"<<std::endl;
				for (const auto& pose : poses) {
		        	counter = counter + 1;
		            // Obtener las coordenadas de la pose
					std::tie(goalX, goalY, goalTheta) = pose.second;
		            printf("This is point X %f\n , This is point Y %f\n", goalX, goalY);
		            
		         
                	     
		        }

				goal_vec = FestinoKnowledge::CoordenatesLocSrv("living_room");
                std::cout <<"Coordenates of inspection_point:"<<std::endl;
                std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                    if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2], 120000))
                    	std::cout << "Cannot move to inspection point" << std::endl;
		        /*for (const auto& pose : poses) {
		        	counter = counter + 1;
		            // Obtener las coordenadas de la pose
		            printf("This is point %d\n", counter);
		            std::tie(goalX, goalY, goalTheta) = pose.second;

		            if(!FestinoNavigation::getClose(goal_vec[goalX], goal_vec[goalY], goal_vec[goalTheta],120000))
                   		 if(!FestinoNavigation::getClose(goal_vec[goalX], goal_vec[goalY], goal_vec[goalTheta], 120000))
                       		 std::cout << "Cannot move to inspection point" << std::endl;
                
                	     
		        }*/


		        FestinoNavigation::stopNavigation();
		        FestinoHRI::enableHumanFollower(false);

		        FestinoHRI::say("I have reached the living room", 4);
		        state = SM_FINAL_STATE;
	    	

	    		break;	

	    	case SM_FIND_QUEUE:
	    		std::cout << "State machine: SM_FIND_QUEUE" << std::endl;
	    		ros::Duration(2, 0).sleep();

	    		/*transform_human_coordinates();
	    		pub_goal.publish(tf_human_coordinates);
	    		
	    		state = SM_NAV_QUEUE;*/

	    		FestinoHRI::enableLegFinder(true);

    			if(!FestinoHRI::frontalLegsFound()){
	    			std::cout << "Not found" << std::endl;

	    			FestinoHRI::enableHumanFollower(false);
	    		}
    			else{
    				voice = "Im going to follow the queue";
	            	FestinoHRI::say(voice, 2);

	    			state = SM_FOLLOW_QUEUE;
	    		}

	    		break;
	    	case SM_NAV_QUEUE:{
            	//Wait for finished navigation
	            std::cout << "State machine: SM_NAV_QUEUE" << std::endl;
	            voice = "Navigating to queue";
	    		FestinoHRI::say(voice, 2);

	            if(simple_move_goal_status.status == actionlib_msgs::GoalStatus::SUCCEEDED && simple_move_status_id == -1){
	                voice = "Goal location reached";
	                FestinoHRI::say(voice, 2);

	                FestinoHRI::enableLegFinder(true);
					FestinoHRI::enableHumanFollower(true);

	               state = SM_FOLLOW_QUEUE;
	            }
	            break;
	        }
	    	case SM_FOLLOW_QUEUE:
	    		std::cout << "State machine: SM_FOLLOW_QUEUE" << std::endl;

	    		FestinoHRI::enableHumanFollower(true);
	    		ros::Duration(200, 0).sleep();
	    		FestinoHRI::enableHumanFollower(false);

	    		state = SM_FINAL_STATE;

	    		break;
	    	case SM_FINAL_STATE:
	    		std::cout << "State machine: SM_FINAL_STATE" << std::endl;	

	    		//Test finished (white light)
				arr_values.values = {0,0,0,1,1,1};
                pub_digital.publish(arr_values);
                ros::Duration(0.5, 0).sleep();

	            voice =  "I have finished the test";
	            FestinoHRI::say(voice, 2);
	    		success = true;
	    		fail = true;
	    		break;
		}
	    ros::spinOnce();
	    loop.sleep();
	}
	return 0;
}