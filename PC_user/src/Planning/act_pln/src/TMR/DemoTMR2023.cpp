//https://github.com/RobotJustina/JUSTINA/blob/develop/catkin_ws/src/planning/act_pln/src/carry_my_luggage.cpp
#include<iostream>
#include <sstream>
#include <algorithm>
#include <map>

#include "ros/ros.h"
#include "ros/time.h"

#include <cmath>
#include <vector> 
#include <string>

#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "robotino_msgs/DigitalReadings.h"
#include "actionlib_msgs/GoalStatus.h"


//Festino Tools
#include "festino_tools/FestinoHRI.h"
#include "festino_tools/FestinoVision.h"
#include "festino_tools/FestinoNavigation.h"
#include "festino_tools/FestinoKnowledge.h"

#include "festino_tools/FestinoHRI.h"
#include "festino_tools/FestinoNavigation.h"
#include "festino_tools/FestinoVision.h"
#include "festino_tools/FestinoKnowledge.h"

#include "std_msgs/Bool.h"

#include "string"

#include <iostream>
#include <stdlib.h>
#include <vector>
#include <string>
#include <sstream>
#include "ros/ros.h"

//Libraries for FestionoTools
#include "festino_tools/FestinoHRI.h"
#include "festino_tools/FestinoVision.h"
#include "festino_tools/FestinoNavigation.h"
#include "festino_tools/FestinoKnowledge.h"
#include "robotino_msgs/DigitalReadings.h"
#include "sensor_msgs/LaserScan.h"

#include "std_msgs/Bool.h"
#include "string"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseStamped.h"
//#include "boost/date_time/posix_time.hpp"
//#include "boost/thread/thread.hpp"
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
//#include <boost/algorithm/string/split.hpp>
#include "sensor_msgs/LaserScan.h"

//Nuevos abajo

#include "object_classification/Classify.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <sensor_msgs/Image.h>
//Nuevos arriba

//Digital readings
#include "robotino_msgs/DigitalReadings.h"

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>

#define GRAMMAR_POCKET_COMMANDS "grammars/gpsr.jsgf"
#define GRAMMAR_POCKET_CONFIRMATIONS "grammars/confirmation.jsgf"

#define GRAMMAR_POCKET_COMMANDS_R "grammars/receptionist_commands.jsgf"
#define GRAMMAR_POCKET_DRINKS "grammars/order_drinks.jsgf"
#define GRAMMAR_POCKET_NAMES "grammars/people_names.jsgf"

std::vector<sensor_msgs::Image> class_image;
object_classification::Classify::Response message_clas;


enum SMState
{
	SM_INIT,
    SM_WAIT_FOR_DOOR,
    //SM_NAVIGATE_TO_INSTRUCTION_POINT,
    //SM_NAVIGATE_TO_LOCATION,
    SM_WAIT_FOR_INSTRUCTION,
    SM_FOLLOW_PERSON_GPSR,
    SM_FOLLOW_OPERATOR,
    SM_PICK_OBJECT,
    SM_BRING_PERSON,
    SM_FIND_PERSON_FOLLOW,
    SM_FIND_PERSON_GPSR,
    SM_RETURN_TO_INSTRUCTION_POINT,
    SM_FINISH_CHALLENGE,
    SM_FINISHED_TASK,
    SM_CARRY_MY_LUGGAGE,
    SM_FIND_OBJECT_GPSR,
    SM_LOOK_FOR_PERSON,
    SM_WAIT_FOR_DRINK,
    SM_PRESENTATION_CONFIRM,
};

    //Robotino Lights
robotino_msgs::DigitalReadings arr_values;
ros::Publisher pub_digital;

bool fail = false;
bool success = false;
SMState state = SM_INIT;
std::vector<float> goal_vec(3);
sensor_msgs::LaserScan laserScan;
    std::vector<std::string> findPersonDetect;

std::string param,lastDrink;
std::stringstream ss;
    std::stringstream ss2;
std::string lastRecoSpeech;
std::vector<std::string> tokens;
    std::vector<std::string> drinks;
    std::string test("receptionist");

std::string grammarCommandsID = "GPSRCommands";
std::string grammarConfirmationsID = "ConfirmationCommands";
    std::string grammarCommandsID_R = "receptionisCommands";
    std::string grammarDrinksID = "receptionistDrinks";
    std::string grammarNamesID = "receptionistNames";


bool flag_door = true;
bool bring_find = false;

int tasks_finished = 0;

std::string object_person_location = "";
std::string place_to_take_the_object_to = "instruction_point_a";
std::string object_person_name = "";

bool human_detector_bool = false;

void humanDetectorCallback(const std_msgs::Bool::ConstPtr& msg)
{
    human_detector_bool = msg -> data; 
}

void callbackLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laserScan = *msg;

    int range=0,range_i=0,range_f=0,range_c=0,cont_laser=0;
    float laser_l=0;
    range=laserScan.ranges.size();
    std::cout<<laserScan.ranges.size()<<std::endl;
    range_c=range/2;
    range_i=range_c-(range/10);
    range_f=range_c+(range/10);
    //std::cout<<"Range Size: "<< range << "\n ";
    //std::cout<<"Range Central: "<< range_c << "\n ";
    //std::cout<<"Range Initial: "<< range_i << "\n ";
    //std::cout<<"Range Final: "<< range_f << "\n ";

    cont_laser=0;
    laser_l=0;
    for(int i=range_c-(range/10); i < range_c+(range/10); i++)
    {
        if(laserScan.ranges[i] > 0 && laserScan.ranges[i] < 4)
        { 
            laser_l=laser_l+laserScan.ranges[i]; 
            cont_laser++;
        }
    }
    //std::cout<<"Laser promedio: "<< laser_l/cont_laser << std::endl;    
    if(laser_l/cont_laser > 0.5)
    {
        flag_door = true;
        //std::cout<<"door open"<<std::endl;
    }
    else
    {
        flag_door = false;
        //std::cout<<"door closed"<<std::endl;
    }
}

//Locations
std::map<std::string, std::string> locations;//key = robot location, value = human location
void navigate_to_location(std::string location)
{

    std::cout << "Navigate to location " << location << std::endl;
    arr_values.values = {0,0,0,0,1,1};
    pub_digital.publish(arr_values);
    goal_vec = FestinoKnowledge::CoordenatesLocSrv(location);
    std::cout <<"Coordenates of " + locations[location] + ":"<<std::endl;
    std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
    if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000)){
        if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2], 120000)){
         	std::cout << "Cannot move to " << locations[location] << std::endl;
                FestinoHRI::say("Just let me go. Cries in robot iiiiii",3);
        }
    }
                FestinoHRI::say("I have arrived to " + locations[location],3);	
                if(location == "instruction_point"){
       	    		FestinoHRI::loadGrammarSpeechRecognized(grammarCommandsID, GRAMMAR_POCKET_COMMANDS);
                    FestinoHRI::say("I'm listening for instructions",4);
                    FestinoHRI::enableSpeechRecognized(true);
                    sleep(6);
                }
/*
                arr_values.values = {0,0,0,1,0,0};
                pub_digital.publish(arr_values);
                ros::Duration(0.5, 0).sleep();
                arr_values.values = {0,0,0,0,0,0};
                pub_digital.publish(arr_values);
                ros::Duration(0.5, 0).sleep();
                arr_values.values = {0,0,0,1,0,0};
                pub_digital.publish(arr_values);
                */
}

std::string find_name(std::vector<std::string> tokens){
    for(std::string name: tokens){
        if(name == "jamie" || name == "morgan" || name == "michael" || name == "jordan" || name == "taylor" || name == "tracy" ||
        name == "robin" || name == "alex" || name == "apple" || name == "mug" ||
        name == "banana" || name == "ball" || name == "mustard" || name == "plate" || name == "pear"){
            return name;
        }
    }

    return "not_found";
}

bool its_an_object(std::string name){
    if(name == "apple" || name == "mug" ||
        name == "banana" || name == "ball" || name == "mustard" || name == "plate" || name == "pear"){
            return true;
        }

        return false;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	class_image.clear();
	std::cout << "holi call" << std::endl;
    class_image.push_back(*msg);

}


int main(int argc, char** argv){
	ros::Time::init();
	std::cout << "INITIALIZING GPSR NODE... " << std::endl;
    ros::init(argc, argv, "SM");
    ros::NodeHandle n;

    ros::Subscriber subLaserScan = n.subscribe("/scan", 1, callbackLaserScan);

    FestinoHRI::setNodeHandle(&n);
    FestinoVision::setNodeHandle(&n);
    FestinoNavigation::setNodeHandle(&n);
    FestinoKnowledge::setNodeHandle(&n);

    std::vector<sensor_msgs::Image> image_bb;

    ros::Subscriber class_image_sub = n.subscribe("/camera/rgb/image_color", 1000, imageCallback);
    ros::ServiceClient client_image_class = n.serviceClient<object_classification::Classify>("classify"); 
	object_classification::Classify srv;
 

    pub_digital = n.advertise<robotino_msgs::DigitalReadings>("/set_digital_values", 1000);
    ros::Subscriber sub_human = n.subscribe("human_detector_bool", 1000, humanDetectorCallback);

    ros::Rate loop(30);

    //Speaker
    std::string voice;

    drinks.push_back("coke");
    //Locations
    locations["instruction_point"] = "gpsr";
    locations["exit"] = "entrance_door";
    locations["entrance"] = "entrance_door";
    locations["kitchen"] = "kitchen";
    locations["dinning_room"] = "dinning_room";
    locations["living_room"] = "living_room";
    locations["bedroom"] = "bedroom";

    //bool human_detector_bool;

    //Robotino Lights
    arr_values.stamp.sec = 0;
    arr_values.stamp.nsec = 0;
    arr_values.values = {0,0,0,0,0,0};
    //This delay is necessary ... why?
    FestinoHRI::say(" ",3);

	while(ros::ok() && !fail && !success){

        std::cout << "In Classification state" << std::endl;
        if(!class_image.empty()){
            srv.request.in.image_msgs = class_image;
            if(client_image_class.call(srv)){
                message_clas = srv.response;
                cv::Mat image = cv_bridge::toCvCopy(message_clas.debug_image.image_msgs.at(0), sensor_msgs::image_encodings::TYPE_8UC3)->image;
                cv::namedWindow("Imgclass", cv::WINDOW_NORMAL);
                cv::resizeWindow("Imgclass", 600, 600);
                cv::imshow("Imgclass", image);
                cv::waitKey(1);
            }
        }

	    switch(state){

			case SM_INIT:
            {
	    		//Init case
	    		std::cout << "State machine: SM_INIT" << std::endl;	
	    		FestinoHRI::loadGrammarSpeechRecognized(grammarCommandsID, GRAMMAR_POCKET_COMMANDS);
	    		
	    		//White light
	    		arr_values.values = {0,0,0,1,1,1};
                pub_digital.publish(arr_values);
                ros::Duration(0.5, 0).sleep();
                
	            ros::Duration(2, 0).sleep();
	            FestinoHRI::enableSpeechRecognized(false);
                FestinoHRI::clean_lastRecogSpeech();
	            voice = "I am ready for the DEMO";
	            FestinoHRI::say(voice, 3);
                
                arr_values.values = {0,0,0,1,0,0};
                pub_digital.publish(arr_values);
                ros::Duration(0.5, 0).sleep();
                arr_values.values = {0,0,0,0,0,0};
                pub_digital.publish(arr_values);
                ros::Duration(0.5, 0).sleep();
                arr_values.values = {0,0,0,1,0,0};
                pub_digital.publish(arr_values);
	    		
                state = SM_WAIT_FOR_DOOR;
                ////////////////////////////////////////
                //state = SM_WAIT_FOR_INSTRUCTION;
                //FestinoHRI::say("I'm listening for instructions",4);
                //FestinoHRI::enableSpeechRecognized(true);
                //sleep(6);
                /////////////////////////////////////////
                break;
            }
            case SM_WAIT_FOR_DOOR:
            {
                std::cout << "State machine: SM_WAIT_FOR_DOOR" << std::endl;
                FestinoHRI::say("I am waiting for the door to be open",3);
                //flag_door = false;
                if(flag_door){

                    arr_values.values = {0,0,0,0,1,1};
                    pub_digital.publish(arr_values);
                    ros::Duration(0.5, 0).sleep();
                    arr_values.values = {0,0,0,0,0,0};
                    pub_digital.publish(arr_values);
                    ros::Duration(0.5, 0).sleep();
                    arr_values.values = {0,0,0,0,1,1};
                    pub_digital.publish(arr_values);

                    FestinoHRI::say("I can see that the door is open, I am going to the instruction point",5);

                    navigate_to_location("instruction_point");

                    state = SM_WAIT_FOR_INSTRUCTION;
                }
                break;

            }
            case SM_WAIT_FOR_INSTRUCTION:
            {
                std::cout << "State machine: SM_WAIT_FOR_INSTRUCTION" << std::endl;

                arr_values.values = {0,0,0,1,0,0};
                pub_digital.publish(arr_values);
                
                //FestinoHRI::say("I'm listening for instructions",4);

                //arr_values.values = {0,0,0,1,0,0};
                //pub_digital.publish(arr_values);
   	    		//FestinoHRI::loadGrammarSpeechRecognized(grammarCommandsID, GRAMMAR_POCKET_COMMANDS);

                //FestinoHRI::clean_lastRecogSpeech();
                //FestinoHRI::enableSpeechRecognized(true);
                //sleep(6);
                //arr_values.values = {0,0,0,0,1,0};
                //pub_digital.publish(arr_values);
                //FestinoHRI::enableSpeechRecognized(false);
                

/*
Go to {location}
Go to {location} and find {object}
Go to {location} and find {persFon}
Go to {location} and bring me {object}
*/


                tokens.clear();
                lastRecoSpeech = FestinoHRI::lastRecogSpeech();
                //sleep(5);
                std::cout << "escucheee " << lastRecoSpeech << std::endl;

                if(lastRecoSpeech != ""){

                    tokens.clear();
                    boost::algorithm::split(tokens,lastRecoSpeech, boost::algorithm::is_any_of(" "));
                    FestinoHRI::clean_lastRecogSpeech();
                    if(tokens[0] == "go" && tokens[1] == "to"){
                        FestinoHRI::enableSpeechRecognized(false);
                        arr_values.values = {0,0,0,0,1,0};
                        pub_digital.publish(arr_values);
                        FestinoHRI::say("Did you say " + lastRecoSpeech + ", please say robot yes or robot no",7);
                        arr_values.values = {0,0,0,1,0,0};
                        pub_digital.publish(arr_values);
                        FestinoHRI::loadGrammarSpeechRecognized(grammarConfirmationsID, GRAMMAR_POCKET_CONFIRMATIONS);
                        FestinoHRI::enableSpeechRecognized(true);
                        if(FestinoHRI::waitForSpecificSentence("robot yes",10000)){
                            FestinoHRI::enableSpeechRecognized(false);
                            for(std::string location : tokens){
                                if(location == "living"){
                                    object_person_location = "living_room";
                                    break;
                                }
                                if(location == "dinning"){
                                    object_person_location = "dinning_room";
                                    break;
                                }
                                if(location == "bedroom"){
                                    object_person_location = "bedroom";
                                    break;
                                }
                                if(location == "kitchen"){
                                    object_person_location = "kitchen";
                                    break;
                                }
                            }

                            bool go_to = true;
                            for(std::string tok : tokens){
                                if(tok == "find"){
                                    bring_find = false;
                                    go_to = false;
                                    object_person_name = find_name(tokens);
                                    if(object_person_name != "not_found"){
                                        if(its_an_object(object_person_name)){//if its an object
                                            state = SM_FIND_OBJECT_GPSR;
                                        } else {
                                            state = SM_FIND_PERSON_GPSR;
                                        }
                                    } else {
                                        std::cout << " no entendi 1 " << std::endl;

                                        FestinoHRI::say("I didn't understood, could you please repeat the instruction?",3);	
                                        sleep(3);
                                    }
                                    break;
                                }
                                if(tok == "bring"){
                                    bring_find = true;
                                    go_to = false;
                                    object_person_name = find_name(tokens);
                                    if(object_person_name != "not_found"){
                                        if(its_an_object(object_person_name)){//if its an object
                                            state = SM_PICK_OBJECT;
                                        } else {
                                            state = SM_FIND_PERSON_GPSR;  //if its a person
                                        }
                                        
                                    } else {
                                        std::cout << " no entendi 2 " << std::endl;

                                        FestinoHRI::say("I didn't understood, could you please repeat the instruction?",3);	
                                        sleep(3);
                                    }
                                    break;
                                }
                            }
                            if(go_to){
                                arr_values.values = {0,0,0,0,1,0};
                                pub_digital.publish(arr_values);
                                FestinoHRI::say("I'm going to navigate to " + locations[object_person_location],4);
                                navigate_to_location(object_person_location);
                                state = SM_FINISHED_TASK;
                                break;
                            }
                        }
                        else{
                            FestinoHRI::loadGrammarSpeechRecognized(grammarCommandsID, GRAMMAR_POCKET_COMMANDS);
                            FestinoHRI::say("Please repeat instruction",4);
                            FestinoHRI::enableSpeechRecognized(true);
                            sleep(6);
                            break;
                        }
                    } 
                    else {

                        if(tokens[0] == "carry"){
                            state = SM_CARRY_MY_LUGGAGE;
                        }
                        else{
                            std::cout << " no entendi 3 " << std::endl;
                            arr_values.values = {0,0,0,0,1,0};
                            pub_digital.publish(arr_values);
                            FestinoHRI::say("I didn't understood, could you please repeat the instruction?",3);	
                            sleep(3);
                        }
                        
                    }

                }

                break;
            }
            case SM_FOLLOW_PERSON_GPSR:
            {
                state = SM_FIND_PERSON_FOLLOW;
                break;
            }
            case SM_FIND_PERSON_FOLLOW:
            {

                std::cout << "State machine: SM_FOLLOW_PERSON" << std::endl;
                arr_values.values = {0,0,0,1,0,1};
                pub_digital.publish(arr_values);
    			if(!FestinoHRI::frontalLegsFound()){
	    			std::cout << "Not found" << std::endl;

	    			voice = "I can't found you, please stand in front of me";
					FestinoHRI::say(voice, 5);

	    			FestinoHRI::enableHumanFollower(false);
	    		}
    			else{

    				if(human_detector_bool){
                        arr_values.values = {0,0,0,0,1,0};
                        pub_digital.publish(arr_values);
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
		    			voice = "I can't found you, please stand in front of me";
						FestinoHRI::say(voice, 5);
    				}
    				
	    		}
                break;
            }
	    	case SM_FOLLOW_OPERATOR:
            {
	    		std::cout << "State machine: SM_FOLLOW_OPERATOR" << std::endl;	
	    		
	    		//Following (turquoise light)
				arr_values.values = {0,0,0,1,0,1};
                pub_digital.publish(arr_values);
                ros::Duration(0.5, 0).sleep();
/* Confirm Arrive location
	    		if(confirm_car_again){
	    			voice = "Please say here is the car if we reached the final destination";
	    			FestinoHRI::say(voice, 5);
	    			confirm_car_again = false;
	    		}*/
	    		
				FestinoHRI::enableHumanFollower(true);

	    		FestinoHRI::enableSpeechRecognized(true);

	    		if(!FestinoHRI::frontalLegsFound()){
	    			std::cout << "Lost operator" << std::endl;
	    			voice = "I lost you";
					FestinoHRI::say(voice, 5);
	    			FestinoHRI::enableHumanFollower(false);
	    			state = SM_FIND_PERSON_FOLLOW;
                    break;
	    		}
    			/*
                Wait to arrive
    			if(FestinoHRI::waitForSpecificSentence("here is the car", 5000)){

    				//Command recognized (green light)
					arr_values.values = {0,0,0,0,1,0};
	                pub_digital.publish(arr_values);
	                ros::Duration(0.5, 0).sleep();

    				FestinoHRI::enableSpeechRecognized(false);
    				state = SM_WAIT_CONF_CAR;  SM_FINISHED_TASK
    			}
    			*/
	    		break;
            }

            case SM_PICK_OBJECT:
            {
                //Find the object
                navigate_to_location(object_person_location);
                arr_values.values = {0,0,0,1,1,0};
                pub_digital.publish(arr_values);
                //Pick up object

                    FestinoHRI::say("Please put the " + object_person_name + " in the tray, tell me, robot yes, when you put the " + object_person_name + " in the tray",5);
                    sleep(5);

				//Waiting for command (red light)
                ros::Duration(0.5, 0).sleep();
                arr_values.values = {0,0,0,1,0,0};
                pub_digital.publish(arr_values);
				//Enable speech recognition
				FestinoHRI::enableSpeechRecognized(true);
				ros::Duration(2, 0).sleep();

				//Waiting for the operator to confirm
                FestinoHRI::loadGrammarSpeechRecognized(grammarConfirmationsID, GRAMMAR_POCKET_CONFIRMATIONS);

	    		if(FestinoHRI::waitForSpecificSentence("robot yes", 5000)){

	    			FestinoHRI::enableSpeechRecognized(false);

	    			//Command recognized (green light)
					arr_values.values = {0,0,0,0,1,0};
	                pub_digital.publish(arr_values);
	                ros::Duration(0.5, 0).sleep();

                    navigate_to_location(place_to_take_the_object_to);

                    FestinoHRI::say("Here is the " + object_person_name + ", plase take the " + object_person_name + " from the tray and tell me robot yes when you did it",5);
                    sleep(5);

                    FestinoHRI::loadGrammarSpeechRecognized(grammarConfirmationsID, GRAMMAR_POCKET_CONFIRMATIONS);

	    		    if(FestinoHRI::waitForSpecificSentence("robot yes", 5000)){
					    state = SM_FINISHED_TASK;
                    }
	    		}

                break;
            }
            case SM_BRING_PERSON:
            {
                FestinoHRI::say("Hello padawan " + object_person_name + ", please tell me, i want, and after that your favorite drink", 7);
                FestinoHRI::loadGrammarSpeechRecognized(grammarDrinksID,GRAMMAR_POCKET_DRINKS);
                FestinoHRI::enableSpeechRecognized(true);
                arr_values.values = {0,0,0,1,0,0};
                pub_digital.publish(arr_values);
                sleep(2);
                sleep(2);

               state = SM_WAIT_FOR_DRINK;
               break;
            }
            case SM_WAIT_FOR_DRINK:
            {
                //std::cout << test << ".-> State SM_WAIT_FOR_PRESENTATION: Waiting for the names." << std::endl;
                arr_values.values = {0,0,0,0,1,0};
                pub_digital.publish(arr_values);

                tokens.clear();
                lastRecoSpeech = FestinoHRI::lastRecogSpeech();

                std::cout << "frase :"<<lastRecoSpeech<<std::endl;
                if(lastRecoSpeech != "")
                    {
                        boost::algorithm::split(tokens,lastRecoSpeech, boost::algorithm::is_any_of(" "));
                        if(tokens[0].compare("i") == 0)
                        {
                            if(param.compare(" ") != 0 || param.compare("") != 0)
                            {
                                ss.str("");
                                ss2.str("");
                                ss << "Ok, your favorite drink is ";
                                boost::algorithm::split(tokens,lastRecoSpeech, boost::algorithm::is_any_of(" "));
                                ss2.str("");
                                    for(int i = 0; i < tokens.size(); i++)
                                {
                                    std::cout<<"token ["<<i<<"]: "<<tokens[i]<<std::endl;
                                    if(i < tokens.size() -1)
                                        ss2 << " ";
                                }
                                
                                if(tokens[2].compare("orange") == 0)
                                {
                                        ss << tokens[2]<<" "<<tokens[3];
                                        lastDrink = "orange juice";
                                }
                                else
                                {
                                    ss << tokens[2];
                                    lastDrink = tokens[2];
                                }

                                //names.push_backack(ss2.str());
                                ss << ", tell me robot yes or robot no";
                                
                                FestinoHRI::say(ss.str(), 6);
                                state = SM_PRESENTATION_CONFIRM;
                            }
                            break;
                        }
                    }
                break;
            }

            case SM_PRESENTATION_CONFIRM:
            {
                std::cout << test << ".-> State SM_PRESENTATION_CONFIRM. Wait for robot yes or robot no" << std::endl;
                
                FestinoHRI::loadGrammarSpeechRecognized(grammarCommandsID_R,GRAMMAR_POCKET_COMMANDS_R);
                FestinoHRI::enableSpeechRecognized(true);
                arr_values.values = {0,0,0,1,0,0};
                pub_digital.publish(arr_values);
                sleep(2);
                sleep(2);

                if (FestinoHRI::waitForSpecificSentence("robot yes",5000))
                {
                        FestinoHRI::enableSpeechRecognized(false);
                        drinks.push_back(lastDrink);
                        ss2.str("");
                        ss2 << "Ok, your favorite drink is " << drinks[drinks.size() - 1]<<", i'm going to the kitchen for your " << drinks[drinks.size() -1] << ", please padawan " << object_person_name << " go to the living room";
                        arr_values.values = {0,0,0,0,1,0};
                        pub_digital.publish(arr_values);
                        FestinoHRI::say(ss2.str(), 9);
                        
                        FestinoHRI::clean_lastRecogSpeech();
                        navigate_to_location("kitchen");
                        arr_values.values = {0,0,0,0,1,1};
                        pub_digital.publish(arr_values);
                        FestinoHRI::say("please master, put the drink in my tray, and tell me robot yes",6);
                        sleep(2);
                        FestinoHRI::clean_lastRecogSpeech();
                        FestinoHRI::loadGrammarSpeechRecognized(grammarCommandsID_R,GRAMMAR_POCKET_COMMANDS_R);
                        FestinoHRI::enableSpeechRecognized(true);
                        arr_values.values = {0,0,0,1,0,0};
                        pub_digital.publish(arr_values);
                        sleep(2);
                        sleep(2);
                        if (FestinoHRI::waitForSpecificSentence("robot yes",5000))
                        {
                            navigate_to_location("living_room");
                            ss.str("");
                            ss << "Please human, take your " << drinks[drinks.size() -1 ];
                            FestinoHRI::say(ss.str(), 6);
                            state = SM_FINISHED_TASK;
                            lastRecoSpeech = "";
                            FestinoHRI::clean_lastRecogSpeech();
                        }
                }
                else
                {
                    FestinoHRI::enableSpeechRecognized(false);
                    arr_values.values = {0,0,0,0,1,0};
                    pub_digital.publish(arr_values);
                    //drinks.erase(names.end() - 1);
                    FestinoHRI::clean_lastRecogSpeech();
                    FestinoHRI::say("Sorry I did not understand you, Please tell me, i want and after that your favorite drink", 8);
                    FestinoHRI::loadGrammarSpeechRecognized(grammarDrinksID, GRAMMAR_POCKET_DRINKS);
                    FestinoHRI::enableSpeechRecognized(true);
                    arr_values.values = {0,0,0,1,0,0};
                    pub_digital.publish(arr_values);
                    sleep(2);
                    sleep(2);
                    state = SM_WAIT_FOR_DRINK;
                }
                break;
            }

            case SM_FIND_PERSON_GPSR:
            {

                //Find the person
                navigate_to_location(object_person_location);

                state = SM_LOOK_FOR_PERSON;

                break;
            }
            case SM_LOOK_FOR_PERSON:
            {
                //Recognize
                arr_values.values = {0,0,0,0,0,1};
                pub_digital.publish(arr_values);
               
                findPersonDetect = FestinoVision::enableRecogFacesName(true);
                for(std::string person_found : findPersonDetect)
                {
                    if(person_found == object_person_name)
                    {
                        FestinoHRI::say(object_person_name + " I found you",3);
                        sleep(3);
                        if(bring_find)
                        {
                            state = SM_BRING_PERSON;
                            break;
                        }
                        else
                        {
                            state = SM_FINISHED_TASK;
                            break;
                        }
                    }
                
                }
                FestinoNavigation::moveDistAngle(0, 0.1, 100);
                break;
            }                
            case SM_FIND_OBJECT_GPSR:
            {
                navigate_to_location(object_person_location);
                arr_values.values = {0,0,0,0,0,1};
                pub_digital.publish(arr_values);
                //Find object
                        FestinoHRI::say(" I found you the " + object_person_name,3);
                        sleep(3);


                        state = SM_FINISHED_TASK;
                        break;
                
            }
            case SM_FINISHED_TASK:
            {
                std::cout << "State machine: SM_FINISH_CHALLENGE" << std::endl;

                tasks_finished++;
                if(tasks_finished >= 20){
                    state = SM_FINISH_CHALLENGE;
                } else {
                    state = SM_RETURN_TO_INSTRUCTION_POINT;
                }

                break;
            }
            case SM_RETURN_TO_INSTRUCTION_POINT:
            {

                    std::cout << "State machine: SM_RETURN_TO_INSTRUCTION_POINT" << std::endl;
    
                    arr_values.values = {0,0,0,0,1,1};
                    pub_digital.publish(arr_values);
                    ros::Duration(0.5, 0).sleep();
                    arr_values.values = {0,0,0,0,0,0};
                    pub_digital.publish(arr_values);
                    ros::Duration(0.5, 0).sleep();
                    arr_values.values = {0,0,0,0,1,1};
                    pub_digital.publish(arr_values);

                    FestinoHRI::say("I've completed the task number" + std::to_string(tasks_finished) + "I am going back to the instruction point for the next task",3);
                    sleep(3);

                    navigate_to_location("instruction_point");
                    state = SM_WAIT_FOR_INSTRUCTION;

                    break;
            }
            case SM_FINISH_CHALLENGE:
            {
                arr_values.values = {0,0,0,1,1,1};
                pub_digital.publish(arr_values);
                std::cout << "State machine: SM_FINISH_CHALLENGE" << std::endl;


                FestinoHRI::say("I've completed the GPSR challenge, bye",3);
                sleep(3);

                navigate_to_location("exit");
                
                success = true;
	    		fail = true;

                break;
            }

        }
	    ros::spinOnce();
	    loop.sleep();
	}
	return 0;
}