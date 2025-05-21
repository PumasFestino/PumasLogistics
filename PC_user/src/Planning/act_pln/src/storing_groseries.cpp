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

std::vector<sensor_msgs::Image> class_image;
object_classification::Classify::Response message_clas;


#define SM_INIT 0
#define SM_WAIT_FOR_START_COMMAND 10
#define SM_NAVIGATION_TO_TABLE 20
#define SM_FIND_OBJECTS_ON_TABLE 30
#define SM_SAVE_OBJECTS_PDF 40
#define SM_TAKE_OBJECT_RIGHT 50
#define SM_TAKE_OBJECT_LEFT 60
#define SM_GOTO_CUPBOARD 70
#define SM_FIND_OBJECTS_ON_CUPBOARD 80
#define SM_PUT_OBJECT_ON_TABLE_RIGHT 90
#define SM_PUT_OBJECT_ON_TABLE_LEFT 100
#define SM_FINISH_TEST 110

#define GRAMMAR_POCKET_COMMANDS "grammars/gpsr.jsgf"
#define GRAMMAR_POCKET_CONFIRMATIONS "grammars/confirmation.jsgf"



std::vector<float> goal_vec(3);

std::string grammarCommandsID = "GPSRCommands";
std::string grammarConfirmationsID = "ConfirmationCommands";

//Locations
std::map<std::string, std::string> locations;//key = robot location, value = human location
void navigate_to_location(std::string location)
{
    std::cout << "Navigate to location" << std::endl;
    //arr_values.values = {0,0,0,0,1,1};
    //pub_digital.publish(arr_values);
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

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	class_image.clear();
	std::cout << "holi call" << std::endl;
    class_image.push_back(*msg);

}

int main(int argc, char** argv)
{
	std::cout << "INITIALIZING ACT_PLN STORING GROSERIES TEST    ..." << std::endl;
	ros::init(argc, argv, "act_pln");
	ros::NodeHandle n;
	FestinoHRI::setNodeHandle(&n);
	FestinoNavigation::setNodeHandle(&n);
	FestinoVision::setNodeHandle(&n);
	FestinoKnowledge::setNodeHandle(&n);
	ros::Rate loop(10);

	std::vector<sensor_msgs::Image> image_bb;

    ros::Subscriber class_image_sub = n.subscribe("/camera/rgb/image_color", 1000, imageCallback);
    ros::ServiceClient client_image_class = n.serviceClient<object_classification::Classify>("classify"); 
	object_classification::Classify srv;
 


	int nextState = 0;
	int maxAttempsGraspLeft = 0;
	int maxAttempsGraspRight = 0;
	int maxAttempsPlaceObj = 0;

	bool fail = false;
	bool success = false;
	bool stop=false;
	bool findObjCupboard = true;
	bool leftArm;

	//Speaker
    std::string voice;

	std::vector<std::string> idObjectGrasp;

	std::string lastRecoSpeech;

	geometry_msgs::Pose poseObj_1;
	geometry_msgs::Pose poseObj_2;

	std::vector<std::string> validCommands;
	validCommands.push_back("robot yes");


	while(ros::ok() && !fail && !success)
	{
		switch(nextState)
		{

			case SM_INIT:
			{
				std::cout << "----->  State machine: INIT" << std::endl;

				voice = "I'm ready for storing groseries test";
				FestinoHRI::say(voice, 5);

				voice = "I'm waiting for the start command";
				FestinoHRI::say(voice, 5);

				nextState = SM_WAIT_FOR_START_COMMAND;
			}
			break;



			case SM_WAIT_FOR_START_COMMAND:
			{
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: WAIT_FOR_START_COMMAND" << std::endl;

				//Enable speech recognition 
				FestinoHRI::enableSpeechRecognized(true);
	    		ros::Duration(2, 0).sleep();

				//Waiting for the operator to start
				if(!FestinoHRI::waitForSpecificSentence(validCommands, lastRecoSpeech, 15000)){
					voice = "Please repeat the command";
					FestinoHRI::say(voice, 5);
				}
				else
				{
				  if(lastRecoSpeech.find("robot yes") != std::string::npos){
				  	//Disable speech recognition 
	    			FestinoHRI::enableSpeechRecognized(false);
					ros::Duration(2, 0).sleep();

				    nextState = SM_NAVIGATION_TO_TABLE;
				  }
				  else
				    nextState = SM_WAIT_FOR_START_COMMAND;
				}
			}
			break;


			case SM_NAVIGATION_TO_TABLE:
			{
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: NAVIGATION_TO_TABLE" << std::endl;
				voice = "I am going to navigate to the side table";
				FestinoHRI::say(voice, 5);

				goal_vec = FestinoKnowledge::CoordenatesLocSrv("table_kitchen");
                std::cout <<"Coordenates of entrance_door"<<std::endl;
                std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                    if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2], 120000))
                        std::cout << "Cannot move to entrace_door" << std::endl;

				voice = "I arrived to side table";
				FestinoHRI::say(voice, 5);

				nextState = SM_FIND_OBJECTS_ON_TABLE;
			}
			break;



			case SM_FIND_OBJECTS_ON_TABLE:
			{
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: FIND_OBJECTS_ON_TABLE" << std::endl;
				voice = "I am going to search objects on the kitchen table";
				FestinoHRI::say(voice, 5);

				std::cout << "In Classification state" << std::endl;
				if(!class_image.empty()){
					srv.request.in.image_msgs = class_image;
		    		//image_bb = client_image_class.call(srv).debug_image;
		    		if(client_image_class.call(srv)){
		    			message_clas = srv.response;
		    			cv::Mat image = cv_bridge::toCvCopy(message_clas.debug_image.image_msgs.at(0), sensor_msgs::image_encodings::TYPE_8UC3)->image;
		    			cv::namedWindow("Imgclass", cv::WINDOW_NORMAL);
		    			cv::resizeWindow("Imgclass", 600, 600);
		    			cv::imshow("Imgclass", image);
		    			cv::waitKey(1);
		    		}
				}
			}
			break;

			/*
			case SM_SAVE_OBJECTS_PDF:
			{
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: SAVE_OBJECTS_PDF" << std::endl;
				FestinoTools::pdfImageExport("StoringGroseriesTest","/home/$USER/objs/");
				if(idObjectGrasp.size() > 1)
						nextState = SM_FINISH_TEST;
				else if(idObjectGrasp.size() > 0)
						nextState = SM_FINISH_TEST;
				else
					nextState = SM_FIND_OBJECTS_ON_TABLE;
			}
			break; */

			case SM_FINISH_TEST:
			{
				std::cout << "" << std::endl;
				std::cout << "" << std::endl;
				std::cout << "----->  State machine: FINISH_TEST" << std::endl;
				nextState = -1;
			}
			break;

			default:
			{
				fail = true;
				success = true;
			}
			break;

		}
		ros::spinOnce();
		loop.sleep();
	}

	return 0;
}