#include <iostream>
#include <sstream>
#include <algorithm>

#include "ros/ros.h"
#include "ros/time.h"

#include <cmath>
#include <vector>
#include <map>
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
#include "festino_tools/FestinoHardware.h"

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

// Test results variables
bool fail = false;
bool success = false;

bool legs_found = false;
bool human_detector = false;
geometry_msgs::Pose human_coordinates;
geometry_msgs::PoseStamped tf_human_coordinates;

SMState state = SM_FOLLOW_OPERATOR;

// Variables globales
geometry_msgs::Point last_centroid;
bool new_centroid = false;
bool person = false;
double CENTER_THRESHOLD = 10.0; // Umbral en píxeles para considerar centrado
const int IMAGE_CENTER_X = 320;      // Centro horizontal de la imagen (ajustar según resolución)
const int IMAGE_CENTER_Y = 260;      // Centro vertical de la imagen (ajustar según resolución)

// Grammars
std::string namesGrammar("help_me_carry_names.json");
std::string interactionCommandsGrammar("help_me_carry_interaction_commands.json");
std::string navigationCommandsGrammar("help_me_carry_navigation_commands.json");

actionlib_msgs::GoalStatus simple_move_goal_status;

int simple_move_status_id = 0;
bool stop = false;
int counter = 0;

std::vector<float> goal_vec(3);


bool start  = true;


// Callback para actualizar el centroide
void centroidCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    last_centroid = *msg;
    new_centroid = true;
    // ROS_DEBUG("Centroide actualizado: x=%.2f, y=%.2f", msg->x, msg->y);
}

// Función para verificar si está centrado
bool isCenteredX()
{
    if(!new_centroid) return false;
    
    float error = last_centroid.x - IMAGE_CENTER_X;
    return (fabs(error) < CENTER_THRESHOLD);
}

bool isCenteredY()
{
    if(!new_centroid) return false;
    
    float error = last_centroid.y - IMAGE_CENTER_Y;
    return (fabs(error) < CENTER_THRESHOLD);
}

// Función para mover la camara (simulada)
void moveCamera(double dx, double dy)
{
    FestinoNavigation::move_base(0,0,dx*0.02,0.3);
}

// Función principal de centrado
void centerCamera()
{
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/vision/person_centroid", 1, centroidCallback);
    // ros::Subscriber sub = nh.subscribe("/vision/pose_2d", 1, pose_2dCallback);
    ros::Rate rate(10); // 10 Hz
    bool kinect = true;
    ROS_INFO("Iniciando rutina de centrado de camara...");

    while (ros::ok())
    {
        ros::spinOnce(); // Procesar callbacks

        if (new_centroid)
        {
            if (isCenteredX())
            {
                ROS_INFO("Persona centrada en la imagen!");
                if(person)
                {
                    person = true;
                }
                break; // Salir del ciclo cuando esté centrado
            }
            else
            {
                // Calcular movimiento necesario
                double dx = IMAGE_CENTER_X - last_centroid.x;
                double dy = IMAGE_CENTER_Y - last_centroid.y;
                
                // Mover la camara (proporcional al error)
                moveCamera(dx * 0.1, dy * 0.1); // Factor de ganancia 0.1
                if(person)
                {
                    person = false;
                    break;
                }
                if(kinect && !isCenteredY())
                {
                    kinect = FestinoHardware::move_kinect(dy*0.2, 0.3);
                    // sleep(0.2);
                }
            }
            
            new_centroid = false; // Resetear flag
        }
        else
        {
            ROS_WARN_THROTTLE(5, "No hay nadie...");
            break;
        }

        rate.sleep();
    }
}


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
    tf_human_coordinates.pose.position.x = tf_human_coordinates.pose.position.x * human_coordinates.position.x;
    tf_human_coordinates.pose.position.y = tf_human_coordinates.pose.position.y * human_coordinates.position.y;
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
    FestinoHardware::setNodeHandle(&n);

    ros::Publisher pub_digital = n.advertise<robotino_msgs::DigitalReadings>("/set_digital_values", 1000);
    ros::Subscriber sub_human_bool = n.subscribe("human_detector_bool", 1000, humanBoolCallback);
    ros::Subscriber sub_human_coordinates = n.subscribe("human_detector_coordinates", 1000, humanCoordinatesCallback);
    ros::Publisher pub_goal = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000); 
    ros::Subscriber sub_move_goal_status   = n.subscribe("/simple_move/goal_reached", 10, callback_simple_move_goal_status);

    ros::Rate loop(30);

    // Interacion variables
    std::string voice;
    std::string recogSpeech;
    std::string left_arm_pose;
    std::string chosenDirection;
    std::vector<std::string> pointingDirections;

    //Open Pose variables
    bool pointing_hand;
    bool human_detector_bool;

    //Robotino Lights
    robotino_msgs::DigitalReadings arr_values;
    arr_values.stamp.sec = 0;
    arr_values.stamp.nsec = 0;
    arr_values.values = {0,0,0,0,0,0};
    //This delay is necessary
    FestinoHRI::say(" ",3);
    FestinoHardware::init_kinect();
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
    std::map<int, std::tuple<float, float, float>> poses; // Curioso el float solito
    float currentX, currentY, currentTheta;
    float goalX, goalY, goalTheta;
    std::vector<float> goal_vec(3);

    // Variable para contar las poses
	int pose_counter = 0;


	while(ros::ok() && !fail && !success){
	    
        switch(state){
			case SM_FOLLOW_OPERATOR:
                
                std::cout << "State machine: SM_FOLLOW_OPERATOR" << std::endl;
                
                voice = "Hello my name is Festino. Say, festino yes, if we are ready";
                FestinoHRI::say(voice, 5);

                
                

                while (start) {
                    
                    recogSpeech = FestinoHRI::lastRecogSpeech(navigationCommandsGrammar);
                    
                    if(recogSpeech == "yes" || recogSpeech == "festino yes" ){
                        
                        start=false;

                        voice = "ok";
                        FestinoHRI::say(voice, 5);
                    }
                }



                FestinoHRI::enableHumanFollower(true);
                FestinoHRI::enableLegFinder(true);

                //Storage the ultimate pose with Map
                FestinoNavigation::getRobotPoseWrtMap(currentX, currentY, currentTheta);
                poses[pose_counter] = std::make_tuple(currentX, currentY, currentTheta);
                
                while (!stop) {

                    recogSpeech = FestinoHRI::lastRecogSpeech(navigationCommandsGrammar);

                    if(recogSpeech == "festino we arrived" || recogSpeech == "robot we arrived" || recogSpeech == "we arrived" || recogSpeech == "yes"){

                        std::cout << "We arrived" << std::endl;
                        stop = true;
                        state = SM_WAIT_CONF_CAR;
                        FestinoHardware::init_kinect();
                        sleep(3);
                    }
                

                    FestinoHRI::enableHumanFollower(false);
                
                    state = SM_WAIT_CONF_CAR;
                
            break;

            case SM_WAIT_CONF_CAR:
                 
                std::cout << "State machine: SM_WAIT_CONF_CAR" << std::endl;
                
            break;   

            }   

	    }

        ros::spinOnce();
	    loop.sleep();

	}

	return 0;
    
}