// Bibliotecas generales
#include <iostream>
#include <stdlib.h>
#include <vector>
#include <string>
#include <sstream>
#include <ros/ros.h>

// Bibliotecas de festino
#include <festino_tools/FestinoHRI.h>
#include <festino_tools/FestinoVision.h>
#include <festino_tools/FestinoNavigation.h>
#include <festino_tools/FestinoKnowledge.h>
#include <festino_tools/FestinoHardware.h>


#include <yolo_detect/StringArray.h>
// Estados
enum SMState
{
        SM_INIT,
        SM_WAIT_FOR_DOOR,
        SM_SAY_OPEN_DOOR,
        SM_NAVIGATE_TO_STORING_POINT,
        SM_FIND_OBJECTS,
        SM_PRE_GRASP,
        SM_TRY_TO_GRASPING_OBJECT,
        SM_DESCRIBE_OBJECTS,
        SM_NAVIGATE_TO_SHELF,
        SM_FINISH_TEST
};

// VARIABLES GLOBALES
// Banderas
bool success = false;

// Contadores

// Para el movimiento de la cabeza
float pitchAngle;

// Variables auxiliares de navegación
float robot_y, robot_x, robot_a;    
float gx_w, gy_w, gz_w, guest_z, host_z;    
float goalx, goaly, goala;
float dist_to_head;
float theta = 0, thetaToGoal = 0, angleHead = 0;
float pointingArmX, pointingArmY, pointingArmZ;
float pointingDirX, pointingDirY, pointingDirZ, pointingNormal;
float distanceArm = 0.6;

//Strings aux

std::stringstream ss;
std::stringstream ss2;
std::vector <std::string> objects;
//ros::NodeHandle nh;

SMState state = SM_INIT;

void callback_detect(const yolo_detect::StringArray::ConstPtr& msg)
{
        objects  = msg -> data;
}

// Variables de inicio
std::vector<float> goal_vec(3);

int main(int argc, char **argv)
{
    std::cout << "INITIALIZING ACT_PLN BY JOSHUA M... Aaaaaaaiudaaaaa" << std::endl;
    ros::init(argc, argv, "storing_groseries_test");
    ros::NodeHandle nh;
    ros::Rate rate(10);

    //FestinoTools
    FestinoHRI::setNodeHandle(&nh);
    FestinoNavigation::setNodeHandle(&nh);
    FestinoVision::setNodeHandle(&nh);
    FestinoKnowledge::setNodeHandle(&nh);
    FestinoHardware::setNodeHandle(&nh);

    FestinoHardware::init_kinect();
   sleep(2);
        //ESTO NO DEBERIA IR ASI XDDD
    ros::Subscriber detect;
    detect = nh.subscribe("/detected_objects", 1, &callback_detect);

    FestinoHRI::say(" ",1);

    while(ros::ok() && !success)
    {
        switch(state)
        {
                case SM_INIT:
                        std::cout << "SM_INIT --> Start Storing groseries :)" << std::endl;
                        FestinoHRI::say("I'm ready for storing groseries test", 3);
                        FestinoHardware::setHeadOrientation(0.0, 0.0);
                        state = SM_WAIT_FOR_DOOR;
                        break;

                case SM_WAIT_FOR_DOOR:
                        std::cout << "SM_WAIT_FOR_DOOR --> I'm waitig for the door is open" << std::endl;
                        state = FestinoNavigation::waitForDoor() ?  SM_NAVIGATE_TO_STORING_POINT : SM_NAVIGATE_TO_STORING_POINT;
                        break;

                case SM_SAY_OPEN_DOOR:
                        std::cout << "SM_SAY_OPEN_DOOR --> I'm saying to human that open the door" << std::endl;
                        FestinoHRI::say("Human, please open the door,", 3);
                        state = SM_WAIT_FOR_DOOR;
                        break;

                case SM_NAVIGATE_TO_STORING_POINT:
                        std::cout << "SM_NAVIGATE_TO_STORING_POINT --> I'm navigating to the storing point" << std::endl;
                        FestinoHRI::say("I will navigate to the kitchen,", 3);
                        goal_vec = FestinoKnowledge::CoordenatesLocSrv("table");
                        std::cout <<"Coordenates of storing table:"<<std::endl;
                        std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                        if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                            std::cout << "Cannot move to storing table" << std::endl; 
                        
                        FestinoHRI::say("I have arrived to storing table",3);   
                        state = SM_PRE_GRASP;
                        break;

                case SM_PRE_GRASP:
                        std::cout << "SM_PRE_GRASP --> I'm moving my arm to a pre-grasp position" << std::endl;
                        FestinoHardware::setArmPose("pre_grasp");
                        sleep(2);
                        FestinoHardware::setHeadOrientation(0.0, -0.6);
                        state = SM_FIND_OBJECTS;
                        break;

                case SM_FIND_OBJECTS:
                        std::cout << "SM_FIND_OBJECTS --> I'm find objects" << std::endl;
                        // FestinoHRI::say("Oh no! I can't use my arm, i'm sorry, i'm ",3);        
                        FestinoHardware::setArmPose("default");
                        sleep(2);
                        
                        state=SM_DESCRIBE_OBJECTS;
                        break;

                case SM_DESCRIBE_OBJECTS:
                        std::cout << "SM_DESCRIBE_OBJECTS --> I'm going to describe objects" << std::endl;
                        sleep(2);
                        FestinoHRI::say("I'm going to describe the objects", 3);
                        FestinoHardware::move_kinect(-30, 0.2);
                        sleep(4);
                        FestinoNavigation::moveDist(-0.2, 50);

                        ss << "Detected " << objects.size() << " items.\n";
                        
                        FestinoHRI::say(ss.str(), 3);
                        
                        // Iterate over each object, store details in ss2
                        for (size_t i = 0; i < objects.size(); ++i) {
                            std::string object = objects[i];
                        
                            // Find the position of ":"
                            size_t pos = object.find(":");
                            if (pos != std::string::npos)
                            {
                                std::string category = object.substr(0, pos);
                                std::string class_name = object.substr(pos + 2); // +2 to skip ": "

                                ss2 << "I detected " << class_name << " and it belongs to the category " << category;
                                FestinoHRI::say(ss2.str(), 6);

                            } else {
                                ss2 << "Invalid format in object: " << object << ".\n";
                            }
                        }

                        state = SM_NAVIGATE_TO_SHELF;
                        break;
                
                case SM_NAVIGATE_TO_SHELF:
                        FestinoHRI::say("I am going to the shelf", 3);
                        goal_vec = FestinoKnowledge::CoordenatesLocSrv("shelve");
                        std::cout <<"Coordenates of shelve:"<<std::endl;
                        std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                        if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                            std::cout << "Cannot move to shelve" << std::endl; 

                        FestinoHRI::say("I have arrived to shelve",3);  
                        FestinoNavigation::moveDist(0.1, 2000);
                        /*for (ver shelves empezando arriba y bajando 3 veces )
                                objects[1] = onjers[2] (categoria) -> categoria = categoriaShelf
                                decir que el n floor es cartegpria categoriaShelf

                                bajar cabeza*/
                        state = SM_FINISH_TEST;
                        break;
                
                case SM_FINISH_TEST:
                        std::cout << "SM_FINISH_TEST --> I finish the test: wuuuuu :)" << std::endl;
                        success = true;
                        FestinoHRI::say("I have finished the test... wuuu",3);  


                        break;
        }
    }
}