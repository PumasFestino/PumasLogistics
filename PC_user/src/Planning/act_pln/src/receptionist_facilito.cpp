//libraries for general functions
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


//Parameters for the test
#define MAX_FIND_PERSON_COUNT 1
#define MAX_FIND_PERSON_RESTART 0
#define MAX_FIND_PERSON_ATTEMPTS 1
#define MAX_CHECK_DOOR 2
#define TIMEOUT_SPEECH 10000
#define MIN_DELAY_AFTER_SAY 0
#define MAX_DELAY_AFTER_SAY 300
#define MAX_ATTEMPTS_SPEECH_INT 3
#define MAX_ATTEMPTS_SPEECH_RECO 3
#define MAX_ATTEMPTS_CONFIRMATION 2
#define MAX_ATTEMPTS_WAIT_CONFIRMATION 2
#define MAX_ATTEMPTS_MEMORIZING 2
#define MAX_FIND_SEAT_COUNT 4
#define TIMEOUT_MEMORIZING 3000


#define GRAMMAR_POCKET_COMMANDS "grammars/receptionist_commands.jsgf"
#define GRAMMAR_POCKET_DRINKS "grammars/order_drinks.jsgf"
#define GRAMMAR_POCKET_NAMES "grammars/people_names.jsgf"
    
//States for the state machine
enum STATE
{
	SM_INIT,
    SM_SAY_WAIT_FOR_DOOR,
    SM_WAIT_FOR_DOOR,
    SM_GOTO_RECEPTIONIST_POINT,
    SM_NAVIGATE_TO_ENTRANCE_DOOR,
    SM_NAVIGATE_TO_RECO_LOC,
    SM_SAY_OPEN_DOOR,
    SM_WAIT_FOR_OPEN_DOOR,
    SM_WAIT_FOR_PERSON_ENTRANCE,
    SM_INTRO_GUEST,
    SM_WAIT_FOR_PRESENTATION,
    SM_PRESENTATION_CONFIRM,
    SM_MEMORIZING_OPERATOR,
    SM_WAITING_FOR_MEMORIZING_OPERATOR,
    SM_GUIDE_TO_LOC,
    SM_FIND_TO_HOST_LOCATE,
    SM_FIND_TO_HOST_CHAIR_B,
    SM_FIND_TO_HOST_CHAIR_A,
    SM_FIND_TO_HOST_SOFA,
    SM_FIND_TO_GUEST,
    SM_INTRODUCING,
    SM_FIND_EMPTY_SEAT,
    SM_OFFER_EMPTY_SEAT,
    SM_FINISH_TEST
};
bool flag_door = true;
sensor_msgs::LaserScan laserScan;
//Strings aux
std::string nameUnknown;
std::string drinkUnknown;
std::string lastRecoSpeech;
std::string lastInteSpeech;
std::string guestLocation;
std::string auxNames;

std::string test("receptionist");
std::vector<float> goal_vec(3);

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
        if(laserScan.ranges[i] > 0 && laserScan.ranges[i] < 5)
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


int main(int argc, char **argv)
{
    std::cout << "INITIALIZING ACT_PLN BY PAREJITASOFT Inc..." << std::endl; //cout
	ros::init(argc, argv, "receptionist_test");
    ros::NodeHandle nh;
    ros::Rate rate(10);

    //Aux variables
    //Flags
    bool doorOpenFlag = false;
    bool opened = false;
    bool success = false;
    bool findPerson = false;
    bool findSeat = false;
    bool recogName = false;
    bool completeTrainig = false;
    
    
    std::vector<bool> memorizingOperators;
    std::vector<std::string> recogPersonAux;
    
    //Counters
    int findPersonCount = 0;
    int findPersonAttemps = 0;
    int findPersonRestart = 0;
    int findSeatCount = 0;
    int attemptsSpeechReco = 0;
    int attemptsSpeechInt = 0;
    int attemptsConfirmation = 0;
    int attemptsWaitConfirmation = 0;
    int attemptsMemorizing = 0;
    int attemptsCheckDoor = 0;
    int numGuests = 0;

    float pitchAngle;

    int genderRecog;
    int gender = 2;
    

    std::string param, typeOrder;
    std::string lastName, lastDrink;

    std::vector<std::string> findPersonDetect;
    std::vector<std::string> names;
    std::vector<std::string> drinks;
    std::string grammarCommandsID = "receptionisCommands";
    std::string grammarDrinksID = "receptionistDrinks";
    std::string grammarNamesID = "receptionistNames";
    std::string recogLoc = "kitchen";
    std::string seatPlace = "kitchen";
    std::string entranceLoc = "entrance_door";
    std::string hostDrink = "coke";

    names.push_back("john");
    drinks.push_back("coke");

    std::stringstream ss;
    std::stringstream ss2;

    //Nav aux variables
    float robot_y, robot_x, robot_a;    
    float gx_w, gy_w, gz_w, guest_z, host_z;    
    float goalx, goaly, goala;
    float dist_to_head;
    float theta = 0, thetaToGoal = 0, angleHead = 0;
    float pointingArmX, pointingArmY, pointingArmZ;
    float pointingDirX, pointingDirY, pointingDirZ, pointingNormal;
    float distanceArm = 0.6;
    bool usePointArmLeft = false;

    boost::posix_time::ptime prev;
	boost::posix_time::ptime curr;
    
    std::vector<std::string> tokens;

    STATE state = SM_INIT;//SM_SAY_WAIT_FOR_DOOR;

    //FestinoTools
    FestinoHRI::setNodeHandle(&nh);
    FestinoNavigation::setNodeHandle(&nh);
    FestinoVision::setNodeHandle(&nh);
    FestinoKnowledge::setNodeHandle(&nh);
    robotino_msgs::DigitalReadings arr_values;
    ros::Subscriber subLaserScan = nh.subscribe("/scan", 1, callbackLaserScan);

    ros::Publisher pub_digital = nh.advertise<robotino_msgs::DigitalReadings>("/set_digital_values", 1000);
    ros::Rate loop(10);
    
    arr_values.stamp.sec = 0;
    arr_values.stamp.nsec = 0;
    arr_values.values = {0,0,0,0,0,0};
    FestinoHRI::say(" ",2);
    recogName = true;

    while(ros::ok() && !success)
    {
    	switch(state)
    	{
    		case SM_INIT:
    			std::cout << test << ".-> State SM_INIT: Init the test." << std::endl;
                arr_values.values = {0,0,0,1,1,1};
                pub_digital.publish(arr_values);
                ros::Duration(0.5, 0).sleep();
                FestinoHRI::say("I'm ready for receptionist test",3);
                FestinoHRI::enableSpeechRecognized(false);
                
                state = SM_NAVIGATE_TO_ENTRANCE_DOOR;
                break;

            case SM_NAVIGATE_TO_ENTRANCE_DOOR:
                std::cout << test << ".-> State SM_NAVIGATE_TO_ENTRANCE_DOOR: Navigate to the entrance door." << std::endl;
                arr_values.values = {0,0,0,0,1,1};
                pub_digital.publish(arr_values);
                FestinoHRI::enableSpeechRecognized(false);

                FestinoHRI::say("I will navigate to the entrance door",4);
                goal_vec = FestinoKnowledge::CoordenatesLocSrv("entrance_door");
                std::cout <<"Coordenates of entrance_door"<<std::endl;
                std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                    if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2], 120000))
                        std::cout << "Cannot move to inspection point" << std::endl;
                
                FestinoHRI::say("I have reached the entrance door", 4);

                if(flag_door)
                //if(true)
                {
                    FestinoHRI::say("Hello human, please enter to the house",6);
                    state = SM_WAIT_FOR_PERSON_ENTRANCE;
                    findPersonCount = 0;
                    findPersonAttemps = 0;
                    findPersonRestart = 0;
                    attemptsCheckDoor = 0;
                }
                else
                {
                    state = SM_SAY_OPEN_DOOR;
                }
                break;

            case SM_SAY_OPEN_DOOR:
                std::cout << test << ".-> State SM_SAY_OPEN_DOOR: Saying open the door." << std::endl;
                arr_values.values = {0,0,0,0,0,1};
                pub_digital.publish(arr_values);
                FestinoHRI::say("Human, can you open the door please", 6);
                state = SM_WAIT_FOR_OPEN_DOOR;
                break;   

            case SM_WAIT_FOR_OPEN_DOOR:
                std::cout << test << "SM_WAIT_FOR_DOOR" << std::endl;
                arr_values.values = {0,0,0,1,0,1};
                pub_digital.publish(arr_values);
                state = SM_SAY_OPEN_DOOR;
                FestinoHRI::enableSpeechRecognized(false);

                if(flag_door)
                //if(true)
                {
                    FestinoHRI::say("Hello human, can you entrance in the house please and close the door", 6);
                    //JustinaVision::enableDetectObjsYOLO(true);
                    state = SM_WAIT_FOR_PERSON_ENTRANCE;
                    findPersonCount = 0;
                    findPersonAttemps = 0;
                    findPersonRestart = 0;
                }
                else
                {
                    state = SM_SAY_OPEN_DOOR;
                }
                break; 

            case SM_WAIT_FOR_PERSON_ENTRANCE:
                std::cout << test << ".-> State SM_WAIT_FOR_PERSON_ENTRANCE: Intro Guest." << std::endl;
                arr_values.values = {0,0,0,1,0,1};
                pub_digital.publish(arr_values);
                FestinoHRI::enableSpeechRecognized(false);
                if(findPersonAttemps < MAX_FIND_PERSON_COUNT)
                {
                    findPerson = true;
                    findPersonDetect = FestinoVision::enableRecogFacesName(true);
                    sleep(5);
                    if(findPersonDetect.size() == 1)
                    {
                        findPerson = true;
                        //recogPersonAux = FestinoVision::enableRecogFacesName(false);
                    }
                    else
                    {
                        if(findPersonDetect.size() == 0)
                        {
                            FestinoHRI::say("Human, i can't find you",3);
                            //recogPersonAux = FestinoVision::enableRecogFacesName(false);
                        }
                        if(findPersonDetect.size() > 1)
                           FestinoHRI::say("Human, i see more people",3); 

                    }

                    if(findPerson)
                        findPersonCount++;
                    
                    if(findPersonCount > MAX_FIND_PERSON_COUNT)
                    {
                        findPersonCount = 0;
                        findPersonAttemps = 0;
                        findPersonRestart = 0;

                        //recogPersonAux = FestinoVision::enableRecogFacesName(false);
                        recogPersonAux.clear();
                        state = SM_INTRO_GUEST;
                    }
                    else
                    {
                        if(findPersonRestart > MAX_FIND_PERSON_RESTART)
                        {
                            //recogPersonAux = FestinoVision::enableRecogFacesName(false);
                            recogPersonAux.clear();
                            findPersonCount = 0;
                            findPersonRestart = 0;
                            findPersonAttemps++;
                            FestinoHRI::say("Hello human, please enter to the house and close the door please",6);
                        }
                        else
                            findPersonRestart++;
                    }
                }
                else
                {
                    findPersonCount = 0;
                    findPersonAttemps = 0;
                    findPersonRestart = 0;
                    recogName = true;
                    //recogPersonAux = FestinoVision::enableRecogFacesName(false);
                    recogPersonAux.clear();
                    state = SM_INTRO_GUEST;
                }
                break;
    				
    		case SM_INTRO_GUEST:
    			std::cout << test << ".-> State SM_INTRO_GUEST: Intro Guest." << std::endl;
                recogPersonAux = FestinoVision::enableRecogFacesName(false);
                arr_values.values = {0,0,0,1,0,1};
                pub_digital.publish(arr_values);

                attemptsSpeechReco = 0;
                attemptsSpeechInt = 0;
                lastName = "unknown";
                lastDrink = "unknown";

                FestinoHRI::enableSpeechRecognized(false);

                if(recogName)
                {
                    FestinoHRI::say("Hello, my name is Festino, please tell me, what is your name",5);
                    FestinoHRI::loadGrammarSpeechRecognized(grammarNamesID, GRAMMAR_POCKET_NAMES);
                    FestinoHRI::enableSpeechRecognized(true);
                    arr_values.values = {0,0,0,1,0,0};
                    pub_digital.publish(arr_values);
                    sleep(2);
                    sleep(2);
                }
                else
                {
                    FestinoHRI::say("Please tell me, what is your favorite drink", 5);
                    FestinoHRI::loadGrammarSpeechRecognized(grammarDrinksID,GRAMMAR_POCKET_DRINKS);
                    FestinoHRI::enableSpeechRecognized(true);
                    arr_values.values = {0,0,0,1,0,0};
                    pub_digital.publish(arr_values);
                    sleep(2);
                    sleep(2);
                }
                
                attemptsConfirmation = 0;
                attemptsWaitConfirmation = 0;

                state = SM_WAIT_FOR_PRESENTATION;                
                break;
    				
    		case SM_WAIT_FOR_PRESENTATION:
    			std::cout << test << ".-> State SM_WAIT_FOR_PRESENTATION: Waiting for the names." << std::endl;
                arr_values.values = {0,0,0,0,1,0};
                pub_digital.publish(arr_values);

                tokens.clear();
                lastRecoSpeech = FestinoHRI::lastRecogSpeech();

                std::cout << "frase :"<<lastRecoSpeech<<std::endl;
                if(recogName)
                {
                    if(lastRecoSpeech != "")
                    {
                        if(param.compare(" ") != 0 || param.compare("") != 0)
                        {
                            ss.str("");
                            ss2.str("");
                            ss << "Ok, your name is ";
                            boost::algorithm::split(tokens,lastRecoSpeech, boost::algorithm::is_any_of(" "));
                            ss2.str("");
                            for(int i = 0; i < tokens.size(); i++)
                            {
                                std::cout<<"token ["<<i<<"]: "<<tokens[i]<<std::endl;
                                if(i < tokens.size() -1)
                                    ss2 << " ";
                            }
                            if(tokens[0].compare("i'm") != 0)
                            {
                                if(tokens[2].compare("is") == 0)
                                {
                                    ss << tokens[3];
                                    lastName = tokens[3];
                                }

                                else
                                {
                                    ss << tokens[2];
                                    lastName = tokens[2];
                                }
                            }
                            else
                            {
                                ss << tokens[1];
                                lastName = tokens[1];
                            }
                            //names.push_back(ss2.str());
                            ss << ", tell me robot yes or robot no";
                            
                            FestinoHRI::say(ss.str(), 6);
                            state = SM_PRESENTATION_CONFIRM;
                            break;
                        }
                    } 
                }
                else
                {

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
                }

                if(attemptsSpeechReco < MAX_ATTEMPTS_SPEECH_RECO)
                    {
                        FestinoHRI::enableSpeechRecognized(false);
                        if(recogName)
                        {
                            FestinoHRI::say("Please tell me what is your name", 4);
                            FestinoHRI::loadGrammarSpeechRecognized(grammarNamesID, GRAMMAR_POCKET_NAMES);
                            sleep(2);
                        }
                        else
                        {
                            FestinoHRI::say("Please tell me what is your favorite drink", 4);
                            FestinoHRI::loadGrammarSpeechRecognized(grammarDrinksID, GRAMMAR_POCKET_DRINKS);
                            sleep(2);
                        }
                        attemptsSpeechReco++;
                        FestinoHRI::enableSpeechRecognized(true);
                        arr_values.values = {0,0,0,1,0,0};
                        pub_digital.publish(arr_values);
                        sleep(2);
                        sleep(2);
                    }
                    else
                    {
                        FestinoHRI::enableSpeechRecognized(false);
                        FestinoHRI::clean_lastRecogSpeech();
                        arr_values.values = {0,0,0,1,0,0};
                        pub_digital.publish(arr_values);

                        attemptsSpeechReco = 0;
                        attemptsSpeechInt = 0;
                        if(recogName)
                        {
                            ss2.str("");
                                ss2 << "Sorry I did not understand you, you are an unknown person ";
                            FestinoHRI::say(ss2.str(), 7);
                            names.push_back("unknown");
                            recogName = false;
                            //FestinoHRI::enableSpeechRecognized(true);
                            state = SM_INTRO_GUEST;
                        }
                        else
                        {
                            ss2.str("");
                            //if(lastDrink.compare("unknown") == 0)
                            ss2 << "Sorry I did not understand you, your favorite drink is unknown";
                            FestinoHRI::say(ss2.str(), 7);
                            drinks.push_back("unknown");
                            state = SM_MEMORIZING_OPERATOR;
                        }
                    }
                break;

            case SM_PRESENTATION_CONFIRM:
                std::cout << test << ".-> State SM_PRESENTATION_CONFIRM. Wait for robot yes or robot no" << std::endl;
                attemptsSpeechReco = 0;
                attemptsSpeechInt = 0;

                FestinoHRI::loadGrammarSpeechRecognized(grammarCommandsID,GRAMMAR_POCKET_COMMANDS);
                FestinoHRI::enableSpeechRecognized(true);
                arr_values.values = {0,0,0,1,0,0};
                pub_digital.publish(arr_values);
                sleep(2);
                sleep(2);

                if (FestinoHRI::waitForSpecificSentence("robot yes",5000))
                {
                    FestinoHRI::enableSpeechRecognized(false);
                    if(recogName)
                    {
                        names.push_back(lastName);
                        ss2.str("");
                        ss2 << "Ok, your name is " << names[names.size() - 1];
                        arr_values.values = {0,0,0,0,1,0};
                        pub_digital.publish(arr_values);
                        FestinoHRI::say(ss2.str(), 6);
                        recogName = false;
                        state = SM_INTRO_GUEST;
                    }
                    else
                    {
                        drinks.push_back(lastDrink);
                        ss2.str("");
                        ss2 << "Ok, your favorite drink is " << drinks[drinks.size() - 1];
                        arr_values.values = {0,0,0,0,1,0};
                        pub_digital.publish(arr_values);
                        FestinoHRI::say(ss2.str(), 6);
                        attemptsMemorizing = 0;
                        state = SM_MEMORIZING_OPERATOR;
                    }
                }
                else
                {
                    if(attemptsConfirmation < MAX_ATTEMPTS_CONFIRMATION)
                    {
                        attemptsConfirmation++;
                        FestinoHRI::enableSpeechRecognized(false);
                        if(recogName)
                        {
                            arr_values.values = {0,0,0,0,1,0};
                            pub_digital.publish(arr_values);
                            FestinoHRI::say("Sorry I did not understand you, Please tell me what is your name", 7);
                            FestinoHRI::clean_lastRecogSpeech();
                            FestinoHRI::loadGrammarSpeechRecognized(grammarNamesID, GRAMMAR_POCKET_NAMES);
                            FestinoHRI::enableSpeechRecognized(true);
                            arr_values.values = {0,0,0,1,0,0};
                            pub_digital.publish(arr_values);
                            sleep(2);
                            sleep(2);
                            lastRecoSpeech = "";
                                
                            //lastRecoSpeech = FestinoHRI::lastRecogSpeech();
                        }
                        else
                        {
                            arr_values.values = {0,0,0,0,1,0};
                            pub_digital.publish(arr_values);
                            //drinks.erase(names.end() - 1);
                            FestinoHRI::clean_lastRecogSpeech();
                            FestinoHRI::say("Sorry I did not understand you, Please tell me what is your favorite drink", 7);
                            FestinoHRI::loadGrammarSpeechRecognized(grammarDrinksID, GRAMMAR_POCKET_DRINKS);
                            FestinoHRI::enableSpeechRecognized(true);
                            arr_values.values = {0,0,0,1,0,0};
                            pub_digital.publish(arr_values);
                            sleep(2);
                            sleep(2);
                        }
                        state = SM_WAIT_FOR_PRESENTATION;
                    }
                    else
                    {
                        FestinoHRI::enableSpeechRecognized(false);
                        if(recogName)
                        {
                            arr_values.values = {0,0,0,0,1,0};
                            pub_digital.publish(arr_values);
                            names.push_back("unknown");
                            ss2.str("");
                            ss2 << "Sorry, i don't understand you, your name is unknown";
                            FestinoHRI::say(ss2.str(), 6);
                            //FestinoHRI::enableSpeechRecognized(true);
                            arr_values.values = {0,0,0,0,1,0};
                            pub_digital.publish(arr_values);
                            recogName = false;
                            state = SM_INTRO_GUEST;
                        }
                        else
                        {
                            drinks.push_back("water");
                            ss2.str("");
                            ss2 << "Sorry, i don't understand you, your drink by default is water";
                            arr_values.values = {0,0,0,0,1,0};
                            pub_digital.publish(arr_values);
                            FestinoHRI::say(ss2.str(), 6);
                            attemptsMemorizing = 0;
                            state = SM_MEMORIZING_OPERATOR;
                        }
                    }
                }
                break;

            case SM_MEMORIZING_OPERATOR:
                std::cout << test << ".-> State SM_MEMORIZING_OPERATOR: Memorizing operator." << std::endl;
                arr_values.values = {0,0,0,1,0,1};
                pub_digital.publish(arr_values);
                if(attemptsMemorizing < MAX_ATTEMPTS_MEMORIZING)
                {
                    FestinoHRI::say("Human, please not move, and look at me. I'm memorizing your face", 6);
                    boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
                    state = SM_WAITING_FOR_MEMORIZING_OPERATOR;
                }
                else
                {
                    memorizingOperators.push_back(false);
                    state = SM_GUIDE_TO_LOC;
                }   
                break;

            case SM_WAITING_FOR_MEMORIZING_OPERATOR:
                std::cout << test << ".-> State SM_WAITING_FOR_MEMORIZING_OPERATOR: Waiting for Memorizing operator." << std::endl;
                arr_values.values = {0,0,0,1,0,1};
                pub_digital.publish(arr_values);
                state = SM_WAITING_FOR_MEMORIZING_OPERATOR;
                
                FestinoVision::TrainingPerson(names[names.size() - 1]);
                state = SM_GUIDE_TO_LOC;
                
                attemptsMemorizing++;
                break;

            case SM_GUIDE_TO_LOC:
                std::cout << test << ".-> State SM_GUIDING_TO_LOC: Guide to loc." << std::endl;
                arr_values.values = {0,0,0,0,1,1};
                pub_digital.publish(arr_values);
                FestinoHRI::say("Follow me to the living room",3);
                goal_vec = FestinoKnowledge::CoordenatesLocSrv("living_room");
                std::cout <<"Coordenates of sofa"<<std::endl;
                std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                attemptsMemorizing = 0;
                findSeatCount = 0;
                if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                    if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2], 120000))
                        std::cout << "Cannot move to inspection point" << std::endl;
                

                FestinoHRI::say("I'm going to find a empty seat for you, please wait", 5); 
                goal_vec = FestinoKnowledge::CoordenatesLocSrv("sofa");
                std::cout <<"Coordenates of sofa"<<std::endl;
                std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                attemptsMemorizing = 0;
                findSeatCount = 0;
                if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                    if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2], 120000))
                        std::cout << "Cannot move to inspection point" << std::endl;

                findSeat = false;
                state = SM_FIND_EMPTY_SEAT;
                break;

            case SM_FIND_EMPTY_SEAT:
                std::cout << test << ".-> State SM_FIND_EMPTY_SEAT: Finding empty seat" << std::endl;
                arr_values.values = {0,0,0,1,0,1};
                pub_digital.publish(arr_values);
                if(!findSeat)
                {
                    if(findSeatCount < MAX_FIND_SEAT_COUNT)
                    {
                        recogPersonAux.clear();
                        recogPersonAux = FestinoVision::enableRecogFacesName(true);
                        sleep(5);
                        if(recogPersonAux.size() >= 2)
                        {
                            ss.str("");
                            ss << "Sorry, in the sofa are " << recogPersonAux[0] << " and " << recogPersonAux[1];
                            findSeat = false;
                            recogPersonAux.clear();
                            recogPersonAux = FestinoVision::enableRecogFacesName(false);
                            FestinoHRI::say(ss.str(), 5);
                        }
                        else
                        {
                            recogPersonAux.clear();
                            recogPersonAux = FestinoVision::enableRecogFacesName(false);
                            guestLocation = "sofa";
                            findSeat = true;
                        }
                            
                        if(!findSeat)
                        {
                            findSeatCount++;
                            FestinoHRI::say("I'm going to find a empty seat for you again on the left chair", 5);
                            goal_vec = FestinoKnowledge::CoordenatesLocSrv("chair_a");
                            std::cout <<"Coordenates of entrance_door"<<std::endl;
                            std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                            if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                                 if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2], 120000))
                                    std::cout << "Cannot move to chairA" << std::endl;

                            recogPersonAux.clear();
                            recogPersonAux = FestinoVision::enableRecogFacesName(true);
                            sleep(5);

                            if(recogPersonAux.size() >= 1)
                            {
                                ss.str("");
                                ss << "Sorry in the left chair are " << recogPersonAux[0];
                                recogPersonAux.clear();
                                recogPersonAux = FestinoVision::enableRecogFacesName(false);
                                FestinoHRI::say(ss.str(), 3);
                                findSeat = false;
                            }   
                            else
                            {
                                recogPersonAux.clear();
                                recogPersonAux = FestinoVision::enableRecogFacesName(false);
                                guestLocation = "chair_a";
                                findSeat = true;
                            }
                            if(!findSeat)
                            {
                                FestinoHRI::say("I'm going to find a empty seat for you again on the right chair", 5);
                                goal_vec = FestinoKnowledge::CoordenatesLocSrv("chair_b");
                                std::cout <<"Coordenates of entrance_door"<<std::endl;
                                std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                                if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                                     if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2], 120000))
                                        std::cout << "Cannot move to chairB" << std::endl;

                                recogPersonAux.clear();
                                recogPersonAux = FestinoVision::enableRecogFacesName(true);
                                sleep(5);
                                if(recogPersonAux.size() >= 1)
                                {
                                    ss.str("");
                                    ss << "Sorry in the right place are " << recogPersonAux[0];
                                    recogPersonAux.clear();
                                    recogPersonAux = FestinoVision::enableRecogFacesName(false);
                                    FestinoHRI::say(ss.str(), 3);
                                    findSeat = false;
                                    //FestinoHRI::say("I'm going to find a empty seat for you again", 5);
                                    goal_vec = FestinoKnowledge::CoordenatesLocSrv("sofa");
                                    std::cout <<"Coordenates of entrance_door"<<std::endl;
                                    std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                                    if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                                        if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2], 120000))
                                            std::cout << "Cannot move to sofa" << std::endl;
                                }
                                else
                                {
                                    recogPersonAux.clear();
                                    recogPersonAux = FestinoVision::enableRecogFacesName(false);
                                    guestLocation = "chair_b";
                                    findSeat = true;
                                }
                            }
                            break;
                        }
                        else
                        {
                            FestinoHRI::say("Please wait", 3);
                            state = SM_OFFER_EMPTY_SEAT;    
                        }
                    
                    }
                    else
                        state = SM_OFFER_EMPTY_SEAT;
                }
                else
                    state = SM_OFFER_EMPTY_SEAT;
                
                break;                

            case SM_OFFER_EMPTY_SEAT:
                std::cout << test << ".-> State SM_OFFER_EMPTY_SEAT: Offer empty seat" << std::endl;
                arr_values.values = {0,0,0,0,1,0};
                pub_digital.publish(arr_values);
                ss.str("");
                ss << names[names.size() - 1] << ", could you sit in this place, please";

                FestinoHRI::say(ss.str(), 6);
                findPersonCount = 0;
                findPersonAttemps = 0;
                findPersonRestart = 0;
                state = SM_FIND_TO_GUEST;
                break;

            case SM_FIND_TO_GUEST:
                std::cout << test << ".-> State SM_FIND_TO_GUEST: Finding to ." << std::endl;
                arr_values.values = {0,0,0,1,0,1};
                pub_digital.publish(arr_values);
                
                
                findPerson = true;
                if(findPerson)
                {
                    findPersonCount = 0;
                    findPersonAttemps = 0;
                    findPersonRestart = 0;
                    state = SM_NAVIGATE_TO_RECO_LOC;
                }
                else
                {
                    if(findPersonAttemps > MAX_FIND_PERSON_ATTEMPTS)
                    {
                        findPersonCount = 0;
                        findPersonAttemps = 0;
                        findPersonRestart = 0;
                        state = SM_NAVIGATE_TO_RECO_LOC;
                    }
                    else
                        findPersonAttemps++;
                    ros::Duration(0.5).sleep(); 
                }
                break;

            case SM_NAVIGATE_TO_RECO_LOC:
                std::cout << test << ".-> State SM_NAVIGATE_TO_RECOG_LOC: Navigate to the host loc." << std::endl;
                arr_values.values = {0,0,0,1,0,1};
                pub_digital.publish(arr_values);
                findPersonCount = 0;
                findPersonAttemps = 0;
                findPersonRestart = 0;

                goal_vec = FestinoKnowledge::CoordenatesLocSrv("sofa");
                std::cout <<"Coordenates of John"<<std::endl;
                std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                    if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2], 120000))
                        std::cout << "Cannot move to john_location" << std::endl;
                state = SM_FIND_TO_HOST_LOCATE;
                break;

            case SM_FIND_TO_HOST_LOCATE:
                std::cout << test << ".-> State SM_FIND_TO_HOST: Finding to John in john_location." << std::endl;
                arr_values.values = {0,0,0,1,0,1};
                findPersonAttemps++;
                pub_digital.publish(arr_values);
                recogPersonAux.clear();
                recogPersonAux = FestinoVision::enableRecogFacesName(true);
                sleep(5);
                if(recogPersonAux.size() > 0)
                {
                    if(recogPersonAux[0] == "john" || recogPersonAux[1] == "john")//findPerson)
                    {
                        recogPersonAux = FestinoVision::enableRecogFacesName(false);
                        FestinoHRI::say("John, I found you", 3);
                        findPersonCount = 0;
                        findPersonAttemps = 0;
                        findPersonRestart = 0;
                        state = SM_INTRODUCING;
                    }
                    else
                    {
                        recogPersonAux = FestinoVision::enableRecogFacesName(false);
                        FestinoHRI::say("John, I'm going to find you in another site", 5);
                        goal_vec = FestinoKnowledge::CoordenatesLocSrv("chair_b");
                        std::cout <<"Coordenates of John"<<std::endl;
                        std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                        if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                            if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2], 120000))
                                std::cout << "Cannot move to john_location" << std::endl;
                        state = SM_FIND_TO_HOST_CHAIR_B;
                    }
                }
                else
                {
                    FestinoHRI::say("John, I'm going to find you in another site", 5);
                    goal_vec = FestinoKnowledge::CoordenatesLocSrv("chair_b");
                    std::cout <<"Coordenates of John"<<std::endl;
                    std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                    if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                        if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2], 120000))
                            std::cout << "Cannot move to john_location" << std::endl;
                    state = SM_FIND_TO_HOST_CHAIR_B;
                }

                break;

            case SM_FIND_TO_HOST_CHAIR_B:
                std::cout << test << ".-> State SM_FIND_TO_HOST: Finding to John in chairB." << std::endl;
                arr_values.values = {0,0,0,1,0,1};
                pub_digital.publish(arr_values);
                recogPersonAux.clear();
                recogPersonAux = FestinoVision::enableRecogFacesName(true);
                sleep(5);
                if(recogPersonAux.size() > 0)
                {
                    if(recogPersonAux[0] == "john")//findPerson)
                    {
                        recogPersonAux = FestinoVision::enableRecogFacesName(false);
                        FestinoHRI::say("John, I found you", 3);
                        findPersonCount = 0;
                        findPersonAttemps = 0;
                        findPersonRestart = 0;
                        state = SM_INTRODUCING;
                    }
                    else
                    {
                        recogPersonAux = FestinoVision::enableRecogFacesName(false);
                        FestinoHRI::say("John, I'm going to find you in another site", 5);
                        goal_vec = FestinoKnowledge::CoordenatesLocSrv("chair_a");
                        std::cout <<"Coordenates of John"<<std::endl;
                        std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                        if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                            if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2], 120000))
                                std::cout << "Cannot move to john_location" << std::endl;
                        state = SM_FIND_TO_HOST_CHAIR_A;

                    }
                }
                else
                {
                    FestinoHRI::say("John, I'm going to find you in another site", 5);
                    goal_vec = FestinoKnowledge::CoordenatesLocSrv("chair_a");
                    std::cout <<"Coordenates of John"<<std::endl;
                    std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                    if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                        if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2], 120000))
                            std::cout << "Cannot move to john_location" << std::endl;
                    state = SM_FIND_TO_HOST_CHAIR_A;
                }
                break;

        /*case SM_FIND_TO_HOST_CHAIR_A:
                std::cout << test << ".-> State SM_FIND_TO_HOST: Finding to John in chairA." << std::endl;
                arr_values.values = {0,0,0,1,0,1};
                pub_digital.publish(arr_values);
                recogPersonAux.clear();
                recogPersonAux = FestinoVision::enableRecogFacesName(true);
                sleep(5);
                if(recogPersonAux.size() > 0)
                {
                    if(recogPersonAux[0] == "john")//findPerson)
                    {
                        recogPersonAux = FestinoVision::enableRecogFacesName(false);
                        FestinoHRI::say("John, I found you", 3);
                        findPersonCount = 0;
                        findPersonAttemps = 0;
                        findPersonRestart = 0;
                        state = SM_INTRODUCING;
                    }
                    else
                    {
                        recogPersonAux = FestinoVision::enableRecogFacesName(false);
                        FestinoHRI::say("John, I'm going to find you in another site", 5);
                        goal_vec = FestinoKnowledge::CoordenatesLocSrv("sofa");
                        std::cout <<"Coordenates of John"<<std::endl;
                        std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                        if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                            if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2], 120000))
                                std::cout << "Cannot move to john_location" << std::endl;
                        state = SM_FIND_TO_HOST_SOFA;

                    }
                }
                else
                {
                    FestinoHRI::say("John, I'm going to find you in another site", 5);
                    goal_vec = FestinoKnowledge::CoordenatesLocSrv("sofa");
                    std::cout <<"Coordenates of John"<<std::endl;
                    std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                    if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                        if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2], 120000))
                            std::cout << "Cannot move to john_location" << std::endl;
                    state = SM_FIND_TO_HOST_SOFA;
                }
                break;*/

        case SM_FIND_TO_HOST_CHAIR_A:
                std::cout << test << ".-> State SM_FIND_TO_HOST: Finding to John in chairA." << std::endl;
                arr_values.values = {0,0,0,1,0,1};
                pub_digital.publish(arr_values);
                recogPersonAux.clear();
                recogPersonAux = FestinoVision::enableRecogFacesName(true);
                sleep(5);
                if(recogPersonAux.size() > 0)
                {
                    if(recogPersonAux[0] == "john")//findPerson)
                    {
                        recogPersonAux = FestinoVision::enableRecogFacesName(false);
                        FestinoHRI::say("John, I found you", 3);
                        findPersonCount = 0;
                        findPersonAttemps = 0;
                        findPersonRestart = 0;
                        state = SM_INTRODUCING;
                    }
                    else
                    {
                        if(findPersonAttemps > MAX_FIND_PERSON_ATTEMPTS)
                        {
                            findPersonCount = 0;
                            findPersonAttemps = 0;
                            findPersonRestart = 0;
                            state = SM_INTRODUCING;
                            FestinoHRI::say("John I did not find you, I wll navigate to your chair",3);
                            goal_vec = FestinoKnowledge::CoordenatesLocSrv("sofa");
                            std::cout <<"Coordenates of John"<<std::endl;
                            std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                            if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                                if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2], 120000))
                                    std::cout << "Cannot move to john_location" << std::endl;
                        }
                        else
                        {
                            FestinoHRI::say("John, I'm going to find you again",3);
                            goal_vec = FestinoKnowledge::CoordenatesLocSrv("sofa");
                            std::cout <<"Coordenates of John"<<std::endl;
                            std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                            if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                                if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2], 120000))
                                    std::cout << "Cannot move to john_location" << std::endl;
                            state = SM_FIND_TO_HOST_LOCATE;  
                        }

                    }
                }
                else
                {
                    findPersonCount = 0;
                    findPersonAttemps = 0;
                    findPersonRestart = 0;
                    state = SM_INTRODUCING;
                    FestinoHRI::say("John I did not find you, I wll navigate to your chair",3);
                    goal_vec = FestinoKnowledge::CoordenatesLocSrv("sofa");
                    std::cout <<"Coordenates of John"<<std::endl;
                    std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                    if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                        if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2], 120000))
                            std::cout << "Cannot move to john_location" << std::endl;
                }
                break;                

            case SM_INTRODUCING:
                std::cout << test << ".-> State SM_INTRODUCING: Introducing person to John." << std::endl;
                arr_values.values = {0,0,0,0,1,0};
                pub_digital.publish(arr_values);

                //**************************************************************
                //---Mover para presentar // Usar lo mismo para ofrecer silla---
                //**************************************************************

                ss.str("");
                if(gender == 1)
                    ss << "John you have a visitor, his name is " << names[names.size() - 1] << " and his favorite drink is " << drinks[drinks.size() - 1];
                else if(gender == 0)
                    ss << "John you have a visitor, her name is " << names[names.size() - 1] << " and her favorite drink is " << drinks[drinks.size() - 1];
                else
                    ss << "John, " << names[names.size() - 1] << " is your visitor, " << names[names.size() - 1] <<  " likes " << drinks[drinks.size() - 1];


                FestinoHRI::say(ss.str(), 5);
                ss.str("");
                goal_vec = FestinoKnowledge::CoordenatesLocSrv(guestLocation);
                std::cout <<"Coordenates of John"<<std::endl;
                std::cout <<"x = "<<goal_vec[0]<<"; y = "<<goal_vec[1]<<"; a = "<<goal_vec[2]<<std::endl;
                if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2],120000))
                    if(!FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2], 120000))
                        std::cout << "Cannot move to john_location" << std::endl;
                ss << names[names.size() - 1] << " he is John and his favorite drink is " << hostDrink << std::endl;
                FestinoHRI::say(ss.str(), 5);
                findPersonCount = 0;
                findPersonAttemps = 0;
                findPersonRestart = 0;
                recogName = true;
                if( numGuests++ < 1 )
                    state = SM_NAVIGATE_TO_ENTRANCE_DOOR;
                else
                    state = SM_FINISH_TEST;
                break;

            case SM_FINISH_TEST:
                std::cout << test << ".-> State SM_FINISH: Finish the test." << std::endl;
                arr_values.values = {0,0,0,1,1,1};
                pub_digital.publish(arr_values);
                FestinoHRI::say("I have finished the test", 6);
                success = true;
                
                for(int i = 0; i < names.size(); i++ )
                {
                    
                    std::cout << test << names[i] << std::endl;
                }
                break;  
                
    	}
    	rate.sleep();
    	ros::spinOnce();
    }
    return 1;
}
