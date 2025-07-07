#include <stdlib.h>
#include "std_msgs/Float32.h"
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseStamped.h"
#include "festino_tools/FestinoNavigation.h"
#include "festino_tools/FestinoKnowledge.h"
#include "festino_tools/FestinoVision.h"


geometry_msgs::PoseStamped position;
std::vector<float> goal_vec(3);
std::string location = "nuevito";
std::vector<std::string> namesRecog;



int main (int argc, char** argv)
{
    std::cout << "Moving arms and base practice"<< std::endl;
    ros::init(argc, argv, "moving_practice");
    ros::NodeHandle n("~");
    FestinoNavigation::setNodeHandle(&n);
    FestinoKnowledge::setNodeHandle(&n);
    FestinoVision::setNodeHandle(&n);

    ros::Rate loop(10);
    loop.sleep();
    while(ros::ok())
    {
        //std::cout << "entre" <<std::endl;
        //std::string name = "Alana";
        //FestinoVision::TrainingPerson(name);
        //std::cout << "Success " << std::endl;
        
        //namesRecog = FestinoVision::enableRecogFacesName(true);
        //std::cout << "entre_2" <<std::endl;
        //std::cout << namesRecog <<std::endl;
        /*for (int i = 0; i < namesRecog.size(); i++)
        {
            std::cout << "entre_3" <<std::endl;
            std::cout << namesRecog[i] << " ";
        }
        std::cout << std::endl;
        //float goal_x = (rand()%1000)/100.0;
        //float goal_y = (rand()%1000)/100.0-1.0;
        //float goal_a = (rand()%628)/100.0-3.14;
        /*
        goal_vec = FestinoKnowledge::CoordenatesLocSrv(location);
        std::cout <<"Moving to:"<< goal_vec[0] <<" "<< goal_vec[1] << " " << goal_vec[2] << std::endl;
        //std::cout <<"Moving to:"<< goal_x <<" "<< goal_y << " " << goal_a << std::endl;
        //if(FestinoNavigation::getClose(goal_x, goal_y, goal_a, 30000))
        if(FestinoNavigation::getClose(goal_vec[0], goal_vec[1], goal_vec[2], 30000))
            std::cout << "Global goal reached" << std::endl;
        else
            std::cout << "Cannnot navigate to goal pose" << std::endl;
        */
        
        std::cout << "Moving forward" << std::endl;
        FestinoNavigation::moveDist(0.5, 10000);
        std::cout << "Moving backwards" << std::endl;
        FestinoNavigation::moveDist(-0.5, 10000);
        //std::cout << "Moving left" << std::endl;
        //std::cout << "Turn left" << std::endl;
        //FestinoNavigation::moveDistAngle(0.0, 1.5707, 10000);
        //std::cout << "Moving right" << std::endl;
        //std::cout << "Turn right" << std::endl;
        //FestinoNavigation::moveDistAngle(0.0, -1.5707, 10000);
        

        loop.sleep();
        ros:: spinOnce();
    }
    
}