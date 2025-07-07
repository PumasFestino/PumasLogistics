#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_listener.h>
#include "robotino_msgs/DigitalReadings.h"
#include <sstream>
#include "ros/time.h"
#include "actionlib_msgs/GoalStatus.h"


sensor_msgs::LaserScan laserScan;
actionlib_msgs::GoalStatus nav_status;
int simple_move_status_id = 0;
actionlib_msgs::GoalStatus simple_move_goal_status;
std_msgs::String locGoal;



ros::NodeHandle* nh;

//Se puede cambiar, agregar o eliminar los estados
enum SMState {
	SM_INIT,
	SM_CHECK_DOOR,
	SM_OPEN_DOOR,
	SM_CLOSE_DOOR,
    SM_LISTEN_LOC_TAKE,
    SM_NAVIGATE2_LOC,
    SM_PUB_LOC_TAKE,
    SM_WAIT_NAV_FINISH,
    SM_WAIT_NAV_DEL_FINISH,
    SM_LISTEN_LOC_DEL,
    SM_PUB_LOC_DEL,
    SM_GRASP_OBJCT,
    SM_DELIVER_OBJCT,
	SM_FINAL_STATE
};

SMState state = SM_INIT;

bool fail = false;
bool success = false;

bool flag_door = false;

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

    cont_laser=0;
    laser_l=0;
    for(int i=range_c-(range/10); i < range_c+(range/10); i++)
    {
        if(laserScan.ranges[i] > 0 && laserScan.ranges[i] < 4){ 
            laser_l=laser_l+laserScan.ranges[i]; 
            cont_laser++;
        }
    }
    if(laser_l/cont_laser > 0.5){
        flag_door = true;
    }
    else {
        flag_door = false;
    }
}


void callback_simple_move_goal_status(const actionlib_msgs::GoalStatus::ConstPtr& msg)
{
    simple_move_goal_status = *msg;
    std::stringstream ss;
    ss << msg->goal_id.id;
    ss >> simple_move_status_id;
    //std::cout << simple_move_goal_status.status << std::endl;

}


int main(int argc, char** argv)
{
    ros::Time::init();
    bool latch;
    std::string loc_name;
	std::cout << "INITIALIZING PLANNING NODE... " << std::endl;
    ros::init(argc, argv, "test_planning_2");
    ros::NodeHandle n;
    ros::Subscriber subLaserScan = n.subscribe("/scan", 1, callbackLaserScan);
    ros::Subscriber sub_move_goal_status   = n.subscribe("/simple_move/goal_reached", 10, callback_simple_move_goal_status);
    ros::Publisher pub_goal = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000); //, latch=True);
    ros::Publisher pub_digital = n.advertise<robotino_msgs::DigitalReadings>("/set_digital_values", 1000); //, latch=True);
    ros::Publisher pub_speaker = n.advertise<std_msgs::String>("/speak", 1000, latch = true);
    ros::Rate loop(30);
    geometry_msgs::PoseStamped loc_tf;
    robotino_msgs::DigitalReadings arr_values;
    std_msgs::String voice;
    std::string msg;
    loc_tf.header.frame_id = "/map";
    loc_tf.pose.position.x = 0.0;
    loc_tf.pose.position.y = 0.0;
    loc_tf.pose.position.z = 0.0;
    loc_tf.pose.orientation.x = 0.0;
    loc_tf.pose.orientation.y = 0.0;
    loc_tf.pose.orientation.z = 0.0;
    loc_tf.pose.orientation.w = 0.0;
    tf::TransformListener listener;
    tf::StampedTransform transform;
    arr_values.stamp.sec = 0;
    arr_values.stamp.nsec = 0;
    arr_values.values = {0,0,0,0,0,0};
    pub_digital.publish(arr_values);

    while(ros::ok() && !fail && !success){
        switch(state){

        	case SM_INIT:
        		//Init case
        		std::cout << "\nState machine: SM_INIT" << std::endl;	
                msg = "Ready for the navigation test";
                std::cout << msg << std::endl;
                voice.data = msg;
                pub_speaker.publish(voice);
                ros::Duration(2, 0).sleep();
                std::cout << "Opening Gripper" << std::endl;
                arr_values.values = {1,0,0,0,0,0};
                pub_digital.publish(arr_values);
                ros::Duration(1, 0).sleep();
        		state = SM_CHECK_DOOR;
        		break;

        	case SM_CHECK_DOOR:
        		//Checking open door case
        		std::cout << "\nState machine: SM_CHECK_DOOR" << std::endl;
                msg = "Checking Door State";
                std::cout << msg << std::endl;
                voice.data = msg;
                pub_speaker.publish(voice);
                ros::Duration(2, 0).sleep();
                
                sleep(1);
                flag_door = true;
        		if(flag_door == false){
        			state = SM_CLOSE_DOOR;	
        		}	
        		else{
        			state = SM_OPEN_DOOR;	
        		}
        		break;
            
            case SM_CLOSE_DOOR:{
                //door closed case
                std::cout << "\nState machine: SM_CLOSE_DOOR" << std::endl;
                msg = "Door closed. Can you open the door, please";
                std::cout << msg << std::endl;
                voice.data = msg;
                pub_speaker.publish(voice);
                ros::Duration(3, 0).sleep();
                state = SM_CHECK_DOOR;
                break;
            }


        	case SM_OPEN_DOOR:{
        		//Navigate case
        		std::cout << "\nState machine: SM_OPEN_DOOR" << std::endl;
                msg = "Door Open, Listening to navigation goal";
                std::cout << msg << std::endl;
                voice.data = msg;
                pub_speaker.publish(voice);
                ros::Duration(2, 0).sleep();
                state = SM_LISTEN_LOC_TAKE;
                break;
            }

            case SM_LISTEN_LOC_TAKE:{
                std::cout << "\nState machine: SM_LISTEN_LOC_TAKE" << std::endl;
                std::cout << "Waiting for Take goal" << std::endl;
                locGoal = *(ros::topic::waitForMessage<std_msgs::String>("zone_goal_take",ros::Duration(100)));
                loc_name = locGoal.data;
                std::cout << "Take goal Recieved" << std::endl;
                state = SM_PUB_LOC_TAKE;
                //state = SM_GRASP_OBJCT;
                break;
            }

            case SM_PUB_LOC_TAKE:{
                //pub goal location
                std::cout << "\nState machine: SM_PUB_LOC_TAKE" << std::endl;
                try{
                      listener.lookupTransform(loc_name, "/map",  
                                               ros::Time(0), transform);
                    }
                    catch (tf::TransformException ex){
                      ROS_ERROR("%s",ex.what());
                      ros::Duration(1.0).sleep();
                    }
                loc_tf.pose.position.x = -transform.getOrigin().x();
                loc_tf.pose.position.y = -transform.getOrigin().y();
                //loc_tf.pose.position.x = 4.90;
                //loc_tf.pose.position.y = -1.6;
                //MPS_BS
                msg = "Publishing Goal Location";
                std::cout << msg << std::endl;
                voice.data = msg;
                pub_speaker.publish(voice);
                ros::Duration(3, 0).sleep();
                msg = locGoal.data;
                voice.data = msg;
                pub_speaker.publish(voice);
                ros::Duration(2, 0).sleep();
                
                pub_goal.publish(loc_tf);
                std::cout << "Goal: "<< locGoal.data << "-> (" << loc_tf.pose.position.x << ", " << loc_tf.pose.position.y << ")" << std::endl;
                
                state = SM_WAIT_NAV_FINISH;
                //state = SM_GRASP_OBJCT;
                break;
            }

            case SM_WAIT_NAV_FINISH:{
                //wait for finished navigation
                std::cout << "\nState machine: SM_WAIT_NAV_FINISH" << std::endl;
                if(simple_move_goal_status.status == actionlib_msgs::GoalStatus::SUCCEEDED && simple_move_status_id == -1){
                    msg = "Goal location reached";
                    std::cout << msg << std::endl;
                    voice.data = msg;
                    pub_speaker.publish(voice);
                    ros::Duration(3, 0).sleep();
                    simple_move_goal_status.status = actionlib_msgs::GoalStatus::PENDING;
                    simple_move_status_id = 0;
                    state = SM_GRASP_OBJCT;
                }
                break;
            }

            case SM_GRASP_OBJCT:{
                std::cout << "\nState machine: SM_GRASP_OBJCT" << std::endl;
                std::cout << "Closing Gripper" << std::endl;
                arr_values.values = {0,1,0,0,1,0};
                pub_digital.publish(arr_values);
                ros::Duration(1, 0).sleep();
                //check if the sensor indicates that there is something inside the gripper and continue.
                state = SM_LISTEN_LOC_DEL;
                break;
            }

        case SM_LISTEN_LOC_DEL:{
                std::cout << "\nState machine: SM_LISTEN_LOC_DEL" << std::endl;
                std::cout << "Waiting for Deliver Goal" << std::endl;
                locGoal = *(ros::topic::waitForMessage<std_msgs::String>("zone_goal_del",ros::Duration(100)));
                loc_name = locGoal.data;
                std::cout << "Deliver goal Recieved" << std::endl;
                state = SM_PUB_LOC_DEL;
                //state = SM_DELIVER_OBJCT;
                break;
            }

            case SM_PUB_LOC_DEL:{
                std::cout << "\nState machine: SM_PUB_LOC_DEL" << std::endl;
                try{
                      listener.lookupTransform(loc_name, "/map",  
                                               ros::Time(0), transform);
                    }
                    catch (tf::TransformException ex){
                      ROS_ERROR("%s",ex.what());
                      ros::Duration(1.0).sleep();
                    }
                loc_tf.pose.position.x = -transform.getOrigin().x();
                loc_tf.pose.position.y = -transform.getOrigin().y();
                //loc_tf.pose.position.x = 4.9;
                //loc_tf.pose.position.y = -1.6;
                //MPS_BS
                msg = "Publishing Goal Location";
                std::cout << msg << std::endl;
                voice.data = msg;
                pub_speaker.publish(voice);
                ros::Duration(3, 0).sleep();
                msg = locGoal.data;
                voice.data = msg;
                pub_speaker.publish(voice);
                ros::Duration(2, 0).sleep();
                pub_goal.publish(loc_tf);
                std::cout << "Goal: "<< locGoal.data << "-> (" << loc_tf.pose.position.x << ", " << loc_tf.pose.position.y << ")" << std::endl;
                state = SM_WAIT_NAV_DEL_FINISH;
                //state = SM_DELIVER_OBJCT;
                break;
            }

            case SM_WAIT_NAV_DEL_FINISH:{
                //wait for finished navigation
                std::cout << "\nState machine: SM_WAIT_NAV_FINISH" << std::endl;
                if(simple_move_goal_status.status == actionlib_msgs::GoalStatus::SUCCEEDED && simple_move_status_id == -1){
                    msg = "Goal location reached";
                    std::cout << msg << std::endl;
                    voice.data = msg;
                    pub_speaker.publish(voice);
                    ros::Duration(3, 0).sleep();
                    state = SM_DELIVER_OBJCT;
                }
                break;
            }

            case SM_DELIVER_OBJCT:{
                std::cout << "\nState machine: SM_DELIVER_OBJCT" << std::endl;
                std::cout << "Openning Gripper" << std::endl;
                arr_values.values = {1,0,0,0,1,0};
                pub_digital.publish(arr_values);
                ros::Duration(2, 0).sleep();
                //check if the sensor indicates that there is something inside the gripper and continue.
                state = SM_FINAL_STATE;
                break;
            }


        	case SM_FINAL_STATE:{
        		//Navigate case
        		std::cout << "\nState machine: SM_FINAL_STATE" << std::endl;	
                msg =  "I have finished test";
                std::cout << msg << std::endl;
                voice.data = msg;
                pub_speaker.publish(voice);
                ros::Duration(2, 0).sleep();
                arr_values.values = {0,0,0,0,0,0};
                pub_digital.publish(arr_values);
                ros::Duration(1, 0).sleep();
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