//State Machine for main track
#include<iostream>
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

std::string instructions[] = {"goto CS C_Z56 270 platform",
                              "goto CS C_Z42 0 entrance",
                              "goto CS C_Z42 0 output",
                             "goto CS M_Z61 0 entrance"};

int cont_instructions = 0;

//Se puede cambiar, agregar o eliminar los estados
enum SMState {
    SM_INIT,
	SM_WAIT_FOR_INSTRUCTION,
	SM_MOVE,
    SM_ALIGN,
    SM_RETRIEVE,
    SM_DELIVER,
    SM_ASK,
    SM_FINAL_STATE
};

bool fail = false;
bool success = false;
SMState state = SM_INIT;

//-------------------------------------------------------------------------------//
//-----------------------------------PARAMETROS----------------------------------//
//-------------------------------------------------------------------------------//

//Parametro que multiplica al coseno 
#define param_x 0.9
//Parametro que multiplica al seno
#define param_y 0.9
//Parametro que modifica la distancia que avanza el robot para pegarse a la maquina
#define param_calib_dist 0.25
//Parametro que modifica el numero de pasos laterales para llegar a la plataforma (Si se usa cmd_vel)
#define steps_to_platform 4
//Parametro que modifica el numero de pasos laterales para llegar de la plataforma a la banda (Si se usa cmd_vel)
#define steps_to_band 6
//Parametro que modifica la distancia a recorrer para llegar a la plataforma (Si se usa funcion moveLateral)
#define dist_to_platform 0.2
//Umbral de distancia para tomar en cuenta las lecturas del hokuyo y acercarse a la estacion
#define dist_station_threshold 1
//Calibracion del angulo para voltear a ver a la estacion
#define calib_angle 0
//Delay para esperar a que el brazo termine de tomar o dejar pieza
#define arm_delay 30

//-------------------------------------------------------------------------------//
//-------------------------------------------------------------------------------//

// Request flag for new instructions to the planner
bool request = false;
// String to storage instruction tokens
std::vector<std::string> instructionTokens;

// Receive instructions from the planner
void request_instruction(){
    std::cout << "Request a new instruction" << std::endl;	

    if(FestinoCommunication::getInstruction(&instructionTokens)){
        std::cout << "The instruction is: " <<  instructionTokens[0] << std::endl;	
        request = true;

        if(instructionTokens[0] == "move"){
            state = SM_MOVE;
            return;
        }

        if(instructionTokens[0] == "retrieve"){
            state = SM_RETRIEVE;
            return;
        }

        if(instructionTokens[0] == "deliver"){
            state = SM_DELIVER;
            return;
        }

        if(instructionTokens[0] == "ask"){
            state = SM_ASK;
            return;
        }
    }
}

float angulo_rad;
//Variable que guarda el angulo en formato entero (se convierte de string a entero)
int angulo_int = 0;
//TF que va guardando la zona objetivo
geometry_msgs::PoseStamped tf_target_zone;


//Funcion que modifica el lugar al que llega el robot dependiendo de la orientacion de la estacion
void compute_coordinates(){
    //Signo por el que se multiplican los senos y cosenos 
    int dir_sign = 0;
    //Se convierte en angulo de string a entero
    angulo_int = std::stoi(instructionTokens[3]);
    
    float angulo_pose = 0.0f;
    angulo_pose = angulo_int*(M_PI/180);

    if(instructionTokens[4] == "entrance" || instructionTokens[4] == "platform" ){
	    //Si es entrada o platform tiene que mirar contrario a la orientacion del mapa
        angulo_int = angulo_int - 180;
	    //Si es entrada o plataforma se le suman los senos y cosenos   
	    dir_sign = 1;                    
    }
    if(instructionTokens[4] == "output"){
	    //Si es salida se le restan los senos y cosenos  
	    dir_sign = -1; 
    }

    //La coordenada original se modifica para que el robot llegue en la orientacion adecuada de la estacion
    tf_target_zone.pose.position.x = tf_target_zone.pose.position.x + dir_sign*param_x*cos(angulo_pose);
    tf_target_zone.pose.position.y = tf_target_zone.pose.position.y + dir_sign*param_y*sin(angulo_pose); 

    //Se convierte el angulo de degrees a radianes, esto se usa en el main para que el robot voltee a ver hacia la maquina
    angulo_rad = angulo_int*(M_PI/180);
}

//Funcion que obtiene las coordenadas de las zonas con el lookupTransform
void transform_zone()
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

//Callback que recibe las lecturas del hokuyo para cuando se necesite acercar a la maquina
void callbackLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    //Solo cuando flag_wall sea true se realizara todo esto
    if(flag_wall == true){
	    std::cout<< "Entro al if del flag_wall"<<std::endl;
	    laserScan = *msg;

	    int range=0,range_i=0,range_f=0,range_c=0,cont_laser=0;
	    float laser_l=0;
	    range=laserScan.ranges.size();

	    range_c=range/2;
	    range_i=range_c-(range/10);
	    range_f=range_c+(range/10);

	    cont_laser=0;
	    laser_l=0;
	    for(int i=range_c-(range/10); i < range_c+(range/10); i++)
	    {
            //Si la distancia de la lectura es mayor a cero y menor al umbral definido entonces se toma en cuenta para la suma
            if(laserScan.ranges[i] > 0 && laserScan.ranges[i] < dist_station_threshold)
            { 
                laser_l=laser_l+laserScan.ranges[i]; 
                //Se va contando el numero de lecturas que se suman para despues sacar un promedio de distancia
                cont_laser++;
            }
	    }
	    std::cout<< "El promedio de distancia es: " << laser_l/cont_laser << std::endl;

        //Si el promedio de distancia es mayor a 0.20 entonces avanza hacia adelante
	    if(laser_l/cont_laser > 0.20)
	    {
	        std::cout<< "Entro al if del callback del hokuyo"<<std::endl;
            flag_wall = false;
            //La distancia que avanza hacia adelante es el promedio de distancia menos un parametro de calibracion para que no choque
            move_to_machine = laser_l/cont_laser - param_calib_dist;
            FestinoNavigation::moveDistAngle(move_to_machine, 0, 1000);
	    }
    } 
}

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

    // Topics
    ros::Subscriber subLaserScan 	= n.subscribe("/scan", 1, callbackLaserScan);
    ros::Publisher pubMachineInst   = n.advertise<std_msgs::String>("/machine_instruction_msg", 1000);  // revisar con Sergio
    ros::Publisher pubManipulator   = n.advertise<std_msgs::Int32 >("manipulator/action", 1000);        // modificar por brazo de Miguel
    ros::Publisher pubVel           = n.advertise<geometry_msgs::PoseStamped>  ("/cmd_vel", 1000);      //QUITAAAAR

    // Services
    ros::ServiceClient aruco_client = n.serviceClient<img_proc::Find_tag_Srv>("/vision/find_tag/point_stamped");
    img_proc::Find_tag_Srv aruco_srv;

    // cmd_vel
    geometry_msgs::Twist vel;

    // Integer for the gripper node(Review)
    std_msgs::Int32 manipulator_var;

    // String with machine instruction
    std_msgs::String machine_instruction;

    // String to save the current zone 
    std::string zone_buffer = "M_Z01";

    // String to save the current section 
    std::string section_buffer = "indef";

    // String to save the current station
    std::string station_buffer = "NA";

	while(ros::ok() && !fail && !success){
	    switch(state){
			case SM_INIT:
	    		std::cout << "State machine: SM_INIT" << std::endl;	
	            std::cout << "I am ready for the main track challenge" << std::endl;
                
	    	    state = SM_WAIT_FOR_INSTRUCTION;
	    		break;

			case SM_WAIT_FOR_INSTRUCTION:
	    		std::cout << "State machine: SM_WAIT_FOR_INSTRUCTION" << std::endl;	

                //debug_instructions(instructions[cont_instructions]);
                //cont_instructions++;

                //Descomentar cuando se hagan pruebas con el Refbox
                //Ask for instruction once
                if(!request){
                    request_instruction();
                }
	    		break;

	    	case SM_MOVE:
	    		std::cout << "State machine: SM_MOVE" << std::endl;
                request = false;
                
                //Si estamos en la misma zona y queremos pasar de plataforma a entrada entonces solo muevete ahí mismo 
                if((zone_buffer == instructionTokens[2]) && (section_buffer == "platform")){
                    vel.linear.y = 2;
                    //Despues de alinearse con el Aruco se tiene que desplazar a la plataforma
                    for(int i=0; i<steps_to_band; i++){
                        pubVel.publish(vel);
                        ros::Duration(1, 0).sleep();
                    }

                    state = SM_WAIT_FOR_INSTRUCTION;
                }
                //Si se trata de la zona final hay que ir al centro de la misma
                else if (instructionTokens[1] == "ES"){
                    FestinoNavigation::moveDistAngle(-move_to_machine, 0, 1000);
                    transform_zone();
                    navigate_to_location(tf_target_zone);

                    state = SM_WAIT_FOR_INSTRUCTION;
                }
                else{

		            FestinoNavigation::moveDistAngle(-move_to_machine, 0, 1000);

                    // A partir de la instruccion se extrae la zona y con lookTransform se encuentran las coordenadas correspondientes
                    transform_zone();

                    //Dependiendo de la orientacion de la maquina y de si se quiere ir a la entrada o salida se obtienen las coordenadas
                    //tomando como base las coordenadas x,y de la zona, que representan el centro.
                    compute_coordinates();

                    //Navegacion Marco
                    navigate_to_location(tf_target_zone);

                    //Movimiento angular para que vea hacia la máquina
		            FestinoNavigation::moveDistAngle(0.0, angulo_rad-calib_angle, 1000);
                    state = SM_ALIGN;
                }
                
                //Variables que guardan la zona, la seccion y la estacion en la que estamos

                //Zona
                zone_buffer = instructionTokens[2];
                //Seccion
                section_buffer  = instructionTokens[4];
                //Estacion
		        station_buffer = instructionTokens[1];
                
	    		break;

            case SM_ALIGN:
                std::cout << "State machine: SM_ALIGN" << std::endl;
                request = false;

                aruco_srv.request.is_find_tag_enabled = true;
				aruco_client.call(aruco_srv);
				if(aruco_srv.response.success){

                    std::cout << "Ya se alineo en angulo" << std::endl;
                    aruco_srv.request.is_find_tag_enabled = false;
                    aruco_srv.request.is_aling_enabled = true;
		            aruco_client.call(aruco_srv);

                    if(aruco_srv.response.success){

                         if(instruction.at(4) == "platform"){
                             //Negativo a la derecha
                             vel.linear.y = -2;
			                 std::cout << "Publico en vel" << std::endl;
                             //Despues de alinearse con el Aruco se tiene que desplazar a la plataforma
                             for(int i=0; i<steps_to_platform; i++){
                                pubVel.publish(vel);
			                    ros::Duration(1, 0).sleep();
                             }
                         }
                         //Si estamos en la CS, ya sea entrada o salida que se mueva uno a la izquierda
                         else if((instructionTokens[1] == "CS") || (instructionTokens[1] == "BS" && instructionTokens[4] == "output")){
                            //Mueve uno a la izquierda de la banda
                            //Positivo a la izquierda
                            vel.linear.y = 2;
                            std::cout << "Publico uno en vel para quedar a la izquierda de la banda de salida de BS y cualquiera de CS" << std::endl;
                            pubVel.publish(vel);
                            ros::Duration(1, 0).sleep();
                         }
                         //si estamos en la BS y vamos a la entrance entonces que se mueva uno a la derecha
                         else if(instructionTokens[1] == "BS" && instructionTokens[4] == "entrance"){
                            //Mueve uno a la derecha de la banda
                            //Negativo a la derecha
                            vel.linear.y = -2;
                            std::cout << "Publico uno en vel para quedar a la derecha de la banda de entrada de BS" << std::endl;
                            pubVel.publish(vel);
			                ros::Duration(1, 0).sleep();
                         }
                         /*else if(instructionTokens[1] == "RS" && instructionTokens[4] == "output"){
                            //Mueve uno a la derecha de la banda
                            //Negativo a la derecha
                            vel.linear.y = -2;
                            std::cout << "Publico en vel para quedar a la derecha de la banda" << std::endl;
                            pubVel.publish(vel);
			                ros::Duration(1, 0).sleep();
                         }*/

                        std::cout << "Alineado!!!" << std::endl;
                                                     				
			            state = SM_WAIT_FOR_INSTRUCTION;	
			            flag_wall = true;
                        //state = SM_RETRIEVE;	
			            //state = SM_FINAL_STATE;
                    }
                    else{
                        //Se tiene que poner algo para que no repita todo 
                        std::cout << "NotFound" << std::endl;
                        state = SM_FINAL_STATE;
					    //state = SM_ALIGN;
                    }
				}
				else{
                    //Se tiene que poner algo para que no repita todo
					std::cout << "NotFound" << std::endl;
					state = SM_ALIGN;
				}
                break;

			case SM_RETRIEVE:
	    		std::cout << "State machine: SM_RETRIEVE" << std::endl;	
                request = false;
	            
                if(instructionTokens[0] == "takep"){
                    //Tomar de la plataforma
		            std::cout << "Estoy enviando un 1" << std::endl;
                    manipulator_var.data = 1;
                }
                else{
 		            if((station_buffer == "BS" && section_buffer == "output") || station_buffer == "RS" || station_buffer == "CS"){
                        std::cout << "Estoy enviando un 4" << std::endl;
                        //Tomar de la banda izq
                        manipulator_var.data = 4;
                    }
		            else{
                        std::cout << "Estoy enviando un 2" << std::endl;
                        //Tomar de la banda derecha
                        manipulator_var.data = 2;
		            }
                }

                pubManipulator.publish(manipulator_var);
                std::cout << "Estoy tomando" << std::endl;

                //Delay para que pueda tomar la pieza
	    		ros::Duration(arm_delay, 0).sleep();
                std::cout << "Ya pasaron los 30 seg" << std::endl;

                state = SM_WAIT_FOR_INSTRUCTION;
	    		break;
			case SM_DELIVER:
	    		std::cout << "State machine: SM_DELIVER" << std::endl;	
                request = false;	          
	    		
                if(instructionTokens[0] == "dropp"){
                    //Dejar en la plataforma
                    std::cout << "Estoy enviando un 3" << std::endl;
                    manipulator_var.data = 3;
                }
                else{
		            if(station_buffer == "CS" || station_buffer == "DS"){
		                std::cout << "Estoy enviando un 5" << std::endl;
			            //Dejar en la banda izq
			            manipulator_var.data = 5;
		            }
		            else if (station_buffer == "ES"){
                        std::cout << "Estoy enviando un 30" << std::endl;
			            //Dejar en el piso
			            manipulator_var.data = 30;
                    }
                    //Si es la entrada de un RS lo deja por la derecha 
                    else{
                        std::cout << "Estoy enviando un 0" << std::endl;
			            //Dejar en la banda derecha
			            manipulator_var.data = 0;
		            }
                }
                pubManipulator.publish(manipulator_var);

                //Delay para que pueda dejar la pieza
	    		ros::Duration(arm_delay, 0).sleep();
                std::cout << "Ya pasaron los 30 seg" << std::endl;

                state = SM_WAIT_FOR_INSTRUCTION;
	    		break;

			case SM_ASK:
	    		std::cout << "State machine: SM_ASK" << std::endl;	
                request = false;

                //Envía la concatenada la acción y el color de la base si se trata de la BS
                machine_instruction.data = instructionTokens[1]  + " " + instructionTokens[2];
	    		
                pubMachineInst.publish(machine_instruction);
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
