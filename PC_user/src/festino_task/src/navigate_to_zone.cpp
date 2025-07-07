/*  
    ----------------------------------------------------------------------
    ----- This action node navigates to a specified zone on the map. -----
    ----------------------------------------------------------------------
*/

/* ------------------- C++ Libraries --------------------*/
#include <cmath>

/* ------------------- ROS Libraries --------------------*/
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>

/* ------------------- Festino Tools ------------------- */
#include <festino_tools/FestinoHardware.h>
#include <festino_tools/FestinoHRI.h>
#include <festino_tools/FestinoNavigation.h>

//Parametro que multiplica al coseno 
#define param_x 0.9
//Parametro que multiplica al seno
#define param_y 0.9

//String that storage instruction tokens
std::vector<std::string> tokens;

//String que guarda la zona en la que estamos 
std::string zone_buffer = "M_Z01";

//String que guarda la seccion en la que estamos 
std::string sec_buffer = "indef";

std::string station_buffer = "NA";

//Angulo en radianes
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
    angulo_int = std::stoi(tokens[3]);
    
    float angulo_pose = 0.0f;
    angulo_pose = angulo_int*(M_PI/180);

    if(tokens[4] == "entrance" || tokens[4] == "platform" ){
	    //Si es entrada o platform tiene que mirar contrario a la orientacion del mapa
        angulo_int =- 180;
	    //Si es entrada o plataforma se le suman los senos y cosenos   
	    dir_sign = 1;                    
    }
    if(tokens[4] == "output"){
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
    std::cout << tokens[2] << std::endl;
    tf_target_zone.header.frame_id = "/map";
    tf_target_zone.pose = geometry_msgs::Pose();
    /*tf_target_zone.pose.position.x = 0.0;
    tf_target_zone.pose.position.y = 0.0;
    tf_target_zone.pose.position.z = 0.0;
    tf_target_zone.pose.orientation.x = 0.0;
    tf_target_zone.pose.orientation.y = 0.0;
    tf_target_zone.pose.orientation.z = 0.0;
    tf_target_zone.pose.orientation.w = 0.0;*/

    std::cout << "entró al transform zones" << std::endl;

    try{
        std::cout << "entró al try" << std::endl;
        listener.waitForTransform("/map", tokens.at(2), ros::Time(0), ros::Duration(100.0));
        listener.lookupTransform("/map", tokens.at(2), ros::Time(0), transform);
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

    std::cout << "salió del try name:" << tokens.at(2) << " tf x:" << tf_target_zone.pose.position.x << " y:" << tf_target_zone.pose.position.y << std::endl;
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

int main()
{

}
                