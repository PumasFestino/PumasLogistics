//-------------------------------------------------------------------------------//
//-----------------------PARAMETROS Y FUNCIONES PARA DEBUG-----------------------//
//-------------------------------------------------------------------------------//

/*std::string instructions[] = {"goto CS C_Z42 0 platform",
                              "goto CS C_Z42 0 entrance",
                              "goto CS C_Z42 0 output",

                             "goto CS M_Z61 0 entrance"};

int cont_instructions = 0;
int angulo = 315;
std::string zone;

//Función hardcodeada para hacer pruebas rápidas
void compute_coordinates(){
    float quat;

    //Descomentar para pruebas con los parámetros reales
    tf_target_zone.pose.position.x = tf_target_zone.pose.position.x + param_x*cos(angulo*(M_PI/180));
    tf_target_zone.pose.position.y = tf_target_zone.pose.position.y + param_y*sin(angulo*(M_PI/180)); 

    //Descomentar para prueba con la mesa del lab
    //tf_target_zone.pose.position.x = tf_target_zone.pose.position.x + 1;
    //tf_target_zone.pose.position.y = tf_target_zone.pose.position.y; 

    angulo = angulo - 180;                     
    angulo_rad = angulo*M_PI/180;

    std::cout << "el ángulo en grados es: " << angulo << std::endl;
    std::cout << "el ángulo en rad es: " << angulo*M_PI/180 << std::endl;

    tf::Quaternion myQuaternion;

    myQuaternion.setRPY(0,0,angulo*M_PI/180);

    myQuaternion=myQuaternion.normalize();

    tf_target_zone.pose.orientation.x = myQuaternion[0];
    tf_target_zone.pose.orientation.y = myQuaternion[1];
    tf_target_zone.pose.orientation.z = myQuaternion[2];
    tf_target_zone.pose.orientation.w = myQuaternion[3];

    std::cout << "Coordenadas modificadas:" << " tf x:" << tf_target_zone.pose.position.x << " y:" << tf_target_zone.pose.position.y << std::endl;
                    
}

//Funcion hardcodeada para hacer pruebas rapidas
void transform_zone()
{
    tf::TransformListener listener;
    tf::StampedTransform transform;

    //Descomentar cuando se use la zona marcada del lab
    //zone = "C_Z42";
    zone = "M_Z61";

    //Descomentar cuando se quiera ir a la mesa en medio del lab
    //zone = "M_Z13";

    //TF related stuff 
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
        listener.waitForTransform("/map",zone, ros::Time(0), ros::Duration(100.0));
        listener.lookupTransform("/map",zone, ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    //tf_target_zone.pose.position.x = -transform.getOrigin().x();
    //tf_target_zone.pose.position.y = -transform.getOrigin().y();

    tf_target_zone.pose.position.x = transform.getOrigin().x();
    tf_target_zone.pose.position.y = transform.getOrigin().y();
    tf_target_zone.pose.position.z = transform.getOrigin().z();
    tf_target_zone.pose.orientation.x = transform.getRotation().x();
    tf_target_zone.pose.orientation.y = transform.getRotation().y();
    tf_target_zone.pose.orientation.z = transform.getRotation().z();
    tf_target_zone.pose.orientation.w = transform.getRotation().w();

    //std::cout << "salió del try name:" << tokens.at(2) << " tf x:" << tf_target_zone.pose.position.x << " y:" << tf_target_zone.pose.position.y << std::endl;
    std::cout << "salió del try name:" << " tf x:" << tf_target_zone.pose.position.x << " y:" << tf_target_zone.pose.position.y << std::endl;
    std::cout << "Las rotaciones son" << " ori x:" << tf_target_zone.pose.orientation.x << std::endl;
    std::cout << "Las rotaciones son" << " ori y:" << tf_target_zone.pose.orientation.y << std::endl;
    std::cout << "Las rotaciones son" << " ori z:" << tf_target_zone.pose.orientation.z << std::endl;
    std::cout << "Las rotaciones son" << " ori w:" << tf_target_zone.pose.orientation.w << std::endl;
}*/

//Funcion que va recorriendo el arreglo de instrucciones falsas para hacer pruebas
/*void debug_instructions(std::string instruction)
{
    std::cout << "Entré a la función de instrucciones" <<std::endl; 
    //Tokenize instruction string
    std::cout << "La instrucción es: " <<  instruction <<std::endl; 
    tokens.clear();
    boost::algorithm::split(tokens, instruction, boost::algorithm::is_any_of(" "));
    
    if(tokens[0] == "goto"){
        state = SM_GO_TO;
        return;
    }

    if(tokens[0] == "take" || tokens[0] == "takep"){
        state = SM_TAKE;
        return;
    }

    if(tokens[0] == "drop" || tokens[0] == "dropp"){
        state = SM_DROP;
        return;
    }

    if(tokens[0] == "ask"){
        state = SM_ASK;
        return;
    }
}*/

//--------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------