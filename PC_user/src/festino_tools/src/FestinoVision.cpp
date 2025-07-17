#include "festino_tools/FestinoVision.h"

bool FestinoVision::is_node_set = false;

//Pose Estimation
ros::Subscriber FestinoVision::subPointingHand;
std::string FestinoVision::_pointing_hand;

//Face Recog
ros::ServiceClient FestinoVision::cltFindPersons;
ros::ServiceClient FestinoVision::cltTrainPersons;
std::vector<std::string> FestinoVision::_nameRecog(5);

//Aruco Detect
ros::ServiceClient FestinoVision::cltArucoTf;

//QR Detect
ros::ServiceClient FestinoVision::cltQRSrv;

//Logistics Vision Tasks
ros::ServiceClient FestinoVision::cltCameraTask;
ros::Subscriber FestinoVision::subCentroidPiece;
float FestinoVision::_centroid_x;
float FestinoVision::_centroid_y;

ros::NodeHandle* FestinoVision::nh = nullptr;

//Aquí se configuran los nodos, el tipo de mensaje, buffer, el topico, etc.
bool FestinoVision::setNodeHandle(ros::NodeHandle* _nh)
{
    nh = _nh;

    if(FestinoVision::is_node_set)
        return true;
    if(nh == 0)
        return false;
    std::cout << "FestinoVision.->Setting ros node..." << std::endl;

    //Pose Estimation
    subPointingHand =   nh -> subscribe("/vision/pointing_direction", 1, &FestinoVision::callbackPointingHand);

    //face_recog
    cltFindPersons  =   nh -> serviceClient<vision_msgs::FaceRecogSrv>("/vision/recognize_face/names");
    cltTrainPersons =   nh -> serviceClient<vision_msgs::FaceTrainSrv>("/vision/training_face/name");
    
    //Aruco Detect
    cltArucoTf      =   nh -> serviceClient<img_proc::Tag_with_tf>("/vision/find_tag");

    //QR Detect
    cltQRSrv        =   nh -> serviceClient<img_proc::ReadQRCode>("/vision/read_qr_code");

    //Logistics camera tasks
    cltCameraTask    =   nh -> serviceClient<vision_logistics::RunTask>("/vision/run_camera_task");
    subCentroidPiece =   nh -> subscribe("/vision/lid_centroid", 1, &FestinoVision::callbackCentroid);


    //Pose Estimation controls
    nh  ->  setParam("/pose_2d_enabled", true);
    nh  ->  setParam("/pose_3d_enabled", true);
    nh  ->  setParam("/pointing_enabled", true);
    return true;
}

// ----- Pose Estimation ------ //
std::string FestinoVision::PointingHand()
{
    return _pointing_hand;
}

void FestinoVision::callbackPointingHand(const std_msgs::String::ConstPtr& msg)
{
    _pointing_hand = msg -> data; 
}

void FestinoVision::enablePoseEstimation(bool enabled)
{
    nh -> setParam("/pose_2d_enabled", enabled);
    nh -> setParam("/pose_3d_enabled", enabled);
    nh -> setParam("/pointing_enabled", enabled);

    std::cout << "Pose estimation system is: " << (enabled ? "activate" : "desactivado") << std::endl;
}

// ----- Face recog ----- //
std::vector<std::string> FestinoVision::enableRecogFacesName(bool flag)
{
    std::cout << "FestinoVision.->Recong person: ";
    vision_msgs::FaceRecogSrv srv;
    srv.request.is_face_recognition_enabled = flag;

    if (cltFindPersons.call(srv))
    {   
        for (int i = 0; i < srv.response.names.size(); i++)
        {
            //std::cout << "entre_3" <<std::endl;
            std::cout << srv.response.names[i] << " " << std::endl;
        }
        _nameRecog = srv.response.names;
        //std::cout << "lleno" <<std::endl;
        return _nameRecog;
    }

    else
    {
        std::vector<std::string> vector_vacio;
        //std::cout << "vacio" <<std::endl;
        vector_vacio = srv.response.names;
        return vector_vacio;
    }
   
}

bool FestinoVision::TrainingPerson(std::string person)
 {
     std::cout << "FestinoVision.->Train person: " << person << std::endl;
     vision_msgs::FaceTrainSrv srv;
     std_msgs::String name_msg;
     name_msg.data = person;
     srv.request.name = name_msg; 
     if (cltTrainPersons.call(srv))
     {
         std::cout << "Success " << srv.response.success << std::endl;
         std::cout << srv.response.message << std::endl;
         return true;
     }
     else
     {
         return false;
     }
 }

// ----- Aruco Detect ----- //
std::string FestinoVision::getArucoTF(bool flag)
{
    std::cout<< "FestinoVision.-> Detect Aruco Mark with TF" << std::endl;
    std::string mps_name = "";
    img_proc::Tag_with_tf srv;
    srv.request.is_find_tag_enabled = flag;
    if(cltArucoTf.call(srv))
    {
        std::cout << "Success: " << srv.response.success << std:: endl;
    }
    return srv.response.mps_name;
}

// ----- QR Detect ----- //
std::string FestinoVision::enableQRDetect(bool enabled)
{
    std::cout<< "FestinoVision.-> Detect QR Mark" << std::endl;
    img_proc::ReadQRCode srv;
    srv.request.enabled = enabled;
    if(cltQRSrv.call(srv))
    {
        std::cout << "Success: " << srv.response.success << std:: endl;
    }
    return srv.response.qr_data;
}

void FestinoVision::callbackCentroid(const geometry_msgs::Point::ConstPtr& msg)
{
    _centroid_x = msg -> x;
    _centroid_y = msg -> y;
}

 
std::pair<double, double> FestinoVision::findPiece()
{
   
    return std::make_pair(_centroid_x, _centroid_y);
    
}

float FestinoVision::findBand()
{
    std::cout<< "FestinoVision.-> Find Band" << std::endl;
    vision_logistics::RunTask srv;
    srv.request.task_name = "find_conveyor";
    if(cltCameraTask.call(srv))
    {
        std::cout << "Success: " << srv.response.success << std:: endl;
    }
    return srv.response.error_x;
    
}

float FestinoVision::centerBand()
{
    std::cout<< "FestinoVision.-> Center Band " << std::endl;
    vision_logistics::RunTask srv;
    srv.request.task_name = "center_conveyor";
    if(cltCameraTask.call(srv))
    {
        std::cout << "Success: " << srv.response.success << std:: endl;
    }
    return srv.response.error_x;
    
}

/*float FestinoVision::findPiece()
{
    std::cout<< "FestinoVision.-> Find Piece" << std::endl;
    vision_logistics::RunTask srv;
    srv.request.task_name = "find_piece";
    if(cltCameraTask.call(srv))
    {
        std::cout << "Success: " << srv.response.success << std:: endl;
    }
    return srv.response.error_y; 
}*/

float FestinoVision::findEndBand()
{
    std::cout<< "FestinoVision.-> Find End Band" << std::endl;
    vision_logistics::RunTask srv;
    srv.request.task_name = "find_end";
    if(cltCameraTask.call(srv))
    {
        std::cout << "Success: " << srv.response.success << std:: endl;
    }
    return srv.response.error_y;
    
}