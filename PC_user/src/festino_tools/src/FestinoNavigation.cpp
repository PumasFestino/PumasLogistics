#include "festino_tools/FestinoNavigation.h"

bool FestinoNavigation::is_node_set;
actionlib_msgs::GoalStatus FestinoNavigation::_navigation_status;
actionlib_msgs::GoalStatus FestinoNavigation::_simple_move_status;
bool FestinoNavigation::_stop;

//Subscribers for stop signals
ros::Subscriber FestinoNavigation::subStop;
ros::Subscriber FestinoNavigation::subNavigationStop;

//Subscribers for checking goal-pose-reached signal
ros::Subscriber FestinoNavigation::subNavigationStatus;
ros::Subscriber FestinoNavigation::subSimpleMoveStatus;

//Publishers and subscribers for operating the simple_move node
ros::Publisher FestinoNavigation::pubSimpleMoveDist;
ros::Publisher FestinoNavigation::pubSimpleMoveDistAngle;
ros::Publisher FestinoNavigation::pubSimpleMoveLateral;
ros::ServiceClient FestinoNavigation::cltMoveBase;
ros::ServiceClient FestinoNavigation::cltAlingWithLine;

//Subscriber for laser scan
ros::Subscriber FestinoNavigation::subLaserScan;
sensor_msgs::LaserScan FestinoNavigation::_laserScan;

//Publishers and subscribers for mvn_pln
ros::Publisher FestinoNavigation::pubMvnPlnGetCloseLoc;
ros::Publisher FestinoNavigation::pubMvnPlnGetCloseXYA;
ros::Publisher FestinoNavigation::pubNavigationStop;
ros::Publisher FestinoNavigation::pubCmdVel;

//actionlib::SimpleActionClient<move_base::move_baseAction>* FestinoNavigation::actionMoveBase = nullptr;

//Publishers and subscribers for localization
tf::TransformListener* FestinoNavigation::tf_listener;

bool FestinoNavigation::setNodeHandle(ros::NodeHandle* nh)
{
    if(FestinoNavigation::is_node_set)
        return true;
    if(nh == 0)
        return false;

    std::cout << "FestinoNavigation.->Setting ros node..." << std::endl;
    subStop                = nh->subscribe("/stop"                    , 10, &FestinoNavigation::callbackStop);
    subNavigationStop      = nh->subscribe("/navigation/stop"         , 10, &FestinoNavigation::callbackNavigationStop);
    subNavigationStatus    = nh->subscribe("/navigation/status"       , 10, &FestinoNavigation::callbackNavigationStatus);
    subSimpleMoveStatus    = nh->subscribe("/simple_move/goal_reached", 10, &FestinoNavigation::callbackSimpleMoveStatus);
    subLaserScan           = nh->subscribe("/scan"                    , 10, &FestinoNavigation::callbackLaserScan);
    pubSimpleMoveDist      = nh->advertise<std_msgs::Float32          >("/simple_move/goal_dist", 10);
    pubSimpleMoveDistAngle = nh->advertise<std_msgs::Float32MultiArray>("/simple_move/goal_dist_angle", 10);
    pubSimpleMoveLateral   = nh->advertise<std_msgs::Float32          >("/simple_move/goal_dist_lateral", 10);
    pubMvnPlnGetCloseXYA   = nh->advertise<geometry_msgs::PoseStamped >("/move_base_simple/goal", 10);
    pubMvnPlnGetCloseLoc   = nh->advertise<std_msgs::String>           ("/navigation/mvn_pln/get_close_loc", 1);
    pubNavigationStop      = nh->advertise<std_msgs::Empty>            ("/navigation/stop", 10);
    pubCmdVel              = nh->advertise<geometry_msgs::Twist>       ("/cmd_vel", 10);
    cltMoveBase            = nh->serviceClient<simple_move::MoveBase>  ("/navigation/move_base");
    cltAlingWithLine       = nh->serviceClient<simple_move::LaserScanAling>  ("/navigation/align_with_line");

    tf_listener = new tf::TransformListener();
    is_node_set = true;
    _stop = false;

    _navigation_status.status  = actionlib_msgs::GoalStatus::PENDING;
    _simple_move_status.status = actionlib_msgs::GoalStatus::PENDING;
    return true;
}

//Methods for checking if goal position is reached.
bool FestinoNavigation::isLocalGoalReached()
{
    return _simple_move_status.status == actionlib_msgs::GoalStatus::SUCCEEDED;
}

bool FestinoNavigation::isGlobalGoalReached()
{
    return _navigation_status.status == actionlib_msgs::GoalStatus::SUCCEEDED;
}

bool FestinoNavigation::waitForLocalGoalReached(int timeOut_ms)
{
    FestinoNavigation::_stop = false;
    FestinoNavigation::_simple_move_status.status = actionlib_msgs::GoalStatus::PENDING;
    int attempts = timeOut_ms/100;
    ros::Rate loop(10);
    while(ros::ok() && FestinoNavigation::_simple_move_status.status == actionlib_msgs::GoalStatus::PENDING &&
          !FestinoNavigation::_stop && attempts-- >= 0)
    {
        loop.sleep();
        ros::spinOnce();
    }
    
    while(ros::ok() && FestinoNavigation::_simple_move_status.status == actionlib_msgs::GoalStatus::ACTIVE &&
          !FestinoNavigation::_stop && attempts-- >= 0)
    {
        loop.sleep();
        ros::spinOnce();
    }
    FestinoNavigation::_stop = false; //This flag is set True in the subscriber callback
    return FestinoNavigation::_simple_move_status.status == actionlib_msgs::GoalStatus::SUCCEEDED;
}

bool FestinoNavigation::waitForGlobalGoalReached(int timeOut_ms)
{
    FestinoNavigation::_stop = false;
    FestinoNavigation::_navigation_status.status = actionlib_msgs::GoalStatus::PENDING;
    int attempts = timeOut_ms/100;
    ros::Rate loop(10);
    while(ros::ok() && FestinoNavigation::_navigation_status.status == actionlib_msgs::GoalStatus::PENDING &&
          !FestinoNavigation::_stop && attempts-- >= 0)
    {
        loop.sleep();
        ros::spinOnce();
    }
    while(ros::ok() && FestinoNavigation::_navigation_status.status == actionlib_msgs::GoalStatus::ACTIVE &&
          !FestinoNavigation::_stop && attempts-- >= 0)
    {
        loop.sleep();
        ros::spinOnce();
    }
    FestinoNavigation::_stop = false;
    return FestinoNavigation::_navigation_status.status == actionlib_msgs::GoalStatus::SUCCEEDED;
}

//Methods for robot localization
void FestinoNavigation::getRobotPoseWrtMap(float& currentX, float& currentY, float& currentTheta)
{
    tf::StampedTransform transform;
    tf::Quaternion q;
    FestinoNavigation::tf_listener->lookupTransform("map", "base_link", ros::Time(0), transform);
    currentX = transform.getOrigin().x();
    currentY = transform.getOrigin().y();
    q = transform.getRotation();
    currentTheta = atan2((float)q.z(), (float)q.w()) * 2;
}

void FestinoNavigation::getRobotPoseWrtOdom(float& currentX, float& currentY, float& currentTheta)
{
    tf::StampedTransform transform;
    tf::Quaternion q;
    FestinoNavigation::tf_listener->lookupTransform("odom", "base_link", ros::Time(0), transform);
    currentX = transform.getOrigin().x();
    currentY = transform.getOrigin().y();
    q = transform.getRotation();
    currentTheta = atan2((float)q.z(), (float)q.w()) * 2;
}

//These methods use the simple_move node
void FestinoNavigation::startMoveDist(float distance)
{
    std_msgs::Float32 msg;
    msg.data = distance;
    FestinoNavigation::_simple_move_status.status = actionlib_msgs::GoalStatus::PENDING;
    pubSimpleMoveDist.publish(msg);
    ros::spinOnce();
    ros::Duration(0.0333333).sleep();
}

void FestinoNavigation::startMoveDistAngle(float distance, float angle)
{
    std_msgs::Float32MultiArray msg;
    msg.data.push_back(distance);
    msg.data.push_back(angle);
    FestinoNavigation::_simple_move_status.status = actionlib_msgs::GoalStatus::PENDING;
    pubSimpleMoveDistAngle.publish(msg);
    ros::spinOnce();
    ros::Duration(0.0333333).sleep();
}

void FestinoNavigation::startMoveLateral(float distance)
{
    std_msgs::Float32 msg;
    msg.data = distance;
    FestinoNavigation::_simple_move_status.status = actionlib_msgs::GoalStatus::PENDING;
    pubSimpleMoveLateral.publish(msg);
    ros::spinOnce();
    ros::Duration(0.0333333).sleep();
}


/*void FestinoNavigation::move_base(double x, double y, double theta, double time_out)
{   

    if (actionMoveBase == nullptr) {
        std::cerr << "FestinoNavigation.->Action client not initialized!" << std::endl;
        return;
    }
    
    move_base::move_baseGoal goal;
    goal.x = x;
    goal.y = y;
    goal.theta = theta;
    goal.time_out = time_out;

    actionMoveBase->sendGoal(goal);

    bool finished_before_timeout = actionMoveBase->waitForResult(ros::Duration(30.0));
    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = actionMoveBase->getState();
        std::cout<<"FestinoNavigation - move base already"<<std::endl;
    }
    else
    {

        std::cout<<"FestinoNavigation - move base did not finish before the time outy"<<std::endl;
    }

}*/

void FestinoNavigation::alingWithLine(bool enable)
{
    std::cout<< "FestinoNavigation.-> Aling with line" << std::endl;
    simple_move::LaserScanAling srv;
    srv.request.isAlingEnabled = enable;
    if(cltAlingWithLine.call(srv))
    {
        std::cout << "FestinoNavigation.-> Aling with line -- Success: " << srv.response.success << std:: endl;
    }
}

void FestinoNavigation::move_base(double x, double y, double theta, double time_out)
{   
    if (!cltMoveBase.isValid()) {
        std::cerr << "FestinoNavigation.->Service client not initialized!" << std::endl;
        return;
    }
    
    simple_move::MoveBase::Request req;
    simple_move::MoveBase::Response res;

    req.x = x;
    req.y = y;
    req.theta = theta;
    req.time_out = time_out;

    if (cltMoveBase.call(req, res))
    {
        if (res.success)
        {
            std::cout << "FestinoNavigation.-> move base already" << std::endl;
        }
        else
        {
            std::cout << "FestinoNavigation.-> move base failed. " << std::endl;
        }
    } 
    else
    {
        std::cout << "FestinoNavigation.-> move base service call failed" << std::endl;
    }
}


bool FestinoNavigation::moveDist(float distance, int timeOut_ms)
{
    FestinoNavigation::startMoveDist(distance);
    return FestinoNavigation::waitForLocalGoalReached(timeOut_ms);
}

bool FestinoNavigation::moveDistAngle(float distance, float angle, int timeOut_ms)
{
    FestinoNavigation::startMoveDistAngle(distance, angle);
    return FestinoNavigation::waitForLocalGoalReached(timeOut_ms);
}

bool FestinoNavigation::moveLateral(float distance, int timeOut_ms)
{
    FestinoNavigation::startMoveLateral(distance);
    return FestinoNavigation::waitForLocalGoalReached(timeOut_ms);
}

//These methods use the mvn_pln node.
void FestinoNavigation::startGetClose(float x, float y, float angle)
{
    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = "map";
    msg.pose.position.x = x;
    msg.pose.position.y = y;
    msg.pose.position.z = 0;
    msg.pose.orientation.x = 0;
    msg.pose.orientation.y = 0;
    msg.pose.orientation.z = sin(angle/2);
    msg.pose.orientation.w = cos(angle/2);
    FestinoNavigation::_navigation_status.status = actionlib_msgs::GoalStatus::PENDING;
    pubMvnPlnGetCloseXYA.publish(msg);
    ros::spinOnce();
    ros::Duration(0.0333333).sleep();
}

void FestinoNavigation::startGetClose(std::string location)
{
    std_msgs::String msg;
    msg.data = location;
    //FestinoNavigation::_isGlobalGoalReached = false;
    pubMvnPlnGetCloseLoc.publish(msg);
    ros::spinOnce();
}

bool FestinoNavigation::getClose(float x, float y, float angle, int timeOut_ms)
{
    FestinoNavigation::startGetClose(x,y,angle);
    return FestinoNavigation::waitForGlobalGoalReached(timeOut_ms);
}

bool FestinoNavigation::getClose(std::string location, int timeOut_ms)
{
    FestinoNavigation::startGetClose(location);
    return FestinoNavigation::waitForGlobalGoalReached(timeOut_ms);   
}

void FestinoNavigation::stopNavigation()
{
    std_msgs::Empty msg;
    pubNavigationStop.publish(msg);
}

bool FestinoNavigation::waitForDoor()
{
    int range=0,range_i=0,range_f=0,range_c=0,cont_laser=0;
    float laser_l=0;
    
    range = FestinoNavigation::_laserScan.ranges.size();
    
    //std::cout<<_laserScan.ranges.size()<<std::endl;
    
    range_c=range/2;
    range_i=range_c-(range/10);
    range_f=range_c+(range/10);
    //std::cout<<"Range Size: "<< range << "\n ";
    //std::cout<<"Range Central: "<< range_c << "\n ";
    //std::cout<<"Range Initial: "<< range_i << "\n ";
    //std::cout<<"Range Final: "<< range_f << "\n ";

    cont_laser=0;
    laser_l=0;
    for(int i = range_c - (range/10); i < range_c+(range/10); i++)
    {
        if(FestinoNavigation::_laserScan.ranges[i] > 0 && FestinoNavigation::_laserScan.ranges[i] < 4)
        { 
            laser_l = laser_l + FestinoNavigation::_laserScan.ranges[i]; 
            cont_laser++;
        }
    }
    //std::cout<<"Laser promedio: "<< laser_l/cont_laser << std::endl;    
    if(laser_l/cont_laser > 0.50)
    {
        std::cout<<"FestinoNavigation.-> door open"<<std::endl;
        return true;
    }
    else
    {
        std::cout<<"FestinoNavigation.-> door closed"<<std::endl;
        return false;
    }
}

//Callbacks for subscribers
void FestinoNavigation::callbackStop(const std_msgs::Empty::ConstPtr& msg)
{
    FestinoNavigation::_stop = true;
}

void FestinoNavigation::callbackNavigationStop(const std_msgs::Empty::ConstPtr& msg)
{
    FestinoNavigation::_stop = true;
}

void FestinoNavigation::callbackSimpleMoveStatus(const actionlib_msgs::GoalStatus::ConstPtr& msg)
{
    FestinoNavigation::_simple_move_status = *msg;
}

void FestinoNavigation::callbackNavigationStatus(const actionlib_msgs::GoalStatus::ConstPtr& msg)
{
    FestinoNavigation::_navigation_status = *msg;
}

void FestinoNavigation::callbackLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    FestinoNavigation::_laserScan = *msg;
}