#include <ros/ros.h>
#include <protobuf_comm/peer.h>

#include <refbox_protobuf_msgs/BeaconSignal.pb.h>
#include <refbox_protobuf_msgs/Pose2D.pb.h>
#include <refbox_protobuf_msgs/Team.pb.h>
#include <refbox_protobuf_msgs/Time.pb.h>
#include <refbox_protobuf_msgs/MachineInstructions.pb.h>
//--------------------------------NAVIGATION CHALLENGE
#include <refbox_protobuf_msgs/NavigationChallenge.pb.h>
#include <refbox_protobuf_msgs/Zone.pb.h>
//--------------------------------NAVIGATION CHALLENGE

//--------------------------------EXPLORATION CHALLENGE
#include <refbox_protobuf_msgs/MachineReport.pb.h>
//--------------------------------EXPLORATION CHALLENGE

#include <refbox_protobuf_msgs/GameState.pb.h>

#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_listener.h>

#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

//Biblioteca para tokenizar
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>



//LOCALHOST
//#define HOST "localhost"
//ROBOCUP
//#define HOST "172.26.255.255"
//#define HOST "172.23.134.255"

#define HOST "192.168.1.255"

//#define TEAM_COLOR "MAGENTA"
//#define TEAM_COLOR "CYAN"
#define TEAM_NAME "Pumas"
#define CRYPTO_KEY "randomkey"
#define PUBLIC_PORT_S 4445
#define PUBLIC_PORT_R 4444
#define CYAN_PORT_S 4446
#define CYAN_PORT_R 4441
#define MAGENTA_PORT_S 4447
#define MAGENTA_PORT_R 4442

#include <iostream>
#include <typeinfo>

using namespace std;
using namespace protobuf_comm;

using PrepareInstructionCS = llsf_msgs::PrepareInstructionCS;
using PrepareInstructionBS = llsf_msgs::PrepareInstructionBS;

using PrepareMachine = llsf_msgs::PrepareMachine;
using GameState = llsf_msgs::GameState;

        bool m_is_cyan = false;

//--------------------------------ROBOT POSE
        float pose_x = 3.0f;
        float pose_y = 2.0f;
        float pose_ori = 45.0f;
        std::atomic<bool> pose_sem_th;
        std::atomic<bool> pose_sem_main;
//--------------------------------ROBOT POSE

//--------------------------------NAVIGATION CHALLENGE
        ros::Publisher pub_zone;
        using NavigationRoutes = llsf_msgs::NavigationRoutes;
        using Zone = llsf_msgs::Zone;
//--------------------------------NAVIGATION CHALLENGE

//--------------------------------
	ros::Publisher pub_time_over;
	bool beca_start = false;
//--------------------------------

    std::shared_ptr<ProtobufBroadcastPeer> m_public_peer;
    std::shared_ptr<ProtobufBroadcastPeer> m_private_peer;

void callback_MachineInstructions(const std_msgs::String::ConstPtr& machine_instruction){

    std::vector<std::string> tokens;
    tokens.clear();
    boost::algorithm::split(tokens, machine_instruction->data, boost::algorithm::is_any_of(","));

    for(std::string token : tokens){
        ROS_INFO_STREAM("Un token: " << token);
    }

    std::shared_ptr<PrepareMachine> prepare_instruction_message(new PrepareMachine());

    if(tokens[0] == "take"){
        PrepareInstructionCS new_machine_instruction_cs = prepare_instruction_message->instruction_cs();
        new_machine_instruction_cs.set_operation(llsf_msgs::CSOp::RETRIEVE_CAP);
    }
    
    if(tokens[0] == "put"){
        PrepareInstructionCS new_machine_instruction_cs = prepare_instruction_message->instruction_cs();
        new_machine_instruction_cs.set_operation(llsf_msgs::CSOp::MOUNT_CAP);
    }

    if(tokens[0] == "base"){
        PrepareInstructionBS new_machine_instruction_bs = prepare_instruction_message->instruction_bs();
        new_machine_instruction_bs.set_side(llsf_msgs::MachineSide::OUTPUT);
        if(tokens[1] == "BASE_SILVER"){
            new_machine_instruction_bs.set_color(llsf_msgs::BaseColor::BASE_SILVER);
        }
        if(tokens[1] == "BASE_BLACK"){
            new_machine_instruction_bs.set_color(llsf_msgs::BaseColor::BASE_BLACK);
        }
        if(tokens[1] == "BASE_CLEAR"){
            new_machine_instruction_bs.set_color(llsf_msgs::BaseColor::BASE_CLEAR);
        }
        if(tokens[1] == "BASE_RED"){
            new_machine_instruction_bs.set_color(llsf_msgs::BaseColor::BASE_RED);
        }
    }
    //TO DO color
    // if("CYAN" == TEAM_COLOR){
    //     machine_report_message->set_team_color(Team::CYAN);
    // } else {
    //     machine_report_message->set_team_color(Team::MAGENTA);
    // }
    
    //Hay que ver la forma para poder usar el public peer que ya está declarado
    // if(m_private_peer != nullptr){
    //     ROS_INFO_STREAM("NOT NULL PTR, SENDING PRIVATE");
    //     m_private_peer->send(PrepareMachine::COMP_ID, PrepareMachine::MSG_TYPE, prepare_instruction_message);
    // }
}

class Handler {

    
    using BeaconSignal = llsf_msgs::BeaconSignal;
    using Pose2D = llsf_msgs::Pose2D;
    using Team = llsf_msgs::Team;
    using Time = llsf_msgs::Time;

    using GameState = llsf_msgs::GameState;

//--------------------------------EXPLORATION CHALLENGE
    using MachineReport = llsf_msgs::MachineReport;
    using MachineReportEntry = llsf_msgs::MachineReportEntry;
//--------------------------------EXPLORATION CHALLENGE
    private:
        std::string m_host;
        int m_port_s;
        int m_port_r;

        MessageRegister *m_mr;

        unsigned long int m_sequence_nr_;

        std::string m_team_name = TEAM_NAME;
        //Default values 1
        std::string m_robot_name = "Festino";
        int m_robot_number = 1;
        //Default values 2


        bool m_running;
        bool team_color_set;

        std::thread m_beacon_thread;

        //--------------------------------NAVIGATION CHALLENGE
        std::map<Zone, std::string> zones_map;
        std::map<std::string, Zone> zones_map_str;
        //--------------------------------NAVIGATION CHALLENGE

        //--------------------------------EXPLORATION CHALLENGE

        //--------------------------------EXPLORATION CHALLENGE

    public:
        Handler(std::string host, int port_s, int port_r)
            : m_host(host), m_port_s(port_s)
            , m_port_r(port_r)
            , m_mr(new MessageRegister()) {
                //--------------------------------NAVIGATION CHALLENGE
                zones_map[Zone::C_Z11] = "C_Z11";
                zones_map[Zone::C_Z12] = "C_Z12";
                zones_map[Zone::C_Z13] = "C_Z13";
                zones_map[Zone::C_Z14] = "C_Z14";
                zones_map[Zone::C_Z15] = "C_Z15";
                zones_map[Zone::C_Z16] = "C_Z16";
                zones_map[Zone::C_Z17] = "C_Z17";
                zones_map[Zone::C_Z18] = "C_Z18";

                zones_map[Zone::C_Z21] = "C_Z21";
                zones_map[Zone::C_Z22] = "C_Z22";
                zones_map[Zone::C_Z23] = "C_Z23";
                zones_map[Zone::C_Z24] = "C_Z24";
                zones_map[Zone::C_Z25] = "C_Z25";
                zones_map[Zone::C_Z26] = "C_Z26";
                zones_map[Zone::C_Z27] = "C_Z27";
                zones_map[Zone::C_Z28] = "C_Z28";

                zones_map[Zone::C_Z31] = "C_Z31";
                zones_map[Zone::C_Z32] = "C_Z32";
                zones_map[Zone::C_Z33] = "C_Z33";
                zones_map[Zone::C_Z34] = "C_Z34";
                zones_map[Zone::C_Z35] = "C_Z35";
                zones_map[Zone::C_Z36] = "C_Z36";
                zones_map[Zone::C_Z37] = "C_Z37";
                zones_map[Zone::C_Z38] = "C_Z38";

                zones_map[Zone::C_Z41] = "C_Z41";
                zones_map[Zone::C_Z42] = "C_Z42";
                zones_map[Zone::C_Z43] = "C_Z43";
                zones_map[Zone::C_Z44] = "C_Z44";
                zones_map[Zone::C_Z45] = "C_Z45";
                zones_map[Zone::C_Z46] = "C_Z46";
                zones_map[Zone::C_Z47] = "C_Z47";
                zones_map[Zone::C_Z48] = "C_Z48";

                zones_map[Zone::C_Z52] = "C_Z52";
                zones_map[Zone::C_Z53] = "C_Z53";
                zones_map[Zone::C_Z54] = "C_Z54";
                zones_map[Zone::C_Z55] = "C_Z55";
                zones_map[Zone::C_Z56] = "C_Z56";
                zones_map[Zone::C_Z57] = "C_Z57";
                zones_map[Zone::C_Z58] = "C_Z58";

                zones_map[Zone::C_Z62] = "C_Z62";
                zones_map[Zone::C_Z63] = "C_Z63";
                zones_map[Zone::C_Z64] = "C_Z64";
                zones_map[Zone::C_Z65] = "C_Z65";
                zones_map[Zone::C_Z66] = "C_Z66";
                zones_map[Zone::C_Z67] = "C_Z67";
                zones_map[Zone::C_Z68] = "C_Z68";

                zones_map[Zone::C_Z72] = "C_Z72";
                zones_map[Zone::C_Z73] = "C_Z73";
                zones_map[Zone::C_Z74] = "C_Z74";
                zones_map[Zone::C_Z75] = "C_Z75";
                zones_map[Zone::C_Z76] = "C_Z76";
                zones_map[Zone::C_Z77] = "C_Z77";
                zones_map[Zone::C_Z78] = "C_Z78";



                zones_map[Zone::M_Z11] = "M_Z11";
                zones_map[Zone::M_Z12] = "M_Z12";
                zones_map[Zone::M_Z13] = "M_Z13";
                zones_map[Zone::M_Z14] = "M_Z14";
                zones_map[Zone::M_Z15] = "M_Z15";
                zones_map[Zone::M_Z16] = "M_Z16";
                zones_map[Zone::M_Z17] = "M_Z17";
                zones_map[Zone::M_Z18] = "M_Z18";

                zones_map[Zone::M_Z21] = "M_Z21";
                zones_map[Zone::M_Z22] = "M_Z22";
                zones_map[Zone::M_Z23] = "M_Z23";
                zones_map[Zone::M_Z24] = "M_Z24";
                zones_map[Zone::M_Z25] = "M_Z25";
                zones_map[Zone::M_Z26] = "M_Z26";
                zones_map[Zone::M_Z27] = "M_Z27";
                zones_map[Zone::M_Z28] = "M_Z28";

                zones_map[Zone::M_Z31] = "M_Z31";
                zones_map[Zone::M_Z32] = "M_Z32";
                zones_map[Zone::M_Z33] = "M_Z33";
                zones_map[Zone::M_Z34] = "M_Z34";
                zones_map[Zone::M_Z35] = "M_Z35";
                zones_map[Zone::M_Z36] = "M_Z36";
                zones_map[Zone::M_Z37] = "M_Z37";
                zones_map[Zone::M_Z38] = "M_Z38";

                zones_map[Zone::M_Z41] = "M_Z41";
                zones_map[Zone::M_Z42] = "M_Z42";
                zones_map[Zone::M_Z43] = "M_Z43";
                zones_map[Zone::M_Z44] = "M_Z44";
                zones_map[Zone::M_Z45] = "M_Z45";
                zones_map[Zone::M_Z46] = "M_Z46";
                zones_map[Zone::M_Z47] = "M_Z47";
                zones_map[Zone::M_Z48] = "M_Z48";

                zones_map[Zone::M_Z52] = "M_Z52";
                zones_map[Zone::M_Z53] = "M_Z53";
                zones_map[Zone::M_Z54] = "M_Z54";
                zones_map[Zone::M_Z55] = "M_Z55";
                zones_map[Zone::M_Z56] = "M_Z56";
                zones_map[Zone::M_Z57] = "M_Z57";
                zones_map[Zone::M_Z58] = "M_Z58";

                zones_map[Zone::M_Z62] = "M_Z62";
                zones_map[Zone::M_Z63] = "M_Z63";
                zones_map[Zone::M_Z64] = "M_Z64";
                zones_map[Zone::M_Z65] = "M_Z65";
                zones_map[Zone::M_Z66] = "M_Z66";
                zones_map[Zone::M_Z67] = "M_Z67";
                zones_map[Zone::M_Z68] = "M_Z68";

                zones_map[Zone::M_Z72] = "M_Z72";
                zones_map[Zone::M_Z73] = "M_Z73";
                zones_map[Zone::M_Z74] = "M_Z74";
                zones_map[Zone::M_Z75] = "M_Z75";
                zones_map[Zone::M_Z76] = "M_Z76";
                zones_map[Zone::M_Z77] = "M_Z77";
                zones_map[Zone::M_Z78] = "M_Z78";
                //--------------------------------NAVIGATION CHALLENGE

            if(ros::param::has("~robot_name")){
                ros::param::get("~robot_name", m_robot_name);
            }
            if(ros::param::has("~robot_number")){
                ros::param::get("~robot_number", m_robot_number);
            }

            ROS_INFO_STREAM("Sending NAME: " << m_robot_name << "\t Sending NUMBER: " << m_robot_number);

            m_mr->add_message_type<BeaconSignal>();
            m_mr->add_message_type<GameState>();
            m_mr->add_message_type<PrepareMachine>();

            m_public_peer =  std::make_shared<ProtobufBroadcastPeer>(m_host, m_port_s, m_port_r, m_mr);

            
            m_public_peer->signal_received().connect(
                boost::bind(&Handler::handleRefboxMessage, this, _1, _2, _3, _4)
            );
            

            m_sequence_nr_ = 0;

            m_team_name = TEAM_NAME;

            m_running = true;
            team_color_set = false;
            

            m_beacon_thread = std::thread(&Handler::sendBeaconSignal, this);
            //m_beacon_thread.join();
            m_beacon_thread.detach();        
        }



        ~Handler() {
            m_running = false;
        }
        

        void reportAMachine(const std_msgs::String::ConstPtr& machine_data){
            //ROS_INFO_STREAM("------A MACHINE REPORT SENT TO REFBOX--------- " << machine << " : ");
            std::shared_ptr<MachineReport> machine_report_message(new MachineReport());

                if(!team_color_set){
                    return;
                }

                if(m_is_cyan){
                    if(/*TODO message is NOT from a cyan machine*/ false) {
                        return;
                    }
                    machine_report_message->set_team_color(Team::CYAN);
                } else {
                    if(/*TODO message is NOT from a magenta machine*/ false){
                        return;
                    }
                    machine_report_message->set_team_color(Team::MAGENTA);
                }


            MachineReportEntry* new_machine_entry = machine_report_message->add_machines();

//            std::shared_ptr<MachineReportEntry> new_machine_entry(new MachineReportEntry());

            /*string my_msg;
            std_msgs::String el_msg;
            
                //std::stringstream ss;
                //ss << my_msg << count;
                el_msg.data = my_msg;//ss.str();
            

            ROS_INFO_STREAM(" el mensaje :O " << el_msg.data);*/
            std::vector<std::string> tokens;
            tokens.clear();
            boost::algorithm::split(tokens, machine_data->data, boost::algorithm::is_any_of(","));

                    for(std::string token : tokens){
                        ROS_INFO_STREAM("Un token: " << token);
                    }

            new_machine_entry->set_name(tokens[0]);
            new_machine_entry->set_type(tokens[1]);
            new_machine_entry->set_zone(zones_map_str[tokens[2]]);
            //new_machine_entry->set_rotation(uint32_t(std::stoul(tokens[3])));
//ROS_INFO_STREAM("El numero ° " << uint32_t(std::stoul(tokens[3])));
            
            //machine_report_message->add_machines();
           //machine_report_message->add_machines();

           //m_public_peer->send(MachineReport::COMP_ID, MachineReport::MSG_TYPE, machine_report_message);
           if(m_private_peer != nullptr){
            ROS_INFO_STREAM("NOT NULL PTR, SENDING PRIVATE");
            m_private_peer->send(MachineReport::COMP_ID, MachineReport::MSG_TYPE, machine_report_message);
           }
        }

        /*void handleRefboxMessagePrivate(boost::asio::ip::udp::endpoint &endpoint, uint16_t comp_id, uint16_t msg_type, std::shared_ptr<google::protobuf::Message> msg)
        {
            
        }*/


        void handleRefboxMessage(boost::asio::ip::udp::endpoint &endpoint, uint16_t comp_id, uint16_t msg_type, std::shared_ptr<google::protobuf::Message> msg) {
            std::shared_ptr<GameState> game_state;
            if ((game_state = std::dynamic_pointer_cast<GameState>(msg))) {
                //ROS_INFO_STREAM("----------------" << comp_id << " : " << msg_type);
                //ROS_INFO_STREAM(""<< game_state->ShortDebugString());
                if(!beca_start && game_state->phase() == llsf_msgs::GameState::PRODUCTION && game_state->game_time().sec() > 180){
			        beca_start = true;
			        std_msgs::Bool time_over;
        			time_over.data = true;
		        	pub_time_over.publish(time_over);
		        }

                if(!team_color_set){
                    auto cyan = game_state->team_cyan();
                    if (cyan == m_team_name){
                        m_is_cyan = true;
                        ROS_INFO_STREAM("COLOR SET CYAN ");
                    } else {
                        ROS_INFO_STREAM("COLOR SET MAGENTA ");
                    }
                    team_color_set = true;

                        ROS_INFO_STREAM("------          \n\n\n\n\nCRYPTO SETUP\n\n\n\n\n      --------- ");
                        if(m_is_cyan){
                            ROS_INFO_STREAM("------          \n\n\n\n\nCyan\n\n\n\n\n      --------- ");
                                m_private_peer =  std::make_shared<ProtobufBroadcastPeer>(m_host, CYAN_PORT_S, CYAN_PORT_R, m_mr,CRYPTO_KEY);
                        } else {
                                ROS_INFO_STREAM("------          \n\n\n\n\n Magenta SETUP\n\n\n\n\n      --------- ");
                                m_private_peer =  std::make_shared<ProtobufBroadcastPeer>(m_host, MAGENTA_PORT_S, MAGENTA_PORT_R, m_mr,CRYPTO_KEY);

                        }

                        m_private_peer->signal_received().connect(
                            boost::bind(&Handler::handleRefboxMessagePrivate, this, _1, _2, _3, _4)
                        );
                    
                }
                
            }
        }

        /*PRIVATE MESSAGES*/
        void handleRefboxMessagePrivate(boost::asio::ip::udp::endpoint &endpoint, uint16_t comp_id, uint16_t msg_type, std::shared_ptr<google::protobuf::Message> msg) {
            switch (msg_type) {
                case 250://NAVIGATION CHALLENGE / Navigation Routes
                {
                    std::shared_ptr<NavigationRoutes> navigation_routes = std::dynamic_pointer_cast<NavigationRoutes>(msg);
                    ROS_INFO_STREAM("------1 NAVIGATION CHALLENGE / Route--------- " << comp_id << " : " << msg_type);
                    ROS_INFO_STREAM(""<< navigation_routes->ShortDebugString());

                    
                    //navigation_routes->routes_size
                                        //ROS_INFO_STREAM(navigation_routes->routes().Get(0).route(0));

                    string my_msg = "";
                    for(int i = 0; i < navigation_routes->routes().Get(0).route_size(); i++){
                    // ROS_INFO_STREAM("UNA ZONA " << i);
                    //ROS_INFO_STREAM(navigation_routes->routes().Get(0).route(i));
                    my_msg.append(zones_map[navigation_routes->routes().Get(0).route(i)] + " ");
                    }


                    std_msgs::String el_msg;
   
                    //std::stringstream ss;
                    //ss << my_msg << count;
                    el_msg.data = my_msg;//ss.str();
   

                    ROS_INFO_STREAM(" el mensaje :O " << el_msg.data);
                    pub_zone.publish(el_msg);
  
                    ROS_INFO_STREAM("------2 NAVIGATION CHALLENGE / Route--------- ");

                    break;
                }   
            }
        }
        void handleRecvErrorPrivate(boost::asio::ip::udp::endpoint &endpoint, std::string msg) {
            ROS_ERROR_STREAM("Error receiving on private port : " << msg);
        }

        void handleSendErrorPrivate(std::string msg) {
            ROS_ERROR_STREAM("Error sending on private port : " << msg);
        }

        // This method should notifies the refbox of a robot
        void sendBeaconSignal() {
            while (m_running) {
                if(!team_color_set){
                    continue;
                }

                auto cur_time = ros::Time::now();
                std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> now =
                std::chrono::high_resolution_clock::now();
                std::chrono::seconds seconds = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch());
                std::chrono::nanoseconds nanoseconds = now.time_since_epoch() - seconds;

                std::shared_ptr<BeaconSignal> msg(new BeaconSignal());

                Time *time = msg->mutable_time();
                time->set_sec(static_cast<google::protobuf::int64>(seconds.count()));
                time->set_nsec(static_cast<google::protobuf::int64>(nanoseconds.count()));

                Pose2D *pose = msg->mutable_pose();
                    
                /* Create Semaphore */
                while(!pose_sem_th.load()){
                    std::this_thread::yield();
                }
                pose->set_x(pose_x);
                pose->set_y(pose_y);
                pose->set_ori(pose_ori);
                pose_sem_main.store(true);
                pose_sem_th.store(false);
                /* Create Semaphore */

                Time *posetimestamp = pose->mutable_timestamp();
                posetimestamp->set_sec(time->sec());
                posetimestamp->set_nsec(time->nsec());

                msg->set_seq(++m_sequence_nr_);
                msg->set_number(m_robot_number);
                msg->set_team_name(m_team_name);
                msg->set_peer_name(m_robot_name);

                msg->set_team_color(m_is_cyan ? Team::CYAN : Team::MAGENTA);

                ROS_INFO_STREAM("Beacon Signal: " << msg->ShortDebugString());

                m_public_peer->send(BeaconSignal::COMP_ID, BeaconSignal::MSG_TYPE, msg);
                std::this_thread::sleep_for(std::chrono::milliseconds(500));//ROS_INFO_STREAM("Sending: ");
            }
        }
};

Handler* p;

/*//DEBUG
float debug_value_x = 0.0f;
float debug_value_y = 1.0f;
float debug_value_ori = 2.0f;
*/
int main(int argc, char** argv) 
{

    ros::init(argc, argv, "refbox_comm_node");
    ros::NodeHandle n;

    pose_sem_main.store(true);
    pose_sem_th.store(false);

    //NAVIGATION CHALLENGE
    pub_zone = n.advertise<std_msgs::String>("/zone_msg", 1000);

    pub_time_over = n.advertise<std_msgs::Bool>("/time_over", 1000);

    p = new Handler(HOST, PUBLIC_PORT_S, PUBLIC_PORT_R);

    ros::Subscriber subMachineInstructions = n.subscribe("/machine_instruction_msg", 10, callback_MachineInstructions);
	

    ros::Rate r(10);
    while (ros::ok()) {
                       

        //Obtaining robot location
	    geometry_msgs::PoseStamped tf_robot_pose;
	    tf::TransformListener listener_rob;
        tf::StampedTransform transform_rob;


        //real
	//Para obtener la transformada bien primero va map y luego base_link
        try {
            listener_rob.waitForTransform("/Log_origin", "/base_link",   
                                   ros::Time(0), ros::Duration(1000.0));
            listener_rob.lookupTransform("/Log_origin","/base_link",   
                                   ros::Time(0), transform_rob);
        } catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        /* Create Semaphore */
        while(!pose_sem_main.load()){
            std::this_thread::yield();
        }
        /*//DEBUG
        debug_value_x += 0.01f;
        pose_x = debug_value_x;
        debug_value_y += 0.01f;
        pose_y = debug_value_y;
        debug_value_ori += 1.0f;
        pose_ori = debug_value_ori;
        */
        //real
        pose_x = transform_rob.getOrigin().x();
	    pose_y = transform_rob.getOrigin().y();
        //CHALLENGE TRACK, REMOVE WHEN IN MAIN TRACK
        if(m_is_cyan){
            pose_x -= 2;
        } else {
            pose_y += 2;
        }
        tfScalar yaw, pitch, roll;
        tf::Matrix3x3 mat(transform_rob.getRotation());
        mat.getEulerYPR(yaw, pitch, roll);
        pose_ori = yaw;
        pose_sem_th.store(true);
        pose_sem_main.store(false);
        /* Create Semaphore */

        std::cout << "Pose x: " << pose_x << " y: " << pose_y << " orientation: " << pose_ori << std::endl;

        ros::spinOnce();
	    r.sleep();
    }
    return 0;
}
