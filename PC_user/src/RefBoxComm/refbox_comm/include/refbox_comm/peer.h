#include <ros/ros.h>
#include <protobuf_comm/peer.h>
#include <refbox_protobuf_msgs/OrderInfo.pb.h>
#include <refbox_protobuf_msgs/GameState.pb.h>
#include <refbox_protobuf_msgs/MachineInfo.pb.h>
#include <refbox_protobuf_msgs/RingInfo.pb.h>
#include <refbox_protobuf_msgs/BeaconSignal.pb.h>
#include <refbox_protobuf_msgs/Time.pb.h>
#include <refbox_protobuf_msgs/Team.pb.h>
#include <refbox_protobuf_msgs/MachineInstructions.pb.h>
#include <refbox_protobuf_msgs/Pose2D.pb.h>


#define TEAM_NAME "GRIPS"
#define CRYPTO_KEY "randomkey"

using namespace protobuf_comm;

class Peer
{
    using OrderInfo = llsf_msgs::OrderInfo;
    using MachineInfo = llsf_msgs::MachineInfo;
    using GameState = llsf_msgs::GameState;
    using RingInfo = llsf_msgs::RingInfo;
    using Order = llsf_msgs::Order;
    using BeaconSignal = llsf_msgs::BeaconSignal;
    using Time = llsf_msgs::Time;
    using Team = llsf_msgs::Team;
    using PrepareMachine = llsf_msgs::PrepareMachine;
    using PrepareInstructionRS = llsf_msgs::PrepareInstructionRS;
    using Machine = llsf_msgs::Machine;
    using Pose2D = llsf_msgs::Pose2D;

public:
    Peer(std::string host, int priv_recv_port, int pub_recv_port);
    ~Peer();

    void handleRefboxMessagePrivate(boost::asio::ip::udp::endpoint &endpoint, uint16_t comp_id, uint16_t msg_type, std::shared_ptr<google::protobuf::Message> msg);

    void handleRefboxMessagePublic(boost::asio::ip::udp::endpoint &endpoint, uint16_t comp_id, uint16_t msg_type, std::shared_ptr<google::protobuf::Message> msg);
    void handleRecvErrorPrivate(boost::asio::ip::udp::endpoint &endpoint, std::string msg);
    void handleSendErrorPrivate(std::string msg);
    // this method should notify the refbox of a robot, somehow it does not work
    // however the general way of sending like this should be correct
    void sendBeaconSignal(std::shared_ptr<BeaconSignal> msg);
    void sendPrepareMachineCs();

private:
    std::string m_host;
    int m_priv_recv_port;
    int m_pub_recv_port;
    unsigned long int m_sequence_nr_;
    MessageRegister *m_mr;
    std::shared_ptr<ProtobufBroadcastPeer> m_private_peer;
    std::shared_ptr<ProtobufBroadcastPeer> m_public_peer;
    std::string m_team_name;
    bool m_is_cyan;
    std::thread m_beacon_thread;
    bool m_running;
};
