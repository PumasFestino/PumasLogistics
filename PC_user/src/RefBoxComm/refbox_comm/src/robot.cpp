/*#include "refbox_comm/comm_twr.h"

Robot::Robot() : m_mr(new MessageRegister())
{
    m_private_peer = std::make_shared<ProtobufStreamServer>(2016);
    m_private_peer->message_register().add_message_type<BeaconSignal>();
    m_private_peer->signal_received().connect(
        boost::bind(&Robot::handleMessage, this, _1, _2, _3, _4));

    m_running = true;
}

Robot::~Robot()
{
    m_running = false;
}

void Robot::handleMessage(unsigned int id, uint16_t comp_id, uint16_t msg_type, std::shared_ptr<google::protobuf::Message> msg)
{
    ROS_INFO_STREAM("Recv message on private");
    std::shared_ptr<BeaconSignal> beaconMsg;
    Peer pe("localhost", 4441, 4444);
    if ((beaconMsg = std::dynamic_pointer_cast<BeaconSignal>(msg)))
    {
        ROS_INFO_STREAM("received BeaconMsg: " << beaconMsg->ShortDebugString());
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        pe.sendBeaconSignal(beaconMsg);
    }
    
}*/