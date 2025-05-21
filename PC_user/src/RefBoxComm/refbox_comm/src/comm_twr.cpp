#include "refbox_comm/comm_twr.h"

Peer::Peer(std::string host, int priv_recv_port, int pub_recv_port)
    : m_host(host), m_priv_recv_port(priv_recv_port), m_pub_recv_port(pub_recv_port),
      m_mr(new MessageRegister())
{
    m_mr->add_message_type<OrderInfo>();
    m_mr->add_message_type<GameState>();
    m_mr->add_message_type<MachineInfo>();
    m_mr->add_message_type<RingInfo>();
    m_mr->add_message_type<BeaconSignal>();
    m_mr->add_message_type<PrepareMachine>();
    //m_mr->add_message_type<Pose2D>();

    m_private_peer = std::make_shared<ProtobufBroadcastPeer>(m_host, 4446, m_priv_recv_port, m_mr, CRYPTO_KEY);
    m_private_peer->signal_received().connect(
        boost::bind(&Peer::handleRefboxMessagePrivate, this, _1, _2, _3, _4));
    m_private_peer->signal_recv_error().connect(
        boost::bind(&Peer::handleRecvErrorPrivate, this, _1, _2));
    m_private_peer->signal_send_error().connect(
        boost::bind(&Peer::handleSendErrorPrivate, this, _1));

    m_public_peer = std::make_shared<ProtobufBroadcastPeer>(m_host, 4445, m_pub_recv_port,  m_mr);
    m_public_peer->signal_received().connect(
        boost::bind(&Peer::handleRefboxMessagePublic, this, _1, _2, _3, _4));

    m_team_name = "";
    m_is_cyan = false;
    m_running = true;
    m_sequence_nr_ = 0;
    // m_beacon_thread = std::thread(&Peer::sendBeaconSignal, this);
}

Peer::~Peer()
{
    m_running = false;
}

void Peer::handleRefboxMessagePrivate(boost::asio::ip::udp::endpoint &endpoint, uint16_t comp_id, uint16_t msg_type, std::shared_ptr<google::protobuf::Message> msg)
{
    ROS_INFO_STREAM("Recv message on private");
    std::shared_ptr<OrderInfo> order_info;
    if ((order_info = std::dynamic_pointer_cast<OrderInfo>(msg)))
    {
        ROS_INFO_STREAM("" << order_info->ShortDebugString());
    }
}

void Peer::handleRefboxMessagePublic(boost::asio::ip::udp::endpoint &endpoint, uint16_t comp_id, uint16_t msg_type, std::shared_ptr<google::protobuf::Message> msg)
{
    std::shared_ptr<GameState> game_state;
    if ((game_state = std::dynamic_pointer_cast<GameState>(msg)))
    {
        ROS_INFO_STREAM("----------------" << comp_id << " : " << msg_type);
        ROS_INFO_STREAM("" << game_state->ShortDebugString());

        auto cyan = game_state->team_cyan();
        m_team_name = TEAM_NAME;
        if (cyan == TEAM_NAME)
        {
            m_is_cyan = true;
        }
    }
}

void Peer::handleRecvErrorPrivate(boost::asio::ip::udp::endpoint &endpoint, std::string msg)
{
    ROS_ERROR_STREAM("Error receiving on private port : " << msg);
}

void Peer::handleSendErrorPrivate(std::string msg)
{
    ROS_ERROR_STREAM("Error sending on private port : " << msg);
}

// this method should notify the refbox of a robot, somehow it does not work
// however the general way of sending like this should be correct
void Peer::sendBeaconSignal(Robot &robot)
{
        if (robot.beacon2main() == nullptr)
        {
            ROS_ERROR_STREAM("Still no message: ");
            return;
        }
            //ROS_INFO_STREAM("I am here: ");
        std::shared_ptr<BeaconSignal> beaconMsg_main;
        beaconMsg_main = std::dynamic_pointer_cast<BeaconSignal>(robot.beacon2main());
        if (m_team_name != TEAM_NAME)
            return;
        auto cur_time = ros::Time::now();
        float pos_x = -2.5;
        float pos_y = 3.5;
        float pos_ori = 0.0;
        std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> now =
            std::chrono::high_resolution_clock::now();
        std::chrono::seconds seconds = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch());
        std::chrono::nanoseconds nanoseconds = now.time_since_epoch() - seconds;

        std::shared_ptr<BeaconSignal> msg(new BeaconSignal());
        Time *time = msg->mutable_time();
        time->set_sec(static_cast<google::protobuf::int64>(seconds.count()));
        time->set_nsec(static_cast<google::protobuf::int64>(nanoseconds.count()));

        auto *pose = msg->mutable_pose();
        pose->set_x(pos_x);
        pose->set_y(pos_y);
        pose->set_ori(pos_ori);
        auto posetimestamp = pose->mutable_timestamp();
        posetimestamp->set_sec(static_cast<google::protobuf::int64>(seconds.count()));
        posetimestamp->set_nsec(static_cast<google::protobuf::int64>(nanoseconds.count()));

        msg->set_seq(++m_sequence_nr_);
        msg->set_number(1);
        msg->set_team_name(m_team_name);
        msg->set_peer_name("Festino");
        msg->set_team_color(m_is_cyan ? Team::CYAN : Team::MAGENTA);

        ROS_INFO_STREAM("Sending ------------------------- BeaconSignal: " << beaconMsg_main->ShortDebugString());
        m_private_peer->send(2000, 1, beaconMsg_main);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

void Peer::sendPrepareMachineCs()
{
    while (m_running)
    {
        auto cur_time = ros::Time::now();
        std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> now =
            std::chrono::high_resolution_clock::now();
        std::chrono::seconds seconds = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch());
        std::chrono::nanoseconds nanoseconds = now.time_since_epoch() - seconds;

        std::shared_ptr<PrepareMachine> msg(new PrepareMachine());
        std::shared_ptr<PrepareMachine> msg_b(new PrepareMachine());

        auto *cs = msg->mutable_instruction_cs();
        auto *bs = msg_b->mutable_instruction_bs();
        msg->set_team_color(llsf_msgs::Team::CYAN);
        msg->set_machine("C-CS1");
        cs->set_operation(llsf_msgs::CSOp::RETRIEVE_CAP);

        msg_b->set_team_color(llsf_msgs::Team::CYAN);
        msg_b->set_machine("C-BS");
        bs->set_color(llsf_msgs::BaseColor::BASE_RED);
        bs->set_side(llsf_msgs::MachineSide::INPUT);

        ROS_INFO_STREAM("Sending - PrepareCS: " << msg->ShortDebugString());
        //ROS_INFO_STREAM("Sending - PrepareBS: " << msg_b->ShortDebugString());
        m_private_peer->send(2000, 101, msg);
        //m_private_peer->send(2000, 101, msg_b);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

Robot::Robot() : m_mr(new MessageRegister())
{
    m_private_peer_robot = std::make_shared<ProtobufStreamServer>(2016);
    m_private_peer_robot->message_register().add_message_type<BeaconSignal>();
    m_private_peer_robot->signal_received().connect(
        boost::bind(&Robot::handleMessage, this, _1, _2, _3, _4));

    m_running = true;
    beacon2main_msg = nullptr;
}

Robot::~Robot()
{
    m_running = false;
}

void Robot::handleMessage(unsigned int id, uint16_t comp_id, uint16_t msg_type, std::shared_ptr<google::protobuf::Message> msg)
{
    ROS_INFO_STREAM("Recv message on private");
    std::shared_ptr<BeaconSignal> beaconMsg;
    if ((beaconMsg = std::dynamic_pointer_cast<BeaconSignal>(msg)))
    {
        this->beacon2main_msg = msg;
        ROS_INFO_STREAM("BeaconSignal - correct pointer: " << beacon2main_msg->ShortDebugString());
    }
}

std::shared_ptr<google::protobuf::Message> Robot::beacon2main()
{
    return beacon2main_msg;
}

void Robot::sendTasktoRobot(AgentTask agent_task)
{
    m_private_peer_robot->send(1, agent_task);
}