#include <ros/ros.h>
#include <protobuf_comm/server.h>
#include <refbox_protobuf_msgs/BeaconSignal.pb.h>

#define TEAM_NAME "GRIPS"
#define CRYPTO_KEY "randomkey"

using namespace protobuf_comm;

class Robot
{
    using BeaconSignal = llsf_msgs::BeaconSignal;

public:
    Robot();
    ~Robot();

    void handleMessage(unsigned int id, uint16_t comp_id, uint16_t msg_type, std::shared_ptr<google::protobuf::Message> msg);


private:
    std::string m_host;
    int m_priv_recv_port;
    int m_pub_recv_port;
    unsigned long int m_sequence_nr_;
    MessageRegister *m_mr;
    std::shared_ptr<ProtobufStreamServer> m_private_peer;
    bool m_is_cyan;
    std::thread m_beacon_thread;
    bool m_running;
};