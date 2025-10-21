#ifndef ROBOTINONODE_HPP_
#define ROBOTINONODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "robotino_node/ComROS.hpp"
#include "robotino_node/BumperROS.hpp"
#include "robotino_node/MotorArrayROS.hpp"
#include "robotino_node/PowerManagementROS.hpp"
#include "robotino_node/OmniDriveROS.hpp"
#include "robotino_node/GyroscopeROS.hpp"
//#include "robotino_node/AnalogInputArrayROS.hpp"
//#include "robotino_node/DigitalInputArrayROS.hpp"
#include "robotino_node/DigitalOutputArrayROS.hpp"
//#include "robotino_node/DistanceSensorArrayROS.hpp"
//#include "robotino_node/ElectricalGripperROS.hpp"
#include "robotino_node/EncoderInputROS.hpp"

//#include "sensor_msgs/msg/joint_state.hpp"

class RTONode : public rclcpp::Node
{
public:
	RTONode(const std::string& name);
	~RTONode();

	void execute();

private:
    std::shared_ptr<ComROS> com_;
	std::shared_ptr<BumperROS> bumper_;
	std::shared_ptr<MotorArrayROS> motor_array_;
	std::shared_ptr<PowerManagementROS> power_management_;
	std::shared_ptr<OmniDriveROS> omni_drive_;
	std::shared_ptr<GyroscopeROS> imu_;
	//std::shared_ptr<AnalogInputArrayROS> analog_input_array_;
	//std::shared_ptr<DigitalInputArrayROS> digital_input_array_;
	std::shared_ptr<DigitalOutputArrayROS> digital_output_array_;
	//std::shared_ptr<DistanceSensorArrayROS> distance_sensor_array_;
	//std::shared_ptr<ElectricalGripperROS> electrical_gripper_;
	std::shared_ptr<EncoderInputROS> encoder_input_;

	void initModules();

};

#endif /* ROBOTINONODE_HPP_ */
