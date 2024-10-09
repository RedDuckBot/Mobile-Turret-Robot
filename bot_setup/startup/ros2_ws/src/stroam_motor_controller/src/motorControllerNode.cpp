#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "stroam_interfaces/action/motors_instruct.hpp"
#include "motor_driver/motorDriver.hpp"

//L289N pins to GPIO pins
#define IN1 23
#define IN2 24
#define IN3 17
#define IN4 27
#define ENA 25
#define ENB 22

using motorDriver:: MotorDriver;
using motorDriver::MotorDirection;
using MotorsInstruct = stroam_interfaces::action::MotorsInstruct;
using MotorsGoalHandle = rclcpp_action::ServerGoalHandle<MotorsInstruct>;
using namespace std::placeholders;

/**
 * @class MotorNode
 * @brief Represents an action server for Stroam's motor controller (L298N). 
 * 
 * This class node communicates with Stroam's manager node to enable movement of 
 * entire robot.
 */
class MotorControllerNode : public rclcpp::Node 
{
    public:
        MotorControllerNode() :
	    Node("Motors_Node"),
            leftMotors_(IN3, IN4, ENB, "leftMotors"),
            rightMotors_(IN1, IN2, ENA, "rightMotors")
        {
		motor_control_enabled = false;
		callback_group_1_ = this->create_callback_group(
            		rclcpp::CallbackGroupType::Reentrant);

		motor_contr_server_ = rclcpp_action::create_server<MotorsInstruct>(
				this,
				"motor_controller_actions",
				std::bind(&MotorControllerNode::goal_callback, this, _1, _2),
				std::bind(&MotorControllerNode::cancel_callback, this, _1),
				std::bind(&MotorControllerNode::handle_accepted_callback, this, _1),
				rcl_action_server_get_default_options(),
				callback_group_1_
		);
		
		RCLCPP_INFO(this->get_logger(), "Motor controller node initialized");
        }

    private:
        MotorDriver leftMotors_; 
        MotorDriver rightMotors_; 

	bool motor_control_enabled; 

	rclcpp_action::Server<MotorsInstruct>::SharedPtr motor_contr_server_;
	rclcpp::CallbackGroup::SharedPtr callback_group_1_;
	
	/**
	 * @brief Handle incoming motor controller goal inside MotorsInstruct
	 */
	rclcpp_action::GoalResponse goal_callback(
		const rclcpp_action::GoalUUID &uuid, 
		std::shared_ptr<const MotorsInstruct::Goal> goal)
	{
		(void) uuid;
		(void) goal;
		return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	}

	/**
	 * @brief Handle a rejected motor controller goal request
	 */
	rclcpp_action::CancelResponse cancel_callback(
		const std::shared_ptr<MotorsGoalHandle> goal_handle)
	{
		(void) goal_handle;
		return rclcpp_action::CancelResponse::ACCEPT;
	}
	
	/**
	 * @brief Handle accepted motor controller goal sent from Stroam manager node
	 */
	void handle_accepted_callback(
		const std::shared_ptr<MotorsGoalHandle> goal_handle)
	{
		(void) goal_handle;
	}

};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto motor_controller_node = std::make_shared<MotorControllerNode>();
    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(motor_controller_node);
    executor.spin();

    rclcpp::spin(motor_controller_node->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}
