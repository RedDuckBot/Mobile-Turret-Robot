#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "stroam_interfaces/action/turret_instruct.hpp"
#include "turret/turret.hpp"

using turret::Turret;
using TurretInstruct = stroam_interfaces::action::TurretInstruct;
using TurretGoalHandle = rclcpp_action::ServerGoalHandle<TurretInstruct>;
using namespace std::placeholders;

/**
 * @class TurretNode
 * @brief Represents an action server for Stroam's turret. 
 */
class TurretNode: public rclcpp::Node 
{
    public:
        TurretNode() : 
		Node("Motor_Controller")
        {
		turret_enabled = false;
		callback_group_1_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
		
		 turret_server_ = rclcpp_action::create_server<TurretInstruct>(
			this,
			"turret_actions",
			std::bind(&TurretNode::goal_callback,this, _1, _2),				 		      	      	      std::bind(&TurretNode::cancel_callback, this, _1),
			std::bind(&TurretNode::handle_accepted_callback, this, _1),
		        rcl_action_server_get_default_options(),
			callback_group_1_
		);
		
		RCLCPP_INFO(this->get_logger(), "Turret node initialized");
        }

    private:
        std::unique_ptr<Turret> turret_ = std::make_unique<Turret>();

	bool turret_enabled; 
	
	rclcpp_action::Server<TurretInstruct>::SharedPtr turret_server_;
	rclcpp::CallbackGroup::SharedPtr callback_group_1_;
	
	/**
	 * @brief Handle incoming turret goal from TurretInstruct
	 */
	rclcpp_action::GoalResponse goal_callback(
		const rclcpp_action::GoalUUID &uuid, 
		std::shared_ptr<const TurretInstruct::Goal> goal)
	{
		(void) uuid;
		(void) goal;
		return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	}

	/**
	 * @brief Handle a rejected turret goal request
	 */
	rclcpp_action::CancelResponse cancel_callback(
		const std::shared_ptr<TurretGoalHandle> goal_handle)
	{
		(void) goal_handle;
		return rclcpp_action::CancelResponse::ACCEPT;
	}
	
	/**
	 * @brief Handle accepted turret goal sent from Stroam manager node
	 */
	void handle_accepted_callback(
		const std::shared_ptr<TurretGoalHandle> goal_handle)
	{
		(void) goal_handle;
	}

};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto turret_node = std::make_shared<TurretNode>();
    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(turret_node);
    executor.spin();

    rclcpp::spin(turret_node->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}

