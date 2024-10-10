#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "stroam_interfaces/action/turret_instruct.hpp"
#include "turret/turret.hpp"

//Turret positions are in degrees
#define CENTER_TURRET_POS 90 
#define MAX_TURRET_POS 180
#define MIN_TURRET_POS 0

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
		Node("Motor_Controller"),
		turret_(std::make_unique<Turret>())
        {
		turret_enabled_ = false;
		current_turret_pos_ =  CENTER_TURRET_POS;

		callback_group_1_ = this->
			create_callback_group(rclcpp::CallbackGroupType::Reentrant);
		
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
        std::unique_ptr<Turret> turret_; 

		bool turret_enabled_; 
		int current_turret_pos_; //[0,180] Degrees 
		std::mutex mutex_;

		std::shared_ptr<TurretGoalHandle> current_handle_goal_;
		rclcpp_action::Server<TurretInstruct>::SharedPtr turret_server_;
		rclcpp::CallbackGroup::SharedPtr callback_group_1_;
	
	/**
	 * @brief Handle incoming turret goal (TurretInstruct)
	 *
	 * Policy: A new turret instruction is accepted provided that a current 
	 * goal isn't being executed, otherwise reject it. 
	 *@param goal new incoming turret instruction 
	 */
	rclcpp_action::GoalResponse goal_callback(
		const rclcpp_action::GoalUUID &uuid, 
		std::shared_ptr<const TurretInstruct::Goal> goal)
	{
		{
			std::lock_guard<std::mutex> lock(mutex_);
			if (current_handle_goal_) 
			{
				if (current_handle_goal_ -> is_active())
				{
					RCLCPP_INFO(this->get_logger(), 
						"[Turret Server]: A goal is active, rejected new goal");
					return rclcpp_action::GoalResponse::REJECT;
				}
			}
		}

		(void) uuid;
		(void) goal;
		return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	}

	/**
	 * @brief Handle a rejected turret goal request
	 *
	 */
	rclcpp_action::CancelResponse cancel_callback(
		const std::shared_ptr<TurretGoalHandle> goal_handle)
	{
		(void) goal_handle;
		return rclcpp_action::CancelResponse::ACCEPT;
	}
	
	/**
	 * @brief Handle accepted turret goal sent from Stroam manager node
	 *@param new_goal_handle the latest turret action 
	 */
	void handle_accepted_callback(
		const std::shared_ptr<TurretGoalHandle> new_goal_handle)
	{
		{
			std::lock_guard<std::mutex> lock (mutex_);
			this -> current_handle_goal_ = new_goal_handle;
		}

		execute_turret_goal(new_goal_handle);
		send_goal_result_(new_goal_handle);
	}

	/**
	 *@brief Operate turret based on passed turret instructions 
	 * 
	 * @param goal_handle the action containing turret instructions 
	 */
	void execute_turret_goal(const std::shared_ptr<TurretGoalHandle> 
		goal_handle)
	{
		auto turret_goal = goal_handle -> get_goal();

		if (turret_goal -> fire_turret)
		{
			turret_-> fire();
			return;
		}

		if (turret_goal -> laser_on)
		{
			turret_ -> laser_on();
		}
		else
		{
			turret_ -> laser_off();
		}

		handle_turret_rotation(turret_goal-> right_joy_stick_x);
	}

	/**
	 *@brief Set turrets position (utility function for execute_turret_goal)
	 */
	void handle_turret_rotation(float joy_stick_val)
	{
		int offset_angle; 

		offset_angle = get_offset_angle(joy_stick_val);
		if (offset_angle == 0) return;

		if (offset_angle > 0)
		{
			handle_right_rotation(offset_angle); 
		}
		else 
		{
			handle_left_rotation(offset_angle); 
		}
	}

	/** 
	 *@brief Get offset angle from joy-stick value
	 *
	 * The returned angle will be used to offset turrets current pos. 
	 *@param joyVal right joy stick value [-1,1]
	 *@return offset_angle is a value [0,90] 
	*/
	int get_offset_angle(float joyVal)
	{
		int offset_angle = 0; 


		if (joyVal == 0.0)	
		{
			return offset_angle;
		}
		else if(joyVal > 0.0)
		{
			offset_angle = joyVal * CENTER_TURRET_POS;

		} 
		else 
		{
			offset_angle = joyVal * CENTER_TURRET_POS;
		}

		return offset_angle; 
	}

	/**
	 *@brief Offset from turrets current pos to the right
	 *@param offset_angle positive offset angle (0,90]
	 */	
	void handle_right_rotation(int offset_angle) 
	{
		if (offset_angle + current_turret_pos_ > MAX_TURRET_POS)
		{
			current_turret_pos_ = MAX_TURRET_POS;
		}
		else
		{
			current_turret_pos_ = offset_angle + current_turret_pos_;
		}
		
		turret_ -> move_to_pos(current_turret_pos_);
	}

	/**
	 *@brief Offset from turrets current pos to the left 
	 *@param offset_angle negative offset angle (0,90] 
	 */	
	void handle_left_rotation(int offset_angle) 
	{
		if (offset_angle + current_turret_pos_ < MIN_TURRET_POS)
		{
			current_turret_pos_ = MIN_TURRET_POS;
		}
		else 
		{
			current_turret_pos_ = offset_angle + current_turret_pos_;
		}
		
		turret_ -> move_to_pos(current_turret_pos_);
	}
	void send_goal_result_(const std::shared_ptr<TurretGoalHandle> goal_handle)
	{
		auto result = std::make_shared<TurretInstruct::Result>();
		result->result = "Success";
		goal_handle->succeed(result);
	}

};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto turret_node = std::make_shared<TurretNode>();
    rclcpp::executors::MultiThreadedExecutor executor;

    executor.add_node(turret_node);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}

