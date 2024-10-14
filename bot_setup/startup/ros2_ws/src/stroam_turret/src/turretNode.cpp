#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "stroam_interfaces/action/turret_instruct.hpp"
#include "turret/turret.hpp"
#include <fmt/core.h>
#include <string>
#include <iostream>
#include <cstdlib>

//Turret positions are in degrees
#define CENTER_TURRET_POS 90 
#define MAX_TURRET_POS_LEFT 180 
#define MAX_TURRET_POS_RIGHT 0  

using turret::Turret;
using TurretInstruct = stroam_interfaces::action::TurretInstruct;
using TurretGoalHandle = rclcpp_action::ServerGoalHandle<TurretInstruct>;
using namespace std::placeholders;
using namespace fmt; 


/**
 * @class TurretNode
 * @brief Represents a server for Stroam's turret. 
 *
 * This server node receives turret instructions from Stroam's managing node.
 * Two action servers within this class handle requests for turret button 
 * presses and joy-stick moves.
 */
class TurretNode: public rclcpp::Node 
{
    public:
        TurretNode() : 
		Node("Turret"),
		turret_(std::make_unique<Turret>())
        {
		current_turret_pos_ =  CENTER_TURRET_POS;

		//Initialize call back groups
		callback_group_1_ = this->
			create_callback_group(rclcpp::CallbackGroupType::Reentrant);

		callback_group_2_ = this->
			create_callback_group(rclcpp::CallbackGroupType::Reentrant);

		//Initialize action servers: button presses and joy-stick movements
		 turret_joy_server_ = rclcpp_action::create_server<TurretInstruct>(
			this,
			"turret_joy_moves",
			std::bind(&TurretNode::goal_callback,this, _1, _2),				 		      	      	      
			std::bind(&TurretNode::cancel_callback, this, _1),
			std::bind(&TurretNode::handle_accepted_callback, this, _1),
		        rcl_action_server_get_default_options(),
			callback_group_1_
		);
		 turret_button_server_ = rclcpp_action::create_server<TurretInstruct>(
			this,
			"turret_button_presses",
			std::bind(&TurretNode::goal_callback,this, _1, _2),				 		      	      	      
			std::bind(&TurretNode::cancel_callback, this, _1),
			std::bind(&TurretNode::handle_accepted_callback, this, _1),
		        rcl_action_server_get_default_options(),
			callback_group_2_
		);
		
		RCLCPP_INFO(this->get_logger(), "Turret node initialized");
        }

    private:
        std::unique_ptr<Turret> turret_; 

		int current_turret_pos_; //[0,180] Degrees 
		std::mutex mutex_button_; //Lock used by turret button server
		std::mutex mutex_joy_;   //Lock used by turret joy-stick server

		std::shared_ptr<TurretGoalHandle> current_button_handle_goal_;
		std::shared_ptr<TurretGoalHandle> current_joy_handle_goal_;
		rclcpp_action::Server<TurretInstruct>::SharedPtr turret_joy_server_;
		rclcpp_action::Server<TurretInstruct>::SharedPtr turret_button_server_;


		//Callback group 1 contains main turret node
		//Callback group 2 contains a node for handling turret button presses
		rclcpp::CallbackGroup::SharedPtr callback_group_1_; 
		rclcpp::CallbackGroup::SharedPtr callback_group_2_;
	
	/**
	 * @brief Handle incoming turret goal (TurretInstruct)
	 *
	 * Policy: A new turret instruction is accepted provided that a current 
	 * goal isn't being executed, otherwise reject it.  
	 * These instructions come in as either joy-stick moves or button presses
	 * which are handled by the action servers.
	 *@param goal new incoming turret instruction 
	 */
	rclcpp_action::GoalResponse goal_callback(
		const rclcpp_action::GoalUUID &uuid, 
		std::shared_ptr<const TurretInstruct::Goal> goal)
	{
		(void) uuid;

		bool callback_result; 

		if (goal -> is_button_presses) {
			callback_result = help_goal_callback(
				current_button_handle_goal_, mutex_button_, "button"
			);
		}
		else if (goal -> is_joy_moves) 
		{
			callback_result = help_goal_callback(
				current_joy_handle_goal_, mutex_joy_, "joy-stick"
			);
		}

		if (callback_result) 
			return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
		else return rclcpp_action::GoalResponse::REJECT;
	}

	/**
	 *@brief Determine if newest turret instruction can be executed. 
	 *
	 * Utility function for general goal_callback func. These instructions are
	 * either button presses or joy-stick moves
	 *@param current_handle_goal either current button or joy-stick goal 
	 *@param thread_lock either button or joy-stick thread lock 
	 *@param instruction_name the name of type of instruc. handle: button or mv
	 *@return true if newest turret goal can be executed, i.e. a current one
	 *        isn't in execution. 
	 */
	bool help_goal_callback(
		std::shared_ptr<TurretGoalHandle> current_handle_goal,
		std::mutex& thread_lock,
		const std::string instruction_name
		 )
	{
		{ 
			std::lock_guard<std::mutex> lock(thread_lock);
			if (current_handle_goal) 
			{
				if (current_handle_goal -> is_active())
				{
					RCLCPP_INFO(this->get_logger(), 
						("A" + instruction_name + 
						"goal is active, rejected new one.").c_str()
					);
					return false; 
				}
			}
		}

		return true;
	}

	/**
	 * @brief Handle a rejected turret goal request
	 * 
	 * Currently, no measures need be taken for canceled turret instruction
	 * since new instruc. are ignored if turret is handling a current one. 
	 */
	rclcpp_action::CancelResponse cancel_callback(
		const std::shared_ptr<TurretGoalHandle> goal_handle)
	{
		(void) goal_handle;
		return rclcpp_action::CancelResponse::ACCEPT;
	}
	
	/**
	 *@brief Handle accepted turret goal (either button presses or joy moves) 
	 *@param goal_handle the latest turret action 
	 */
	void handle_accepted_callback(
		const std::shared_ptr<TurretGoalHandle> new_goal_handle)
	{
		auto turret_goal = new_goal_handle -> get_goal();

		if (turret_goal -> is_button_presses)
		{
			{
				std::lock_guard<std::mutex> lock (mutex_button_);
				this -> current_button_handle_goal_ = new_goal_handle;
				//execute_turret_button_goal(new_goal_handle);
			}
		}

		if (turret_goal -> is_joy_moves)
		{
			{
				std::lock_guard<std::mutex> lock (mutex_joy_);
				this -> current_joy_handle_goal_ = new_goal_handle;
				execute_turret_joy_goal(new_goal_handle);
			}
		}

		// send_goal_result_(new_goal_handle);
	}

	/**
	 *@brief Execute button commands (firing,toggle laser)
	 *@param goal_button_handle the action containing turret button values
	 */
	void execute_turret_button_goal(const std::shared_ptr<TurretGoalHandle>
		goal_button_handle)
		{
			auto turret_button_goal = goal_button_handle -> get_goal();

			if (turret_button_goal -> laser_on)
			{
				turret_ -> laser_on();
			}
			else
			{
				turret_ -> laser_off();
			}

			if (turret_button_goal -> fire_turret)
			{
				turret_ -> fire();
			}
		}

	/**
	 *@brief Execute turret movement 
	 * 
	 * @param goal_joy_handle the action containing turret joy-stick values  
	 */
	void execute_turret_joy_goal(const std::shared_ptr<TurretGoalHandle> 
		goal_joy_handle)
	{
		auto turret_joy_goal = goal_joy_handle -> get_goal();

		handle_turret_rotation(turret_joy_goal-> right_joy_stick_x);
	}

	/**
	 *@brief Set turrets position 
	 */
	void handle_turret_rotation(float joy_stick_val)
	{
		int offset_angle, offset_buffer = 15; 

		offset_angle = get_offset_angle(joy_stick_val);

		if (offset_angle == 0) return;

		if (joy_stick_val > 0.0)
		{
			offset_angle -= offset_buffer;
			handle_right_rotation(offset_angle); 
		}
		else 
		{
			offset_angle -= offset_buffer;
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
		else 
		{
			offset_angle = std::abs(joyVal * CENTER_TURRET_POS);
		}

		return offset_angle; 
	}

	/**
	 *@brief Offset from turrets current pos to the right
	 *@param offset_angle offset angle (0,90]
	 */	
	void handle_right_rotation(int offset_angle) 
	{
		if (current_turret_pos_ - offset_angle < MAX_TURRET_POS_RIGHT)
		{
			current_turret_pos_ = MAX_TURRET_POS_RIGHT;
		}
		else
		{
			current_turret_pos_ = current_turret_pos_ - offset_angle;
		}
		
		turret_ -> move_to_pos(current_turret_pos_);
	}

	/**
	 *@brief Offset from turrets current pos to the left 
	 *@param offset_angle  offset angle (0,90] 
	 */	
	void handle_left_rotation(int offset_angle) 
	{
		if (current_turret_pos_ + offset_angle > MAX_TURRET_POS_LEFT)
		{
			current_turret_pos_ = MAX_TURRET_POS_LEFT;
		}
		else 
		{
			current_turret_pos_ = current_turret_pos_ + offset_angle;
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