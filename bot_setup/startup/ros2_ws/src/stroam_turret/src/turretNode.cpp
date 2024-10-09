#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "stroam_interfaces/msg/xbox_controller.hpp"
#include "turret/turret.hpp"

using turret::Turret;

/**
 * @class TurretNode
 * @brief Represents an action server for Stroam's turret. 
 */
class TurretNode : public Node 
{
    public:
        TurretNode() 
        {
		callback_group_1_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
		RCLCPP_INFO(this->get_logger(), "Turret node initialized");
        }



    private:
        std::unique_ptr<Turret> turret_ = std::make_unique<Turret>();

	rclcpp_CallbackGroup::sharedPtr callback_group_1_;
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto turret_node = std::make_shared<TurretNode>();
    rclcpp::executors::MultiThreadedEXecutor executor;

    executor.add_node(turret_node);
    executor.spin():

    rclcpp::spin(turret_node->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}

