#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.h"
#include "rclcpp_lifecycle/lifecyle_node.hpp"
#include "turret/turret.hpp"

using LifecycleCallbackReturn = 
    rclcpp_lifecycle::node_intefaces::LifecycleNodeInterface::CallbackReturn;
using turret::Turret;

class TurretNode : public rclcpp_lifecycle::LifecycleNode
{
    public:
        TurretNode() : LifecycleNode("turret")
        {
            RCLCPP_INFO(this->get_logger(), "Turret node initialized");
        }

        LifecycleCallbackReturn on_configure(const 
            rclcpp_lifecycle::State &previous_state)
        {
            (void) previous_state;
            RCLCPP_INFO(this->get_logger(),"IN on_configure");

            return LifecycleCallbackReturn::SUCCESS;
        }

        LifecycleCallbackReturn on_activate(const 
            rclcpp_lifecycle::State &previous_state)
        {
            (void) previous_state;
            RCLCPP_INFO(this->get_logger(),"IN on_activate");

            return LifecycleCallbackReturn::SUCCESS;
        }

        LifecycleCallbackReturn on_cleanup(const 
            rclcpp_lifecycle::State &previous_state)
        {
            (void) previous_state;
            RCLCPP_INFO(this->get_logger(),"IN on_cleanup");

            return LifecycleCallbackReturn::SUCCESS;
        }

        LifecycleCallbackReturn on_deactivate(const 
            rclcpp_lifecycle::State &previous_state)
        {
            (void) previous_state;
            RCLCPP_INFO(this->get_logger(),"IN on_deactivate");

            return LifecycleCallbackReturn::SUCCESS;
        }

        LifecycleCallbackReturn on_shutdown(const 
            rclcpp_lifecycle::State &previous_state)
        {
            (void) previous_state;
            RCLCPP_INFO(this->get_logger(),"IN on_shutdown");

            return LifecycleCallbackReturn::SUCCESS;
        }


    private:
        std::unique_ptr<Turret> turret_ = std::make_unique<Turret>();
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto turret_node = std::make_shared<TurretNode>();
    rclcpp::spin(turret_node->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}

