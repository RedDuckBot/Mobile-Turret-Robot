#!/usr/binenv python3

import rclpy 
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from stroam_interfaces.action import RemoteControl
from rclpy.action import ActionServer
from rclpy.action.server import SErverGoalHandle


class StroamLifecycNodeManager(Node):
    """
    Represents a life cycle node manager for Stroam: turret, 
    motor controller, and camera life cycle nodes.

    Attributes:
        turret_client: Represents a service client for lifecycle node turret
        motor_client: Represents a service client for lifecycle node motor controller
        camera_client: Represents a service client for lifecycle node camera
        stroam_manager: Represents an action server for remote control from 
                        Xbox360 controller
    """

    def __init__(self):
        """
        Initializes a new instance of the StroamcycNodeManager class
        """

        super()__init__("stroam_lifecyc_manager")
        
        turret_srv_change_state_name = "/turret/change_state"
        motor_srv_change_state_name = "/motor_controller/change_state"
        camera_srv_change_state_name = "/camera/change_state"

        self.turret_client = self.create_client(ChangeState, 
                turret_srv_change_state_name)
        self.motor_client = self.create_client(ChangeState, 
                motor_srv_change_state_name)
        self.camera_client = self.create_client(ChangeState, 
                camera_srv_change_state_name)

        self.stroam_manager_server_ = ActionServer(self, RemoteControl, 
                "stroam_manager", execute_callback=self.execute_callback)

        self.get_logger().info("Stroam Manager has been started")

        def execute_callback(self, goal_handle: ServerGoalHandle):
            pass





def main(args=None):
    rclpy.init(args=args)
    manager_node = StroamLifecycNodeManager()
    rclpy.spin(manager_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()


