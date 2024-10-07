import rclpy 
from rclpy.node import Node 
from stroam_interface.msg import XboxController
from xbox360controller import Xbox360Controller
from rclpy.action.client import ClientGoalHandle
from rclpy.action import ActionClient


class XboxControllerNode(Node):
    def __init__(self):
        super().__init__("controller")
        self.controller_client_1 = ActionClient(self, Drive, "droid_buttons")
        self.controller_client_2 = ActionClient(self, Drive, "droid_motors")

        #Initialize and setup xbox controller
        self.Xcontroller_ = Xbox360Controller(0,axis_threshold=0.0)

        self.current_right_joy = 0.0
        self.current_left_joy = 0.0
        self.get_logger().info("Xbox controller node initialized.")


def main():
    rclpy.init()
    controller_node = XboxControllerNode()
    rclpy.spin(controller_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()