#!/usr/binenv python3

import rclpy 
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from stroam_interfaces.action import MotorsInstruct, TurretInstruct
from stroam_interfaces.msg import XboxController
from rclpy.action import ActionClient


class StroamNodeManager(Node):
    """
    Represents a node manager for Stroam: turret, 
    motor controller, and camera nodes.

    Attributes:
        turret_button_client_: Represents an action client for turret buttons 
        turret_joy_client_: Represents an action client for turret joy-sticks 
        motors_client_: Represents an action client for motor controller node
        camera_client_: Represents a service client for lifecycle camera node 
        controller_client_: Represensts a subscriber for Xbox360 contr inputs
        in_drive_mode_: Used to differentiate between turret and drive mode
    """

    def __init__(self):

        super().__init__("stroam_manager")
        
        motors_topic_name = "/motor_controller_actions"
        camera_srv_change_state_name = "/camera/change_state"

        self.motors_client_ = ActionClient(self, MotorsInstruct,
                 motors_topic_name)
        self.turret_joy_client_ = ActionClient(self, TurretInstruct, 
                "/turret_joy_moves")
        self.turret_button_client_ = ActionClient(self, TurretInstruct, 
                "/turret_button_presses")
        self.controller_client_ = self.create_subscription(XboxController,
                "/xbox_controller", self.handle_controller_messages, 10)
        #self.camera_client_ = self.create_client(ChangeState, 
                #camera_srv_change_state_name)

        self.in_drive_mode_ = True
        self.first_time_entering_mode_ = True
        self.laser_on_ = False

        self.get_logger().info("Stroam manager started")


    def handle_controller_messages(self, msg: XboxController):
        """
        Handles Xbox360 controller messages.

        Controller inputs meaning:
                x       <--- fire turret
                b       <--- toggle laser
                mode    <--- if true then in drive mode else in turret mode
        Args:
                msg (XboxController): Contains controller inputs.
        """
        
        self.display_any_changes_in_mode(msg)

        if msg.mode:
            self.send_motors_goal(msg)
        else:
            self.send_turret_goal(msg)

    def display_any_changes_in_mode(self, msg: XboxController):
        """
        Check for any changes in modes: turret to driving and vice versa.
        If so then display it.

        Args:
            msg (XboxController): Contains controller inputs.
        """
        if self.first_time_entering_mode_:
            self.in_drive_mode_ = msg.mode
            self.display_mode()
            self.first_time_entering_mode_ = False
        else:
            if msg.mode != self.in_drive_mode_:
                self.in_drive_mode_ = msg.mode
                self.display_mode()

    def display_mode(self):
        """
        Utility func. for display_any_changes_in_mode.
        """

        mode_str = "Drive mode" if self.in_drive_mode_ else "Turret mode" 

        if self.first_time_entering_mode_:
            self.get_logger().info("Starting session in " + mode_str)
        else:
            self.get_logger().info("Switching to " + mode_str)

    def send_motors_goal(self, msg: XboxController):
        """
        Send joy-stick inputs to motor controller action server.
        Utility func. for handle_controller_messages.

        Args:
                msg (XboxController): Contains controller inputs.
        """

        #Create motors goal
        motors_goal = MotorsInstruct.Goal()
        motors_goal.left_joy_stick_y = msg.left_joy_y
        motors_goal.right_joy_stick_y = msg.right_joy_y

        self.motors_client_.send_goal_async(motors_goal)

    def send_turret_goal(self, msg: XboxController):
        """
        Send button and right joy-stick values to turret action server node.

        Args:
                msg (XboxController): Contains controller inputs.
        """

        turret_joy_goal = TurretInstruct.Goal()
       
        if msg.x or msg.b:
            self.help_turret_goal_button_press(msg)
            return

        turret_joy_goal.is_joy_moves = True
        turret_joy_goal.is_button_presses = False
        turret_joy_goal.right_joy_stick_x = msg.right_joy_x
        
        self.turret_joy_client_.send_goal_async(turret_joy_goal)

    def help_turret_goal_button_press(self, msg: XboxController):
        """
        Send button presses for turret. Utility func. for send_turret_goal. 

         Args:
            msg (XboxController): Contains controller inputs.
        """

        turret_button_goal = TurretInstruct.Goal() 

        turret_button_goal.fire_turret = True if msg.x else False
        turret_button_goal.laser_on = True if msg.b else False  

        turret_button_goal.is_button_presses = True
        turret_button_goal.is_joy_moves = False

        self.turret_button_client_.send_goal_async(turret_button_goal)


def main(args=None):
    rclpy.init(args=args)
    manager_node = StroamNodeManager()
    rclpy.spin(manager_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()


