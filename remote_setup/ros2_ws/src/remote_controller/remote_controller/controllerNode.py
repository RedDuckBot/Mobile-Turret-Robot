import rclpy 
from rclpy.node import Node 
from stroam_interfaces.msg import XboxController 
from xbox360controller import Xbox360Controller
from dataclasses import dataclass
import time


class XboxControllerNode(Node):
    """
    Represents a Xbox360 controller node.

    Attributes:
        Xcontroller_ (Xbox360Controller): Controller object tracks joy stick
                                            & button presses
        controller_pub_: the publisher of XboxController messages on 
                        /xbox_controller topic
        current_right_joy_ (float): initial right joy stick value [-1,1]
        current_left_joy_ (float): initial left joy stick value [-1,1]
        drive_mode_ (bool): Initially drive mode is enabled
    """

    VALID_BUTTON_NAMES = ["button_x", "button_b", "button_mode"] 
    """
        x               <--- fire turret
        b               <--- toggle laser   
        mode            <--- if true then in drive mode else in turret mode
    """

    def __init__(self):  
        super().__init__("controller")
        self.controller_pub_ = self.create_publisher(XboxController, 
                                "/xbox_controller", 10)

        #Initialize and setup xbox controller
        self.controller_ = Xbox360Controller(0,axis_threshold=0.0)
        self.controller_.button_x.when_pressed = self.on_button_press
        self.controller_.button_b.when_pressed = self.on_button_press
        self.controller_.button_a.when_pressed = self.on_button_press
        self.controller_.button_mode.when_pressed = self.on_button_press
        self.controller_.axis_r.when_moved = self.on_joy_move
        self.controller_.axis_l.when_moved = self.on_joy_move

        self.current_right_joy_ = 0.0
        self.current_left_joy_ = 0.0
        self.drive_mode_ = True
        self.get_logger().info("Xbox controller node initialized.")

    def on_joy_move(self, axis):
        """
        Publish interface XboxController message for joy stick values.

        Args:
            axis: joy values [-1,1] for either left or right joystick
        """

        msg = XboxController()

        if not (axis.y > -0.25 and axis.y < 0.25) or  \
            not (axis.x > -0.25 and axis.x < 0.25):
            if axis.name == "axis_r":
                msg.right_joy_x = axis.x
                msg.right_joy_y = axis.y
                self.current_right_joy_ = axis.y
                msg.left_joy_y = self.current_left_joy_
            else:
                msg.left_joy_y = axis.y
                self.current_left_joy_ = axis.y
                msg.right_joy_y = self.current_right_joy_
        else:
            if axis.name == "axis_r":
                msg.right_joy_x = 0.0
                msg.right_joy_y = 0.0
                self.current_right_joy_ = 0.0 
                msg.left_joy_y = self.current_left_joy_
            else:
                msg.left_joy_y = 0.0
                self.current_left_joy_ = 0.0
                msg.right_joy_y = self.current_right_joy_

        msg.mode = self.drive_mode_
        self.controller_pub_.publish(msg)
            
    def on_button_press(self, button):
        """
        Publish interface XboxController message for button presses.

        Args:
            button: Contains name of button pressed
        """

        if button.name in self.VALID_BUTTON_NAMES:
            if button.name in self.VALID_BUTTON_NAMES[:2]: 
                self.handle_turret_button_press(button)
            else: 
                self.handle_button_press_modes()

    def handle_turret_button_press(self, button):
        """
        Utility function for on_button_press to handle turret button presses.

        Args:
            button: Contains the name of button pressed
        """
        msg = XboxController()

        if not self.drive_mode_:
            if button.name == self.VALID_BUTTON_NAMES[1]:
               msg.b = True  #Laser button
            else:
                msg.x = True #Fire button

            msg.mode = self.drive_mode_
            self.controller_pub_.publish(msg)

    def handle_button_press_modes(self): 
        """
        Utility function for on_button_press to handle mode button press.
        """
        msg = XboxController()

        if self.drive_mode_:
            self.get_logger().info("[Remote Contr] Switching to Turret Mode")
        else:
            self.get_logger().info("[Remote Contr] Switching to Drive Mode")

        self.drive_mode_ = not self.drive_mode_
        msg.mode = self.drive_mode_
        self.controller_pub_.publish(msg)

def main():
    rclpy.init()
    controller_node = XboxControllerNode()
    rclpy.spin(controller_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()