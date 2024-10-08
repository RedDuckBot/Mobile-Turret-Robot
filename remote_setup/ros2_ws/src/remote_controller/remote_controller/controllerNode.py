import rclpy 
from rclpy.node import Node 
from stroam_interface.msg import XboxController 
from xbox360controller import Xbox360Controller

class XboxControllerNode(Node):
    """
    Represents a Xbox360 controller node.

    Attributes:
        Xcontroller_ (Xbox360Controller): Controller object tracks joy stick
                                            & button presses
        controller_pub_: the publisher of XboxController messages on 
                        /stroam_manager topic
        current_right_joy_ (float): initial right joy stick value [-1,1]
        current_left_joy_ (float): initial left joy stick value [-1,1]
        drive_mode_ (bool): Initially drive mode is enabled
        laser_on_ (bool): Tracks state of laser [on,off]
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
                                "stroam_manager", 10)

        #Initialize and setup xbox controller
        self.Xcontroller_ = Xbox360Controller(0,axis_threshold=0.0)
        self.controller.button_x.when_pressed = self.on_button_press
        self.controller.button_b.when_pressed = self.on_button_press
        self.axis_r.when_moved = self.on_joy_move
        self.axis_l.when_moved = self.on_joy_move

        self.current_right_joy_ = 0.0
        self.current_left_joy_ = 0.0
        self.drive_mode_ = True
        self.laser_on_ = False
        self.get_logger().info("Xbox controller node initialized.")

        def on_joy_move(self, axis):
            """
            Publish interface XboxController message for joy stick values.

            Args:
                axis: joy values [-1,1] for either left or right joystick
            """

            msg = XboxController()

            if not (axis.y > -0.25 and axis.y < 0.25):
                if axis.name == "axis_r":
                    msg.right_joy_y = axis.y
                    self.current_right_joy = axis.y
                    msg.left_joy_y = self.current_left_joy
                else:
                    msg.left_joy_y = axis.y
                    self.current_left_joy = axis.y
                    msg.right_joy_y = self.current_right_joy
            else:
                if axis.name == "axis_r":
                    msg.right_joy_y = 0.0
                    self.current_right_joy = 0.0 
                    msg.left_joy_y = self.current_left_joy
                else:
                    msg.left_joy_y = 0.0
                    self.current_left_joy = 0.0
                    msg.right_joy_y = self.current_right_joy

            self.publish_msg(msg)
            
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
                if button.name == self.VALID_BUTTON_NAMES[0]:
                    msg.x = True
                    msg.b = self.laser_on_
                else: #Toggle laser
                    self.laser_on_ = not self.laser_on_ 
                    msg.x = False
                self.controller_pub_.publish(msg)

        def handle_button_press_modes(self): 
            """
            Utility function for on_button_press to handle mode button press.
            """
            msg = XboxController()

            self.drive_mode_ = not self.drive_mode_
            msg.mode = self.drive_mode_
            self.publish_msg(msg)

            
        def publish_msg(self, msg: XboxController):
            """
            Utility function for finalizing XboxController and publishing it.
            """
            msg.mode = self.drive_mode_
            self.controller_pub.publish(msg)


def main():
    rclpy.init()
    controller_node = XboxControllerNode()
    rclpy.spin(controller_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()