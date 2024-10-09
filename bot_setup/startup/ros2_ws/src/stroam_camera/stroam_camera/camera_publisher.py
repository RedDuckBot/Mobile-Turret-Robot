#!/usr/bin/env python3

import rclpy
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn, LifecycleNode
import socket, pickle, cv2, threading


class CameraNode(LifecycleNode):
    """
    Represents a camera for 'publishing' images. Note: images aren't actually
    being published onto a topic, given that the frame rate was too slow,
    so this lifecycle node instead utilizes UDP sockets. 

    Attributes:
        camera_1_name (STRING):         By default, this is the turret view

        remote_server_ip_1 (STRING):    By default, the remote IP of device wanting
                                        to take on Stroam's turret view

        remote_port_1 (INTEGER):        By default, the remote port of remote device

        is_active (bool):               Used by shutdown or on_deactivate main threads to notify 
                                        the working publishing thread (publish_images) to stop
                                        
        publisher_thread(Thread):       The worker thread reponsible for 'publishing'
                                        images when camera node is in active state
                                        
        videoIndex_ (INTEGER):          Index of device of video usb

        video_capture_:                 cv2 object for image capturing 
        
        video_client_UDP_socket:        Socket used for sending images
    """

    def __init__(self):
        super().__init__("camera_publisher")

        self.declare_parameters(
                namespace='',
                parameters=[
                    ("camera_view_1", rclpy.Parameter.Type.STRING),
                    ("remote_server_ip_1", rclpy.Parameter.Type.STRING),
                    ("remote_port_1", rclpy.Parameter.Type.INTEGER)
                ])
        self.camera_1_name = self.get_parameter("camera_view_1").value
        self.remote_server_ip_ = self.get_parameter("remote_server_ip_1").value
        self.server_port_ = self.get_parameter("remote_port_1").value

        self.is_active_ = False
        self.publisher_thread_ = None
        self.videoIndex_ = 0
        self.video_capture_ = None

        self.video_client_UDP_socket_ = None

        self.get_logger().info("camera node initialized")
    
    #Create ROS2 communications, connect to camera
    def on_configure(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_configure")
        self.video_capture_ = cv2.VideoCapture(self.videoIndex_)

        return TransitionCallbackReturn.SUCCESS

    #Enable camera
    def on_activate(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_activate")

        setup_pass = self.setup_UDP_socket()
        if not setup_pass: 
           return TransitionCallbackReturn.FAILURE

        self.is_active_ = True
        self.publisher_thread_ = threading.Thread(target=self.publish_images)
        self.publisher_thread_.start()

        return super().on_activate(previous_state)  

    def setup_UDP_socket(self) -> bool:
        setup_successful = True
        try:
            self.video_client_UDP_socket_ = socket.socket(socket.AF_INET, 
                    socket.SOCK_DGRAM)
            self.video_client_UDP_socket_.setsockopt(socket.SOL_SOCKET, 
                    socket.SO_SNDBUF,1000000)
        except socket.error as e:
            self.get_logger().info("Failed to setup socket")
            setup_successful = False

        return setup_successful 

    #Disable camera
    def on_deactivate(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_deactivate")
        self.is_active_ = False
        return super().on_deactivate(previous_state)

    #Destroy ROS2 communications, disconnect from camera
    def on_cleanup(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_cleanup")
        if self.video_capture_:
            self.video_capture_.release()

        return TransitionCallbackReturn.SUCCESS
    
    def on_shutdown(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_shutdown")
        if self.video_capture_:
            self.video_capture_.release()
        if self.is_active_:
            self.is_active_ = False

        return TransitionCallbackReturn.SUCCESS

    def publish_images(self):
        while self.is_active_: 
            got_image_pass, frame = self.video_capture_.read()

            if got_image_pass:
                ret, buffer = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 30])

                x_as_bytes = pickle.dumps(buffer)     
                try:
                    self.video_client_UDP_socket_.sendto((x_as_bytes),(self.remote_server_ip_,self.server_port_))
                except socket.error as e:
                    self.get_logger().info("Failed to send frame")
                    self.is_active = False

        self.release_resources() 

    def release_resources(self):
        self.get_logger().info("Closing client connection")
        self.video_client_UDP_socket_.close()

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)
    rclpy.shutdown()

if __name__ == "__main___":
    main()
