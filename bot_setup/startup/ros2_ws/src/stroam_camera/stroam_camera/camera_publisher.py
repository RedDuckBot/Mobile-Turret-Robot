#!/usr/bin/env python3

import rclpy
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn, LifecycleNode
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraNode(LifecycleNode):
    def __init__(self):
        super().__init__("camera_publisher")

        self.declare_parameter("camera_name", rclpy.Parameter.Type.STRING)
        camera_name = self.get_parameter("camera_name").value
        self.camera_topic_name = camera_name + "_images"

        self.imageConverter = CvBridge()
        self.videoIndex = 0
        self.camera_publisher_ = None
        self.camera_pub_buffer_num = 10
        self.publisherFrequency = 1.0 / self.camera_pub_buffer_num
        self.video_capture_ = None
        self.camera_timer_ = None
        self.timer_running_ = False 

        self.camera_publisher_ = self.create_lifecycle_publisher(Image,
            self.camera_topic_name, self.camera_pub_buffer_num)

        self.get_logger().info("camera node initialized")
    
    #Create ROS2 communications, connect to camera
    def on_configure(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_configure")
        self.video_capture_ = cv2.VideoCapture(self.videoIndex)
        #self.video_capture_.set(3,640)
        #self.video_capture_.set(4,480)

        return TransitionCallbackReturn.SUCCESS

    #Enable camera
    def on_activate(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_activate")
        self.camera_timer_ = self.create_timer(self.publisherFrequency, self.timer_callback)
        self.timer_running_ = True
        return super().on_activate(previous_state)  

    #Disable camera
    def on_deactivate(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_deactivate")
        self.close_timer() 
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
        self.close_timer()
        self.destroy_lifecycle_publisher(self.camera_publisher_)

        return TransitionCallbackReturn.SUCCESS

    def close_timer(self):
        if self.timer_running_:
            self.destroy_timer(self.camera_timer_)
            self.timer_running_ = False

    def timer_callback(self):
        got_image_pass, frame = self.video_capture_.read()

        if got_image_pass:
            ros_image = self.imageConverter.cv2_to_imgmsg(frame, encoding="bgr8")
            self.camera_publisher_.publish(ros_image)
    

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)
    rclpy.shutdown()

if __name__ == "__main___":
    main()
