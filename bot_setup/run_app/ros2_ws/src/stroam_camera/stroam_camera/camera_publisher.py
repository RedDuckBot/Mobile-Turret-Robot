#!/usr/bin/env python3

import rclpy
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn, LifecycleNode
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraNode(LifecycleNode):
    def __init__(self):
        super().__init__("camer_publisher")
        self.imageConverter = CvBridge()
        self.videoIndex = 0
        self.publisherFrequency = 1.0
        self.camera_publisher_ = None
        self.camera_pub_buffer_num = 10
        self.video_capture_ = None
        self.camera_timer_ = None

        self.get_logger().info("camera node initialized")
    
    #Create ROS2 communications, connect to camera
    def on_configure(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_configure")
        self.camera_publisher_ = self.create_lifecycle_publisher(Image,"stroam_images",
                self.camera_pub_buffer_num)
        self.video_capture = cv2.VideoCapture(videoIndex)
        self.camera_timer_ = self.create_timer(publisherFrequency, self.publish_image)

        return TransitionCallbackReturn.SUCCESS

    def publish_image(self):
        got_image_pass, frame = self.video_capture.read()

        if got_image_pass:
            ros_image = self.imageConverter.cv2_to_imgmsg(frame, encoding="bgr8")
            self.camera_publisher_.publish(ros_image)
        else:
            raise RuntTimeError("Failed to capture image from camera")
         
    #Enable camera
    def on_activate(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_activate")
        return super().on_activate(previous_state)  
    
    #Disable camera
    def on_deactivate(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_deactivate")
        return super().on_deactivate(previous_state)

    #Destroy ROS2 communications, disconnect from camera
    def on_cleanup(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_cleanup")
        self.close_resources()

        return TransitionCallbackReturn.SUCCESS
    
    def on_shutdown(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_shutdown")
        self.close_resources()

        return TransitionCallbackReturn.SUCCESS
    
    #Utility function for on_cleanup & on_shutdown
    def close_resources():
        self.destroy_lifecycle_publisher(self.camera_publisher_)
        self.destroy_timer(self.camera_timer_)
        self.video_capture.close()


def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)
    rclpy.shutdown()

if __name__ == "__main___":
    main()
