import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


"""Node Classed used to display Stroam's view"""
class ImageViewerNode(Node):
    def __init__(self):
        super().__init__("camera_sub")

        self.declare_parameter("camera_name", rclpy.Parameter.Type.STRING)
        self.camera_node_name = self.get_parameter("camera_name").value
        camera_topic = "/" + self.camera_node_name + "_images"

        self.image_converter = CvBridge()
        self.camera_sub = self.create_subscription(Image, camera_topic,
                            self.display_image_callback, 10)

    def display_image_callback(self, msg):
        frame = self.image_converter.imgmsg_to_cv2(msg,"bgr8")

        cv2.imshow(f"{self.camera_node_name} View", frame)
        cv2.waitKey(1) #Keep window open
        

def main(args=None):
    rclpy.init(args=args)
    image_viewer_node = ImageViewerNode()
    rclpy.spin(image_viewer_node)
    image_viewer_node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()