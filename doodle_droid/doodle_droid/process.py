import rclpy
import rclpy.parameter
from rclpy.node import Node
import rclpy.time

from sensor_msgs.msg import CompressedImage
from std_srvs.srv import Empty
from cv_bridge import CvBridge
import cv2 as cv
from doodle_droid.linedraw.linedraw import *


class ImageProcessingNode(Node):
    def __init__(self):
        super().__init__('image_processing_node')

        self.image_sub = self.create_subscription(CompressedImage, '/image_raw/compressed', self.get_image_callback, 10)
        self.take_picture_service = self.create_service(Empty, '/take_picture', self.capture_image)
        self.processed_image_pub = self.create_publisher(CompressedImage, '/new_image', 10)
        self.current_image = None
        self.absolute_path = '/home/harrison-bounds/ws/ES_HW/doodle_droid/src/DoodleDroid/doodle_droid/doodle_droid/images/output.jpg'
        self.bridge = CvBridge()
        
    def get_image_callback(self, msg):
        self.current_image = msg
    
    def capture_image(self, request, response):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(self.current_image, desired_encoding='passthrough')
        cv.imwrite(self.absolute_path, cv_image) #Convert numpy array tp jpg for image processing
        self.get_logger().info("Begin processing")
        lined_image = doodle_droid.linedraw.linedraw.sketch(self.absolute_path)
        self.get_logger().info("Finished processing")
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    import sys
    main(sys.argv)