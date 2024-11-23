import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from std_srvs.srv import Empty
from doodle_droid.linedraw.linedraw import sketch
from doodle_droid.linedraw.linedraw import visualize
from doodle_droid.linedraw.linedraw import makesvg
from doodle_droid.linedraw import *
import cv2 as cv
from PIL import Image as im 

class Image_Processing_Node(Node):
    def __init__(self):
        super().__init__('image_processing_node')
        self.image_sub = self.create_subscription(CompressedImage, '/image_raw/compressed', self.image_processing_callback, 10)
        self.take_picture = self.create_service(Empty, '/take_picture', self.get_image)
        self.pub = self.create_publisher(CompressedImage, '/new_image', 10)
        self.bridge = CvBridge()
        self.image_name_jpg = 'jpg_images/output_image.jpg'
        self.new_image = None
        
    def image_processing_callback(self, msg):
        self.old_image = msg
    
    def get_image(self, request, response):
        self.new_image = self.old_image
        self.linedraw_func()
        return response
        
    def linedraw_func(self):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(self.new_image, desired_encoding='passthrough')
        cv.imwrite(self.image_name_jpg, cv_image)
        lines = sketch(self.image_name_jpg)
        
        # lined_image = self.bridge.cv2_to_imgmsg(lines, encoding='bgr8')
        # self.pub.publish(lined_image)
        
def main(args=None):
    """Entrypoint for the mynode ROS node."""
    rclpy.init(args=args)
    node = Image_Processing_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    import sys
    main(sys.argv)