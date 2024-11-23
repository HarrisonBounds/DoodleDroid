import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from linedraw import linedraw


class Image_Processing_Node(Node):
    def __init__(self):
        super().__init__('image_processing_node')
        self.image_sub = self.create_subscription(Image, '/image_raw', callback=self.image_processing_callback)
        self.take_picture = self.create_service(Image, 'take_picture', self.get_image)
        self.pub = self.create_publisher(Image, 'new_image', 10)
        self.bridge = CvBridge()
        self.new_image = None
        

    def image_processing_callback(self, msg):
        self.old_image = msg
    
    def get_image(self):
        self.new_image = self.old_image
        self.linedraw()
        
    def linedraw(self):
        cv_image = self.bridge.imgmsg_to_cv2(self.new_image, desired_encoding='bgr8')
        lines = linedraw.sketch(cv_image)
        lined_image = self.bridge.cv2_to_imgmsg(lines, encoding='bgr8')
        
        self.pub.publish(lined_image)

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