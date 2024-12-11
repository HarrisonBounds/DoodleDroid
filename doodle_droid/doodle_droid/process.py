import doodle_droid.linedraw.linedraw
import rclpy
import rclpy.parameter
from rclpy.node import Node
import rclpy.time
from sensor_msgs.msg import CompressedImage
from std_srvs.srv import Empty, SetBool
from cv_bridge import CvBridge
import cv2 as cv
from doodle_droid.linedraw.linedraw import *
from std_msgs.msg import String
import json
from ament_index_python.packages import get_package_share_directory


class ImageProcessingNode(Node):
    def __init__(self):
        super().__init__('image_processing_node')

        self.image_sub = self.create_subscription(CompressedImage, '/image_raw/compressed', self.get_image_callback, 10)
        self.take_picture_service = self.create_service(Empty, '/take_picture', self.capture_image)
        self.take_picture_service = self.create_service(SetBool, '/accept', self.accept_image)
        self.processed_image_pub = self.create_publisher(String, '/new_image', 10)
        self.current_image = None
        self.pkg_name = "doodle_droid"
        self.pkg_share = get_package_share_directory(self.pkg_name)
        
        # self.path = f"{self.pkg_share}/images/my_smiley.jpeg"
        self.path = f"{self.pkg_share}/images/luffy.jpg"
        self.bridge = CvBridge()
        self.from_file = True
        
        self.get_logger().info(f"SELF.PS: {self.pkg_share}")
        self.get_logger().info(f"SELF.PATH: {self.path}")
        
        self.msg = None
        self.normalized_data = []
        
    def get_image_callback(self, msg):
        self.current_image = msg
    
    def capture_image(self, request, response):
        if not self.from_file:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(self.current_image, desired_encoding='passthrough')
            cv.imshow("Raw Image", cv_image)
            cv.waitKey(1) 
            
            self.get_logger().info(f"cv image type: {type(cv_image)}")
            lined_image = doodle_droid.linedraw.linedraw.sketch(cv_image)
            
            self.get_logger().info(f"Number of strokes: {len(lined_image)} ")
            self.get_logger().info("Finished processing")
            
        else:
            image = cv.imread(self.path)
            image_array = np.array(image)
            cv.imshow("Raw Image", image)
            cv.waitKey(1) 
            
            lined_image = doodle_droid.linedraw.linedraw.sketch(image_array)
            
            self.get_logger().info(f"Number of strokes: {len(lined_image)} ")
            self.get_logger().info("Finished processing")
            
        all_values = [value for sublist in lined_image for tuple_item in sublist for value in tuple_item]

        # Get min and max for each component
        min_val = min(all_values)
        max_val = max(all_values)

        # Step 2: Normalize each tuple
        #normalized_data = []
        for sublist in lined_image:
            normalized_sublist = []
            for value in sublist:
                # Normalize each component of the tuple separately
                normalized_tuple = tuple((component - min_val) / (max_val - min_val) for component in value)
                normalized_sublist.append(normalized_tuple)
                
            self.normalized_data.append(normalized_sublist)
              
        return response
    
    def accept_image(self, request, response):
        if request.data:
            self.get_logger().info(f"Normalized list to publish: {self.normalized_data}")
        
            json_data = json.dumps(self.normalized_data)
            msg = String()
            msg.data = json_data
            self.processed_image_pub.publish(msg)
            
            self.get_logger().info(f"DONE PUBLISHING")
            
            response.success = True
            
        else:
            response.success = False
            
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