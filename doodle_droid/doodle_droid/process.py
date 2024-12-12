import doodle_droid.linedraw.linedraw
import rclpy
import rclpy.parameter
from rclpy.node import Node
import rclpy.time

from sensor_msgs.msg import CompressedImage
from std_srvs.srv import Empty
from cv_bridge import CvBridge 
import cv2 as cv
from doodle_droid.linedraw.linedraw import *
from std_msgs.msg import String
import json
from ament_index_python.packages import get_package_share_directory
import rospkg



class ImageProcessingNode(Node):
    def __init__(self):
        super().__init__('image_processing_node')

        self.image_sub = self.create_subscription(CompressedImage, '/image_raw/compressed', self.get_image_callback, 10)
        self.take_picture_service = self.create_service(Empty, '/take_picture', self.capture_image)
        self.processed_image_pub = self.create_publisher(String, '/new_image', 10)
        self.current_image = None
        self.pkg_name = "doodle_droid"
        self.pkg_share = get_package_share_directory(self.pkg_name)

        self.cascade_path = f'{self.pkg_share}/config/haarcascade_frontalface_default.xml'
        self.face_cascade = cv2.CascadeClassifier(self.cascade_path)
        
        # self.path = f"{self.pkg_share}/images/my_smiley.jpeg"
        #self.path = f"{self.pkg_share}/images/matt.png"
        # self.path = f"{self.pkg_share}/images/face_image.png"
        # self.path = f"{self.pkg_share}/images/apple.png"
        # self.path = f"{self.pkg_share}/images/dog.png"
        # self.path = f"{self.pkg_share}/images/NU_Logo.png"
        # self.path = f"{self.pkg_share}/images/luffy_face.png"
        # self.path = f"{self.pkg_share}/images/luffy.jpg"
        self.path = f"{self.pkg_share}/images/totoro_2.jpg"
        self.bridge = CvBridge()
        self.from_file = False
        
        self.get_logger().info(f"SELF.PS: {self.pkg_share}")
        self.get_logger().info(f"SELF.PATH: {self.path}")
    


    def capture_face(self, cv_image):
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)

        for (x, y, w, h) in faces:
            # Increase the box size upwards, downwards, and slightly to the sides
            margin_up = 50 
            margin_down = 30  
            margin_side = 13

            # Adjust the coordinates of the bounding box
            y = max(y - margin_up, 0)  
            x = max(x - margin_side, 0)  
            w = w + 2 * margin_side  
            h = h + margin_up + margin_down  

            # Sharpen the face area
            face_area = gray[y:y+h, x:x+w]

            kernel = np.array([[0, -1, 0],
                       [-1, 5,-1],
                       [0, -1, 0]])
            face_area_sharpened = cv2.filter2D(face_area, -1, kernel)
            face_area_blurred = cv2.GaussianBlur(face_area_sharpened,(3,3),1)

            edges = cv2.Canny(face_area, 100, 200)
            face_area_thickened = cv2.addWeighted(face_area_sharpened, 0.5, edges, 0.5, 0)

            laplacian = cv2.Laplacian(face_area, cv2.CV_64F)
            laplacian_abs = cv2.convertScaleAbs(laplacian)
            face_area_thickened = cv2.addWeighted(face_area, 1.0, laplacian_abs, 0.5, 0)



            return face_area_sharpened


    def get_image_callback(self, msg):
        self.current_image = msg

    
    def capture_image(self, request, response):
        if not self.from_file:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(self.current_image, desired_encoding='passthrough')

            face_image = self.capture_face(cv_image)
            # Step 5: Generate line drawing
            try:
                lined_image = doodle_droid.linedraw.linedraw.sketch(face_image)
                self.get_logger().info(f"Number of strokes: {len(lined_image)}")
                self.get_logger().info("Finished processing")
            except:
                self.get_logger().info("NO FACE")
            
        else:
            image = cv.imread(self.path)
            image_array = np.array(image)
            lined_image = doodle_droid.linedraw.linedraw.sketch(image_array)
            self.get_logger().info(f"Number of strokes: {len(lined_image)} ")
            
            self.get_logger().info(f"Number of strokes: {len(lined_image)} ")
            self.get_logger().info("Finished processing")
            
        # self.get_logger().info(f"lined image: {lined_image}")
            
        all_values = [value for sublist in lined_image for tuple_item in sublist for value in tuple_item]

        # Get min and max for each component
        min_val = min(all_values)
        max_val = max(all_values)

        # Step 2: Normalize each tuple
        normalized_data = []
        for sublist in lined_image:
            normalized_sublist = []
            for value in sublist:
                # Normalize each component of the tuple separately
                normalized_tuple = tuple((component - min_val) / (max_val - min_val) for component in value)
                normalized_sublist.append(normalized_tuple)
                
            normalized_data.append(normalized_sublist)

        # self.get_logger().info(f"Normalized list to publish: {normalized_data}")
        
        json_data = json.dumps(normalized_data)
        msg = String()
        msg.data = json_data
        self.processed_image_pub.publish(msg)
        
        self.get_logger().info(f"DONE PUBLISHING")
              
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