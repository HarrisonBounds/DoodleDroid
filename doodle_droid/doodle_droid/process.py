import doodle_droid.linedraw.linedraw
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_srvs.srv import Empty
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
import json
from ament_index_python.packages import get_package_share_directory

class ImageProcessingNode(Node):
    """
    ROS2 Node for image processing and line drawing generation.

    This node subscribes to a compressed image topic, processes the image to detect faces,
    applies transformations, and generates a line drawing representation of the detected
    face or image. The processed data is published as a normalized JSON string.
    """

    def __init__(self):
        """
        Initialize the ImageProcessingNode.
        """
        super().__init__('image_processing_node')

        # Subscription to compressed image topic
        self.image_sub = self.create_subscription(
            CompressedImage, '/image_raw/compressed', self.get_image_callback, 10
        )

        # Service to capture and process an image
        self.take_picture_service = self.create_service(Empty, '/take_picture', self.capture_image)

        # Publisher for processed image data
        self.processed_image_pub = self.create_publisher(String, '/new_image', 10)

        self.current_image = None  # Current image buffer
        self.pkg_name = "doodle_droid"  # Package name
        self.pkg_share = get_package_share_directory(self.pkg_name)  # Package share directory
        self.normalized_data = []  # Normalized image data buffer

        # Load Haar cascade for face detection
        self.cascade_path = f'{self.pkg_share}/config/haarcascade_frontalface_default.xml'
        self.face_cascade = cv.CascadeClassifier(self.cascade_path)

        # Example image path for testing
        self.path = f"{self.pkg_share}/images/totoro_2.jpg"
        self.bridge = CvBridge()  # Bridge for ROS2 and OpenCV
        self.from_file = False  # Flag to determine image source

        # Log initialization details
        self.get_logger().info(f"Package Share Directory: {self.pkg_share}")
        self.get_logger().info(f"Test Image Path: {self.path}")

    def capture_face(self, cv_image):
        """
        Detect and enhance faces in the given OpenCV image.

        Args:
            cv_image (numpy.ndarray): The input image in OpenCV format.

        Returns:
            numpy.ndarray: The enhanced face region or None if no face is found.
        """
        gray = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)  # Convert to grayscale
        faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)  # Detect faces

        for (x, y, w, h) in faces:
            # Add margins to the bounding box
            margin_up = 50
            margin_down = 30
            margin_side = 13
            y = max(y - margin_up, 0)
            x = max(x - margin_side, 0)
            w = w + 2 * margin_side
            h = h + margin_up + margin_down

            # Enhance the face area
            face_area = gray[y:y+h, x:x+w]
            kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])
            face_area_sharpened = cv.filter2D(face_area, -1, kernel)

            return face_area_sharpened

    def get_image_callback(self, msg):
        """
        Callback for receiving images from the subscribed topic.

        Args:
            msg (CompressedImage): The received image message.
        """
        self.current_image = msg

    def capture_image(self, request, response):
        """
        Service callback to capture, process, and publish the processed image.

        Args:
            request: Empty service request.
            response: Empty service response.

        Returns:
            response: The service response.
        """
        if not self.from_file:
            # Process image from subscription
            cv_image = self.bridge.compressed_imgmsg_to_cv2(
                self.current_image, desired_encoding='passthrough'
            )
            face_image = self.capture_face(cv_image)

            try:
                lined_image = doodle_droid.linedraw.linedraw.sketch(face_image)
                self.get_logger().info(f"Number of strokes: {len(lined_image)}")
            except Exception as e:
                self.get_logger().info(f"Error during processing: {e}")
        else:
            # Process image from file
            image = cv.imread(self.path)
            lined_image = doodle_droid.linedraw.linedraw.sketch(np.array(image))
            self.get_logger().info(f"Number of strokes: {len(lined_image)}")

        # Normalize line drawing data
        all_values = [value for sublist in lined_image for tuple_item in sublist for value in tuple_item]
        min_val, max_val = min(all_values), max(all_values)
        self.normalized_data = [
            [tuple((comp - min_val) / (max_val - min_val) for comp in value) for value in sublist]
            for sublist in lined_image
        ]

        # Publish processed data
        json_data = json.dumps(self.normalized_data)
        msg = String()
        msg.data = json_data
        self.processed_image_pub.publish(msg)
        self.get_logger().info("Finished publishing processed image data.")

        return response

    def accept_image(self, request, response):
        """
        Publish previously normalized data if available.

        Args:
            request: Incoming service request containing data flag.
            response: Outgoing service response indicating success or failure.

        Returns:
            response: The service response.
        """
        if request.data:
            json_data = json.dumps(self.normalized_data)
            msg = String()
            msg.data = json_data
            self.processed_image_pub.publish(msg)
            self.get_logger().info("Re-published normalized image data.")
            response.success = True
        else:
            response.success = False

        return response


def main(args=None):
    """
    Main function to initialize and run the ImageProcessingNode.

    Args:
        args: Command-line arguments passed to the node.
    """
    rclpy.init(args=args)
    node = ImageProcessingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    import sys
    main(sys.argv)
