"""
Calibrator
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

import numpy as np
from apriltag import apriltag
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import Pose, TransformStamped

from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class Calibrator(Node):
    """

    """

    def __init__(self):
        """Run initializer."""
        super().__init__('calibrator')

        self.frequency = 100.0

        # Timers
        self.create_timer(1/self.frequency, self.timer_callback)

        self.surface_publisher = self.create_publisher(Marker, 'surface_marker', 10)

        self.cam_sub = self.create_subscription(CompressedImage, '/image_raw/compressed', self.get_image_callback, 10)


        self.bridge = CvBridge()
        self.current_image = None
        self.surface_pose = None
        self.detected_poses = []
        self.detected_orientations = []
        self.detected_positions = []
        self.detector = apriltag("tagStandard41h12")
 
        self.fx = 600   # dummy parameters for now
        self.fy = 600 
        self.cx = 320  
        self.cy = 240  
        self.tagsize = 0.1016  # using 4 inch apriltags

        self.camera_matrix = np.array([[self.fx, 0, self.cx],
                                  [0, self.fy, self.cy],
                                  [0, 0, 1]])

        self.static_broadcaster = StaticTransformBroadcaster(self)
        world_camera_tf = TransformStamped()
        world_camera_tf.header.stamp = self.get_clock().now().to_msg()
        world_camera_tf.header.frame_id = 'world'
        world_camera_tf.child_frame_id = 'camera'
        world_camera_tf.transform.translation.x = 0.0
        world_camera_tf.transform.translation.y = 0.0
        world_camera_tf.transform.translation.z = 0.0
        self.static_broadcaster.sendTransform(world_camera_tf)

        self.get_logger().info("calibrator initialized")

    

    def timer_callback(self):
        """
        Run the main timer for controlling the calibrator.

        """
        if self.current_image is not None:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(self.current_image, desired_encoding='passthrough')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            detections = self.detector.detect(gray)
            
            detection_num = 0
            detected_orientations = []
            centers = []
            if len(detections)>0: # only look at first detection for now
                for detection in detections:
                    corners = np.array(detection['lb-rb-rt-lt'], dtype=np.float32)
                    # centers.append(detection['center'])
                    
                    object_points = np.array([
                        [-self.tagsize / 2, -self.tagsize / 2, 0],
                        [self.tagsize / 2, -self.tagsize / 2, 0],
                        [self.tagsize / 2, self.tagsize / 2, 0],
                        [-self.tagsize / 2, self.tagsize / 2, 0]
                    ], dtype=np.float32)

                    _, rotation_vector, translation_vector = cv2.solvePnP(object_points, corners, self.camera_matrix, None)

                    rotation_matrix, _ = cv2.Rodrigues(rotation_vector)

                    r = R.from_matrix(rotation_matrix)
                    quaternion = r.as_quat() 

                    pose = Pose()

                    reshaped_vector = translation_vector.reshape(3,)
                    pose.position.x = reshaped_vector[0]
                    pose.position.y = reshaped_vector[1]
                    pose.position.z = reshaped_vector[2]

                    pose.orientation.x = quaternion[0]
                    pose.orientation.y = quaternion[1]
                    pose.orientation.z = quaternion[2]
                    pose.orientation.w = quaternion[3]

                    detected_orientations.append(quaternion)

                    self.surface_pose = pose

                    surface = self.create_marker(detection_num, 'surface', 'camera', pose, [self.tagsize, self.tagsize, 0.1], [1.0, 1.0, 1.0])
                    self.surface_publisher.publish(surface)

                    detection_num += 1

                avg_orientation = np.mean(detected_orientations, axis=0) # calculate average orientation of four april tags (hopefully more accurate)
                avg_orientation /= np.linalg.norm(avg_orientation)
                self.surface_pose.orientation.x = avg_orientation[0]
                self.surface_pose.orientation.y = avg_orientation[1]
                self.surface_pose.orientation.z = avg_orientation[2]
                self.surface_pose.orientation.w = avg_orientation[3]

                if len(centers) >= 4:
                    centers = np.array(centers, dtype=np.int32)
                    centroid = np.mean(centers, axis=0).astype(float)
                    
                    def angle_from_centroid(center):
                        dx = center[0] - centroid[0]
                        dy = center[1] - centroid[1]
                        return np.arctan2(dy, dx)
                    
                    def distance(p1,p2):
                        return np.linalg.norm(p1-p2)
 
                    sorted_centers = sorted(centers, key=angle_from_centroid) # orders centers as bot left, bot right, top right, top left

                    # centers = np.array(sorted_centers)

                    width = distance(sorted_centers[0], sorted_centers[1])
                    height = distance(sorted_centers[0], sorted_centers[3])

                    self.get_logger().info(f"Width: {width:.2f}, Height: {height:.2f}")

                    # self.surface_pose.position.x = centroid[0]
                    # self.surface_pose.position.y = centroid[1]

                    # surface = self.create_marker(999, 'surface', 'camera', self.surface_pose, [2.0, 2.0, 0.1], [0.0, 1.0, 1.0])
                    # self.surface_publisher.publish(surface)

                    # centers = centers.reshape((-1, 1, 2))


                    # cv2.polylines(cv_image, [centers], isClosed=True, color=(0, 255, 0), thickness=2)

                    # # Draw the centroid (red circle)
                    # cv2.circle(cv_image, centroid, 5, (0, 0, 255), -1)  # Red circle for the centroid

                    # # Show the image
                    # cv2.imshow("Calibrator Image", cv_image)
                    # cv2.waitKey(1)

        if self.surface_pose is not None:
            surface = self.create_marker(999, 'surface', 'camera', self.surface_pose, [2.0, 2.0, 0.1], [0.0, 1.0, 1.0])
            self.surface_publisher.publish(surface)


    def get_image_callback(self, msg):
        self.current_image = msg

    def create_marker(self, m_id, name, frame, pose, scale, col):
        """
        Create a standard cube marker object for visualization.

        Args
        ----
            m_id (int) The unique id of the marker
            name (string) The name of the marker
            frame (string) The name of the marker's header frame
            pose (Pose) The pose of the marker
            scale ( [float, float, float] ) The scale of the marker
            col ( [float, float, float] ) Marker color

        Returns
        -------
            marker (visualization_msgs/msg/Marker) The generated marker

        """
        marker = Marker()

        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = frame
        marker.ns = name
        marker.id = m_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose = pose

        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]

        marker.color.r = col[0]
        marker.color.g = col[1]
        marker.color.b = col[2]
        marker.color.a = 1.0

        marker.lifetime = Duration(sec=0, nanosec=0)

        return marker


def main(args=None):
    """Run main function."""
    rclpy.init(args=args)
    node = Calibrator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
