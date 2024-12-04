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

from geometry_msgs.msg import Point, Quaternion, Pose, TransformStamped, Vector3

from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from doodle_droid.motion_planner import MotionPlanner
from std_srvs.srv import Empty
from action_msgs.msg import GoalStatus



def angle_from_centroid(pt, centroid):
    dx = pt[0] - centroid[0]
    dy = pt[1] - centroid[1]
    return np.arctan2(dy, dx)

def distance(p1,p2):
        return np.linalg.norm(p1-p2)


class Calibrator(Node):
    """

    """

    def __init__(self):
        """Run initializer."""
        super().__init__('calibrator')

        self.frequency = 1000.0

        # Timers
        self.create_timer(1/self.frequency, self.timer_callback)

        self.surface_publisher = self.create_publisher(Marker, 'surface_marker', 10)
        self.drawing_dims_publisher = self.create_publisher(Vector3, "drawing_dims", 10)
        self.cam_sub = self.create_subscription(CompressedImage, '/image_raw/compressed', self.get_image_callback, 10)

        self.broadcaster = TransformBroadcaster(self)

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
        world_camera_tf.header.frame_id = 'end_effector'
        world_camera_tf.child_frame_id = 'camera'
        world_camera_tf.transform.translation.x = 0.0 # change to match camera mounting
        world_camera_tf.transform.translation.y = 0.0
        world_camera_tf.transform.translation.z = 0.0
        self.static_broadcaster.sendTransform(world_camera_tf)

        self.motion_planner = MotionPlanner(self)
        self.in_position = False

        self.calibrate_server = self.create_service(Empty, "calibrate", self.calibrate_callback)


        self.get_logger().info("calibrator initialized")

    

    def timer_callback(self):
        """
        Run the main timer for controlling the calibrator.

        """
        # self.get_logger().info("not in position")

        if self.in_position:
            # self.get_logger().info("IN POSITION")
            if self.current_image is not None:
                cv_image = self.bridge.compressed_imgmsg_to_cv2(self.current_image, desired_encoding='passthrough')
                gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                detections = self.detector.detect(gray)
                
                detection_num = 0
                detected_orientations = []
                detected_positions = []
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
                        detected_positions.append(reshaped_vector)

                        self.surface_pose = pose

                        surface = self.create_marker(detection_num, 'surface', 'camera', pose, [self.tagsize, self.tagsize, 0.1], [1.0, 1.0, 1.0], 0.5)
                        self.surface_publisher.publish(surface)

                        detection_num += 1

                    avg_position = np.mean(detected_positions, axis=0) # calculate average position of four april tags (hopefully more accurate)
                    self.surface_pose.position.x = avg_position[0]
                    self.surface_pose.position.y = avg_position[1]
                    self.surface_pose.position.z = avg_position[2]
                    

                    avg_orientation = np.mean(detected_orientations, axis=0) # calculate average orientation of four april tags (hopefully more accurate)
                    avg_orientation /= np.linalg.norm(avg_orientation)
                    self.surface_pose.orientation.x = avg_orientation[0]
                    self.surface_pose.orientation.y = avg_orientation[1]
                    self.surface_pose.orientation.z = avg_orientation[2]
                    self.surface_pose.orientation.w = avg_orientation[3]

                    if len(detected_positions) >= 4:
                        sorted_positions = sorted(
                            detected_positions,
                            key=lambda pt: angle_from_centroid(pt, avg_position)
    )                        # orders centers as bot left, bot right, top right, top left
                        width = distance(sorted_positions[0], sorted_positions[1])
                        height = distance(sorted_positions[0], sorted_positions[3])

                        dims = Vector3()
                        dims.x = width
                        dims.y = height
                        self.drawing_dims_publisher.publish(dims)

                        cam_surface = TransformStamped()
                        cam_surface.header.stamp = self.get_clock().now().to_msg()
                        cam_surface.header.frame_id = 'camera'
                        cam_surface.child_frame_id = 'surface'
                        cam_surface.transform.translation.x = reshaped_vector[0]
                        cam_surface.transform.translation.y = reshaped_vector[1]
                        cam_surface.transform.translation.z = reshaped_vector[2]
                        cam_surface.transform.rotation.x = quaternion[0]
                        cam_surface.transform.rotation.y = quaternion[1]
                        cam_surface.transform.rotation.z = quaternion[2]
                        cam_surface.transform.rotation.w = quaternion[3]

                        self.broadcaster.sendTransform(cam_surface)

                        surface = self.create_marker(999, 'surface', 'camera', self.surface_pose, [height-self.tagsize/2, width-self.tagsize/2, 0.1], [0.0, 1.0, 1.0], 1.0)
                        self.surface_publisher.publish(surface)
    
    async def calibrate_callback(self,request, response):
        # z = 0.188
        start1 = Pose()
        start1.position = Point(x=0.45, y=0.05, z=0.75)
  
        start1.orientation = Quaternion(x=0.9238792,
                                        y=-0.3826833,
                                        z=0.0003047,
                                        w=0.0007357)
        result, status = await self.motion_planner.plan_p(start1.position,start1.orientation,execute=True)

        self.motion_planner.print_status(status)
        # while status != GoalStatus.STATUS_SUCCEEDED:
        #     self.get_logger().info('planning step 1')
        #     # self.motion_planner.print_status(status)
        #     pass
        self.in_position = True


        return response


    def get_image_callback(self, msg):
        self.current_image = msg

    def create_marker(self, m_id, name, frame, pose, scale, col, a):
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
        marker.color.a = a

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
