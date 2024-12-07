"""
Calibrator
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

import numpy as np
from apriltag import apriltag
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import Point, Quaternion, Pose, TransformStamped, Vector3

import tf2_ros
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# from tf2_msgs.

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
        self.surface_pose_publisher = self.create_publisher(Pose, "surface_pose", 10)
        self.cam_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.get_image_callback, 10)

        self.broadcaster = TransformBroadcaster(self)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.bridge = CvBridge()
        self.current_image = None
        self.surface_pose = None

        self.tagsize = 0.1016  # using 4 inch apriltags


        ##### MAGIC NUMBERS
        # self.static_broadcaster = StaticTransformBroadcaster(self)
        # world_camera_tf = TransformStamped()
        # world_camera_tf.header.stamp = self.get_clock().now().to_msg()
        # world_camera_tf.header.frame_id = 'fer_hand'
        # world_camera_tf.child_frame_id = 'camera_link'
        # world_camera_tf.transform.translation.x = 0.05366 # change to match camera mounting
        # world_camera_tf.transform.translation.y = -0.0193
        # world_camera_tf.transform.translation.z = 0.0025

        # world_camera_tf.transform.rotation.x = 0.0 # change to match camera mounting
        # world_camera_tf.transform.rotation.y = -0.7071045
        # world_camera_tf.transform.rotation.z = 0.0
        # world_camera_tf.transform.rotation.w = 0.7071045

        # self.static_broadcaster.sendTransform(world_camera_tf)

        self.static_broadcaster = StaticTransformBroadcaster(self)
        world_camera_tf = TransformStamped()
        world_camera_tf.header.stamp = self.get_clock().now().to_msg()
        world_camera_tf.header.frame_id = 'fer_hand'
        world_camera_tf.child_frame_id = 'camera_link'
        world_camera_tf.transform.translation.x = -0.01751038# change to match camera mounting
        world_camera_tf.transform.translation.y = 0.03731347
        world_camera_tf.transform.translation.z = -0.05544694

        world_camera_tf.transform.rotation.x = 0.00106468  # change to match camera mounting
        world_camera_tf.transform.rotation.y =  0.69706528
        world_camera_tf.transform.rotation.z = -0.0068949
        world_camera_tf.transform.rotation.w = 0.71697372

        self.static_broadcaster.sendTransform(world_camera_tf)



        ########## OBTAINED FROM HAND EYE CALIBRATION
        # self.static_broadcaster = StaticTransformBroadcaster(self)
        # world_camera_tf = TransformStamped()
        # world_camera_tf.header.stamp = self.get_clock().now().to_msg()
        # world_camera_tf.header.frame_id = 'fer_hand'
        # world_camera_tf.child_frame_id = 'camera_color_optical_frame'
        # world_camera_tf.transform.translation.x =  0.055954 # change to match camera mounting
        # world_camera_tf.transform.translation.y = 0.032108
        # world_camera_tf.transform.translation.z = 0.037019

        # world_camera_tf.transform.rotation.x = 0.008965 # change to match camera mounting
        # world_camera_tf.transform.rotation.y = -0.007981
        # world_camera_tf.transform.rotation.z = -0.713568
        # world_camera_tf.transform.rotation.w = 0.700483

        # self.static_broadcaster.sendTransform(world_camera_tf)



        self.motion_planner = MotionPlanner(self)
        self.in_position = False
        self.surface_published = False

        self.calibrate_server = self.create_service(Empty, "calibrate", self.calibrate_callback)


        self.get_logger().info("calibrator initialized")

    

    def timer_callback(self):
        """
        Run the main timer for controlling the calibrator.

        """
        # self.get_logger().info("not in position")
        
        if self.in_position and not self.surface_published:
            # self.get_logger().info("IN POSITION")
            if self.current_image is not None:
                # cv_image = self.bridge.compressed_imgmsg_to_cv2(self.current_image, desired_encoding='passthrough')
                # gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                
                try:
                    base_tag_tf = self.buffer.lookup_transform('base', 'tag', rclpy.time.Time())
                    pose = Pose()
                    pose.position.x = base_tag_tf.transform.translation.x
                    pose.position.y = base_tag_tf.transform.translation.y
                    pose.position.z = base_tag_tf.transform.translation.z
                    pose.orientation = base_tag_tf.transform.rotation

                    self.get_logger().info("x " + str(base_tag_tf.transform.translation.x) )
                    self.get_logger().info("y " + str(base_tag_tf.transform.translation.y) )
                    self.get_logger().info("z " + str(base_tag_tf.transform.translation.z) )
                    self.get_logger().info("\n")

                    self.surface_pose_publisher.publish(pose)
                    self.surface_published = True




                except tf2_ros.LookupException as e:
                    # the frames don't exist yet
                    self.get_logger().info(f'Lookup exception: {e}')
                except tf2_ros.ConnectivityException as e:
                    # the tf tree has a disconnection
                    self.get_logger().info(f'Connectivity exception: {e}')
                except tf2_ros.ExtrapolationException as e:
                    # the times are two far apart to extrapolate
                    self.get_logger().info(f'Extrapolation exception: {e}')
                pass
            # detections = self.detector.detect(gray)
            
       
    
    async def calibrate_callback(self,request, response):
        # z = 0.188


        try:
            cam_optical_tf = self.buffer.lookup_transform('camera_color_optical_frame', 'camera_link', rclpy.time.Time())
            pose = Pose()
            pose.position.x = cam_optical_tf.transform.translation.x
            pose.position.y = cam_optical_tf.transform.translation.y
            pose.position.z = cam_optical_tf.transform.translation.z
            pose.orientation = cam_optical_tf.transform.rotation

            self.get_logger().info("x " + str(cam_optical_tf.transform.translation.x) )
            self.get_logger().info("y " + str(cam_optical_tf.transform.translation.y) )
            self.get_logger().info("z " + str(cam_optical_tf.transform.translation.z) )
            self.get_logger().info("\n")


            self.get_logger().info("x " + str(cam_optical_tf.transform.rotation.x) )
            self.get_logger().info("y " + str(cam_optical_tf.transform.rotation.y) )
            self.get_logger().info("z " + str(cam_optical_tf.transform.rotation.z) )
            self.get_logger().info("w " + str(cam_optical_tf.transform.rotation.w) )
            self.get_logger().info("\n")

            self.surface_pose_publisher.publish(pose)
            self.surface_published = True




        except tf2_ros.LookupException as e:
            # the frames don't exist yet
            self.get_logger().info(f'Lookup exception: {e}')
        except tf2_ros.ConnectivityException as e:
            # the tf tree has a disconnection
            self.get_logger().info(f'Connectivity exception: {e}')
        except tf2_ros.ExtrapolationException as e:
            # the times are two far apart to extrapolate
            self.get_logger().info(f'Extrapolation exception: {e}')
        pass


        # start1 = Pose()
        # start1.position = Point(x=0.45, y=0.05, z=0.75)
  
        # start1.orientation = Quaternion(x=0.9238792,
        #                                 y=-0.3826833,
        #                                 z=0.0003047,
        #                                 w=0.0007357)
        # result, status = await self.motion_planner.plan_p(start1.position,start1.orientation,execute=True)

        # self.motion_planner.print_status(status)
        # # while status != GoalStatus.STATUS_SUCCEEDED:
        # #     self.get_logger().info('planning step 1')
        # #     # self.motion_planner.print_status(status)
        # #     pass
        # self.in_position = True


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
