"""
Calibrates the paper position relative to the robot arm.

Publishers
----------
  + surface_pose geometry_msgs/msg/Pose - The paper surface pose

Subscribers
-----------
  + /realsense/realsense/color/image_rect sensor_msgs/msg/Image - Rectified image from realsense

Servers
-------
  + calibrate std_srvs/srv/Empty - Moves robot to calibration position and detects paper pose
  + test_calibrate std_srvs/srv/Empty - Moves robot to previously detected paper pose
  + manual_calibrate std_srvs/srv/Empty - Saves current robot position as paper pose
"""

from cv_bridge import CvBridge

from doodle_droid.motion_planner import MotionPlanner
from doodle_droid.robot_state import RobotState

from geometry_msgs.msg import Point, Pose, Quaternion, TransformStamped

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_srvs.srv import Empty

import tf2_ros
from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_listener import TransformListener


class Calibrator(Node):
    """
    Calibrates paper position relative to robot.

    Detects apriltags in realsense image and publishes poses to TF tree
    """

    def __init__(self):
        """Run initializer."""
        super().__init__('calibrator')

        self.frequency = 1000.0

        # Timers
        self.create_timer(1/self.frequency, self.timer_callback)

        self.surface_pose_publisher = self.create_publisher(Pose, 'surface_pose', 10)
        self.cam_sub = self.create_subscription(
            Image, '/realsense/realsense/color/image_rect', self.get_image_callback, 10)

        self.broadcaster = TransformBroadcaster(self)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.bridge = CvBridge()
        self.current_image = None
        self.surface_pose = None

        self.positions = []
        self.orientations = []
        self.pose = Pose()
        self.pose_determined = False

        self.tagsize = 0.1016
        self.pen_offset = 0.149

        self.static_broadcaster = StaticTransformBroadcaster(self)
        world_camera_tf = TransformStamped()
        world_camera_tf.header.stamp = self.get_clock().now().to_msg()
        world_camera_tf.header.frame_id = 'fer_hand'
        world_camera_tf.child_frame_id = 'realsense_link'
        world_camera_tf.transform.translation.x = 0.04077700478326539
        world_camera_tf.transform.translation.y = 0.01631535438771024
        world_camera_tf.transform.translation.z = 0.015104079163571737 + 0.021

        world_camera_tf.transform.rotation.x = -0.007100127498884945
        world_camera_tf.transform.rotation.y = -0.711166685920879
        world_camera_tf.transform.rotation.z = -0.005910717653049716
        world_camera_tf.transform.rotation.w = 0.7029627276340044

        self.static_broadcaster.sendTransform(world_camera_tf)

        self.motion_planner = MotionPlanner(self)
        self.robot_state = RobotState(self)
        self.in_position = False
        self.surface_published = False

        self.calibrate_server = self.create_service(
            Empty, 'calibrate', self.calibrate_callback)
        self.manual_calibrate_server = self.create_service(
            Empty, 'manual_calibrate', self.manual_calibrate_callback)
        self.test_calibrate_server = self.create_service(
            Empty, 'test_calibrate', self.test_calibrate_callback)

        self.get_logger().info('calibrator initialized')

    def timer_callback(self):
        """
        Run the main timer for controlling the calibrator.

        Once in position, collect apriltag poses from realsense image
        Average collected poses and publish to surface_pose topic
        """
        if self.in_position and not self.surface_published:
            if self.current_image is not None:
                try:
                    base_tag_tf = self.buffer.lookup_transform('base', 'tag0', rclpy.time.Time())
                    pose = Pose()
                    pose.position.x = base_tag_tf.transform.translation.x
                    pose.position.y = base_tag_tf.transform.translation.y
                    pose.position.z = base_tag_tf.transform.translation.z
                    pose.orientation = base_tag_tf.transform.rotation

                    self.positions.append(
                        np.array([pose.position.x, pose.position.y, pose.position.z]))
                    self.orientations.append(
                        np.array([pose.orientation.x,
                                  pose.orientation.y,
                                  pose.orientation.z,
                                  pose.orientation.w]))

                    base_tag_tf = self.buffer.lookup_transform('base', 'tag1', rclpy.time.Time())
                    pose.position.x = base_tag_tf.transform.translation.x
                    pose.position.y = base_tag_tf.transform.translation.y
                    pose.position.z = base_tag_tf.transform.translation.z
                    pose.orientation = base_tag_tf.transform.rotation

                    self.positions.append(np.array([pose.position.x,
                                                    pose.position.y,
                                                    pose.position.z]))
                    self.orientations.append(np.array([pose.orientation.x,
                                                       pose.orientation.y,
                                                       pose.orientation.z,
                                                       pose.orientation.w]))

                    base_tag_tf = self.buffer.lookup_transform('base', 'tag2', rclpy.time.Time())
                    pose.position.x = base_tag_tf.transform.translation.x
                    pose.position.y = base_tag_tf.transform.translation.y
                    pose.position.z = base_tag_tf.transform.translation.z
                    pose.orientation = base_tag_tf.transform.rotation

                    self.positions.append(np.array([pose.position.x,
                                                    pose.position.y,
                                                    pose.position.z]))
                    self.orientations.append(np.array([pose.orientation.x,
                                                       pose.orientation.y,
                                                       pose.orientation.z,
                                                       pose.orientation.w]))

                    base_tag_tf = self.buffer.lookup_transform('base', 'tag3', rclpy.time.Time())
                    pose.position.x = base_tag_tf.transform.translation.x
                    pose.position.y = base_tag_tf.transform.translation.y
                    pose.position.z = base_tag_tf.transform.translation.z
                    pose.orientation = base_tag_tf.transform.rotation

                    self.positions.append(np.array([pose.position.x,
                                                    pose.position.y,
                                                    pose.position.z]))
                    self.orientations.append(np.array([pose.orientation.x,
                                                       pose.orientation.y,
                                                       pose.orientation.z,
                                                       pose.orientation.w]))

                    if len(self.positions) > 119:

                        pose = Pose()

                        avg_position = np.mean(self.positions, axis=0)
                        self.pose.position.x = avg_position[0]
                        self.pose.position.y = avg_position[1]
                        self.pose.position.z = avg_position[2]

                        avg_orientation = np.mean(self.orientations, axis=0)
                        avg_orientation /= np.linalg.norm(avg_orientation)
                        self.pose.orientation.x = avg_orientation[0]
                        self.pose.orientation.y = avg_orientation[1]
                        self.pose.orientation.z = avg_orientation[2]
                        self.pose.orientation.w = avg_orientation[3]

                        self.get_logger().info('x ' + str(self.pose.position.x))
                        self.get_logger().info('y ' + str(self.pose.position.y))
                        self.get_logger().info('z ' + str(self.pose.position.z))
                        self.get_logger().info('\n')

                        publish_pose = self.pose
                        publish_pose.position.z += self.pen_offset
                        self.surface_pose_publisher.publish(publish_pose)
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

    async def calibrate_callback(self, request, response):
        """
        Run callback function for the calibrate service.

        Moves the robot into calibration position

        Args
        ----
            request (std_srvs/srv/Empty): Empty request

            response (Empty): Empty

        Returns
        -------
            An empty response

        """
        start1 = Pose()
        start1.position = Point(x=0.5, y=0.0, z=0.75)

        start1.orientation = Quaternion(x=0.9238792,
                                        y=-0.3826833,
                                        z=0.0003047,
                                        w=0.0007357)
        result, status = await self.motion_planner.plan_p(start1.position,
                                                          start1.orientation,
                                                          execute=True)

        self.motion_planner.print_status(status)

        self.in_position = True
        self.surface_published = False
        self.positions = []
        self.orientations = []

        return response

    async def test_calibrate_callback(self, request, response):
        """
        Run callback function for the test_calibrate service.

        Moves the robot to previously found paper pose

        Args
        ----
            request (std_srvs/srv/Empty): Empty request

            response (Empty): Empty

        Returns
        -------
            An empty response

        """
        move_pose = self.pose
        move_pose.position.z += self.pen_offset

        move_pose.orientation = Quaternion(x=0.9238792,
                                           y=-0.3826833,
                                           z=0.0003047,
                                           w=0.0007357)
        result, status = await self.motion_planner.plan_c(move_pose, execute=True)

        return response

    async def manual_calibrate_callback(self, request, response):
        """
        Run callback function for the manual_calibrate service.

        Saves current robot position as calibrated paper pose

        Args
        ----
            request (std_srvs/srv/Empty): Empty request

            response (Empty): Empty

        Returns
        -------
            An empty response

        """
        self.pose = await self.robot_state.get_ee_pose()

        move_pose = self.pose
        move_pose.position.z -= 0.001

        move_pose.orientation = Quaternion(x=0.9238792,
                                           y=-0.3826833,
                                           z=0.0003047,
                                           w=0.0007357)
        result, status = await self.motion_planner.plan_c(move_pose, execute=True)

        self.get_logger().info('x ' + str(self.pose.position.x))
        self.get_logger().info('y ' + str(self.pose.position.y))
        self.get_logger().info('z ' + str(self.pose.position.z))
        self.get_logger().info('\n')

        return response

    def get_image_callback(self, msg):
        """
        Run callback function for the image_rect subscriber.

        Stores received iamge into local instance variable

        Args:
        ----
        msg (sensor_msgs/msg/Image): Contains the image from realsense

        """
        self.current_image = msg


def main(args=None):
    """Run main function."""
    rclpy.init(args=args)
    node = Calibrator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
