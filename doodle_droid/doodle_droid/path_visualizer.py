"""Path Visualizer Module."""

import copy

from geometry_msgs.msg import PoseStamped, PoseArray, Pose

from nav_msgs.msg import Path

from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.node import Node

from std_srvs.srv import Empty

from tf2_ros import TypeException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from visualization_msgs.msg import Marker, MarkerArray


class PathVisualizer():
    """Path Visualizer Module."""

    def __init__(self,
                 node: Node):
        """
        Initialize the Path Visualizer Module.

        :param node: The ROS node
        :type node: rclpy.node.Node
        """
        # Inherit Node
        self._node = node

        # Frame Definitions
        self._base_frame = 'base'
        self._ee_frame = 'fer_link8'

        # Publish Path
        self._path_publisher = self._node.create_publisher(
            Path, 'doodle_droid_path', 10)
        self._path = Path()

        # Timer
        self._timer = self._node.create_timer(0.1, self._timer_callback)

    def set_visualizing_waypoints(self,
                                  waypoints: PoseArray,
                                  dt: float = 0.1,
                                  z_offset: float = 0.0):
        # Convert waypoints to path
        self._path.poses = []
        self._path.header.frame_id = self._base_frame
        self._path.header.stamp = self._node.get_clock().now().to_msg()
        for idx, pose in enumerate(waypoints.poses):
            pose_stamped = PoseStamped()
            pose_stamped.header = waypoints.header
            pose_stamped.pose = copy.deepcopy(pose)
            pose_stamped.pose.position.z += z_offset
            self._path.poses.append(pose_stamped)


    def _timer_callback(self):
        self._path_publisher.publish(self._path)