"""
Route Planner Node

Converts individual lines into a sequence of waypoints for the robot to draw. Applies calibration to generate waypoints in the base frame. Uses The MotionPlanner class to interact with MoveIt.

The RobotState communicates through several ROS 2 protocols:

SUBSCRIBERS:
+ /surface_pose (geometry_msgs.msg.Pose) - origin of the drawing surface relative to base frame & orientation
+ /new_image (std_msgs.msg.String) - JSON string of lines to draw

SERVICES:
+ /draw (std_srvs.srv.Empty) - Initialize drawing of the previously published
+ /plot (std_srvs.srv.Empty) - generate a matplotlib plot of the drawing waypoints.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import  PoseArray
from geometry_msgs.msg import Point, Quaternion, Pose
from std_srvs.srv import Empty
from std_msgs.msg import String
import json

from ament_index_python.packages import get_package_share_directory

import numpy as np

from doodle_droid.paper_height_model import PlanePaperHeightModel
from doodle_droid.stroke_router_utils import stroke_dist, lines_to_endpoints, get_cost_matrix, tour_to_robot_waypoints, tsp_nearest_neighbor, plot_robot_waypoints
from doodle_droid.robot_state import RobotState
from doodle_droid.motion_planner import MotionPlanner
from doodle_droid.path_visualizer import PathVisualizer

from scipy.spatial.transform import Rotation as R

class RoutePlannerNode(Node):
    def __init__(self):
        """Construct the route planner node."""
        super().__init__('route_planner_node')
        self.pkg_name = "doodle_droid"

        self.pkg_share = get_package_share_directory(self.pkg_name)
        
        self._robot_state = RobotState(self)
        self._path_visualizer = PathVisualizer(self)
        self._motion_planner = MotionPlanner(self)

        self.paper_size = 0.125
        self.pose_offset = Pose(position=Point(x=0.4, y=0.0, z=0.20))
        self._calibation_pose_sub = self.create_subscription(Pose, "/surface_pose", self._update_offset, 10)

        self.paper_height_model = PlanePaperHeightModel(0, 0, 1, 0) # default to flat paper

        self._draw_waypoints = None
        self._draw_server = self.create_service(Empty, "/draw", self._draw_callback)
        self._plot_server = self.create_service(Empty, "/plot", self._plot_callback)


        self._brush_strokes_subscription = self.create_subscription(String, "/new_image", self._route_callback,10)

        
    def _update_offset(self, msg):
        """Update the offset of the drawing surface."""
        self.pose_offset = msg
    
    def _paper_height_callback(self, msg):
        """Update the paper height model."""
        self.get_logger().info(f"Received paper height model: {msg.data}")
        self.paper_height_model.update(*msg.data)
    
    def pose_waypoints_from_xyz(self, pts):
        """Convert a list of xyz points to a PoseArray."""
        waypoints = PoseArray()
        waypoints.header.frame_id = "base"
        waypoints.header.stamp = self.get_clock().now().to_msg()

        for pt in pts:
            pose = Pose()
            pose.position.x = pt[0]
            pose.position.y = pt[1]
            pose.position.z = pt[2]
            pose.orientation = Quaternion(x=0.9238792,
                                          y=-0.3826833,
                                          z=0.0003047,
                                          w=0.0007357)
            waypoints.poses.append(pose)

        return waypoints
    
    async def _execute_waypoints(self, pts):
        """Execute a list of waypoints using the motion planner and MoveIt"""
        waypoints = self.pose_waypoints_from_xyz(pts)
        self._path_visualizer.set_visualizing_waypoints(waypoints)
        pose_traj = await self._motion_planner._construct_joint_trajectory_from_waypoints(waypoints)
        self.get_logger().info(f"drawing time: {pose_traj.points[-1].time_from_start}s")
        
        start = waypoints.poses[0]

        cached_start_z = start.position.z
        start.position.z += 0.1
        await self._motion_planner.plan_c(start, execute=True) # go to above starting waypoint
        start.position.z = cached_start_z
        await self._motion_planner.plan_c(start, execute=True) # go to starting waypoint
        await self._motion_planner.execute_waypoints(waypoints, velocity=0.02) # traverse all waypoints 
        await self._motion_planner.plan_n("ready", execute=True) # go to ready pose


    async def _plot_callback(self, request, response):
        """Generate a matplotlib plot of the drawing waypoints."""
        self.get_logger().info("plotting waypoints")
        if self._draw_waypoints is not None:
            dx = self.pose_offset.position.x-0.1
            dy = self.pose_offset.position.y
            dz = self.pose_offset.position.z

            quat = self.pose_offset.orientation
            quat = (quat.x, quat.y, quat.z, quat.w)
            rotation = R.from_quat(quat)
            rotated_points = rotation.apply(np.array(self._draw_waypoints))
            translated_waypoints = rotated_points + np.array([dx, dy, 0])
            offset_waypoints = [(x,y,z+dz) for ((_, _, z), (x, y, _)) in zip(self._draw_waypoints, translated_waypoints)] # don't rotate z

            fig, ax = plot_robot_waypoints(offset_waypoints, paper_height_fn=self.paper_height_model.get_paper_height)
            fig.savefig(f"{self.pkg_share}/output.png")
            self.get_logger().info("done plotting waypoints")
        
        return response

    async def _draw_callback(self, request, response):
        """Draw the waypoints from the previously published image."""
        if self._draw_waypoints is None:
            self.get_logger().info("No waypoints to draw")
            return response
        
        dx = self.pose_offset.position.x
        dy = self.pose_offset.position.y
        dz = self.pose_offset.position.z
        
        quat = self.pose_offset.orientation
        quat = (quat.x, quat.y, quat.z, quat.w)
        rotation = R.from_quat(quat)
        rotated_points = rotation.apply(np.array(self._draw_waypoints))
        translated_waypoints = rotated_points + np.array([dx, dy, 0])
        offset_waypoints = [(x,y,z+dz) for ((_, _, z), (x, y, _)) in zip(self._draw_waypoints, translated_waypoints)] # don't rotate z

        self.get_logger().info("going to ready pose")
        self.get_logger().info("at ready pose. drawing image")
        await self._execute_waypoints(offset_waypoints)
        self.get_logger().info("done drawing image")
        return response
    
    def _route_callback(self, request, response):
        """Convert the received line image into waypoints."""
        lines = json.loads(request.data)
        lines = [[(1-x,y) for (x,y) in line] for line in lines]
        pen_down_dists = [stroke_dist(np.array(line)) for line in lines]
        pen_down_dist = sum(pen_down_dists)

        stroke_segments = lines_to_endpoints(lines)

        cost_matrix = get_cost_matrix(stroke_segments)
        tour, _ = tsp_nearest_neighbor(cost_matrix)

        pen_up_dists, robot_xyz_waypoints = tour_to_robot_waypoints(lines,
                                                                    stroke_segments,
                                                                    tour,
                                                                    paper_width=self.paper_size,
                                                                    paper_height=self.paper_size,
                                                                    xoffset=-self.paper_size/2,
                                                                    yoffset=-self.paper_size/2,
                                                                    paper_height_fn=self.paper_height_model.get_paper_height,
                                                                    pen_clearance=0.01)
        self._draw_waypoints = robot_xyz_waypoints

        pen_up_dist = sum(pen_up_dists)
        total_distance = pen_down_dist + pen_up_dist
        
        self.get_logger().info(f"Received {len(lines)} lines")
        self.get_logger().info(f"Total distance: {total_distance}")
        self.get_logger().info(f"Pen down distance: {pen_down_dist} ({100 * pen_down_dist/total_distance:.2f}% of total)")
        self.get_logger().info(f"Pen up distance: {pen_up_dist} ({100 * pen_up_dist/total_distance:.2f}% of total)")
        return response
    
def main():
    """Enter the main loop for the route planner node."""
    rclpy.init()
    node = RoutePlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
