from functools import partial
import rclpy
from rclpy.node import Node
# from moveit2_api.robot_state import RobotState
# from moveit2_api.motion_planner import MotionPlanner
from geometry_msgs.msg import PoseStamped, PoseArray
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Quaternion, Vector3, Pose
from std_srvs.srv import Empty
from std_msgs.msg import String
from action_msgs.msg import GoalStatus
import time
import json

from ament_index_python.packages import get_package_share_directory

import numpy as np

from doodle_droid.paper_height_model import PaperHeightModel, FlatPaperHeightModel, PlanePaperHeightModel
from doodle_droid.stroke_router_utils import stroke_dist, lines_to_endpoints, get_cost_matrix, tour_to_robot_waypoints, tsp_nearest_neighbor, plot_robot_waypoints
from doodle_droid.robot_state import RobotState
from doodle_droid.motion_planner import MotionPlanner



class RoutePlannerNode(Node):
    def __init__(self):
        super().__init__('route_planner_node')
        self.pkg_name = "doodle_droid"

        self.pkg_share = get_package_share_directory(self.pkg_name)
        
        self._robot_state = RobotState(self)
        self._motion_planner = MotionPlanner(self)

        self._test_server = self.create_service(Empty, "_test_line", self._test_line)

        self.paper_height_model = PlanePaperHeightModel(0, 0, 1, -0.188) # default to flat paper
        # self.paper_height_model = PlanePaperHeightModel(0, 0, 1, 0) # default to flat paper

        self._draw_waypoints = None
        self._draw_server = self.create_service(Empty, "/draw", self._draw_callback)
        self._plot_server = self.create_service(Empty, "/plot", self._plot_callback)

        self._brush_strokes_subscription = self.create_subscription(String, "/new_image", self._route_callback,10)

        return # OVERRIDE LACK OF FULL SERVICE SIGNATURES OF THE BELOW!
        raise NotImplementedError("coordinate msg type of a paper height model topic")
        self._paper_height_subscription = self.create_subscription(String, "paper_height", self._paper_height_callback, 10)

        
    
    def _paper_height_callback(self, msg):
        self.get_logger().info(f"Received paper height model: {msg.data}")
        self.paper_height_model.update(*msg.data)
    
    def pose_waypoints_from_xyz(self, pts):
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
        waypoints = self.pose_waypoints_from_xyz(pts)
        start = waypoints.poses[0]
        await self._motion_planner.plan_c(start, execute=True) # go to starting waypoint
        await self._motion_planner.execute_waypoints(waypoints, 1.0) # traverse all waypoints

    async def _plot_callback(self, request, response):
        self.get_logger().info("plotting waypoints")
        if self._draw_waypoints is not None:
            fig, ax = plot_robot_waypoints(self._draw_waypoints)
            fig.savefig(f"{self.pkg_share}/output.png")
            self.get_logger().info("done plotting waypoints")
        
        return response

    async def _draw_callback(self, request, response):
        if self._draw_waypoints is None:
            self.get_logger().info("No waypoints to draw")
            return response

        # N = 10
        # paper_height = 0.174
        # paper_size = 0.05
        # paper_origin = [0.4, 0.0, paper_height]
        # t = np.linspace(0, 2*np.pi, N)
        # x = paper_size * np.cos(t) + paper_origin[0]
        # y = paper_size * np.sin(t) + paper_origin[1]
        # z = np.full_like(x, paper_height)
        # waypoints = list(zip(x, y, z))
        self.get_logger().info("going to ready pose")

        await self._motion_planner.plan_n("ready", execute=True)
        self.get_logger().info("at ready pose. drawing image")
        await self._execute_waypoints(self._draw_waypoints)
        self.get_logger().info("done drawing image")
        return response
    
    def _route_callback(self, request, response):
        lines = json.loads(request.data)
        pen_down_dists = [stroke_dist(np.array(line)) for line in lines]
        pen_down_dist = sum(pen_down_dists)

        stroke_segments = lines_to_endpoints(lines)

        cost_matrix = get_cost_matrix(stroke_segments)
        tour, _ = tsp_nearest_neighbor(cost_matrix)

        pen_up_dists, robot_xyz_waypoints = tour_to_robot_waypoints(lines,
                                                                    stroke_segments,
                                                                    tour,
                                                                    paper_height_fn=self.paper_height_model.get_paper_height,
                                                                    pen_clearance=0.10)
        self._draw_waypoints = robot_xyz_waypoints

        pen_up_dist = sum(pen_up_dists)
        total_distance = pen_down_dist + pen_up_dist
        
        self.get_logger().info(f"Received {len(lines)} lines")
        self.get_logger().info(f"Total distance: {total_distance}")
        self.get_logger().info(f"Pen down distance: {pen_down_dist} ({100 * pen_down_dist/total_distance:.2f}% of total)")
        self.get_logger().info(f"Pen up distance: {pen_up_dist} ({100 * pen_up_dist/total_distance:.2f}% of total)")
        self.get_logger().info(f"draw waypoints: { self._draw_waypoints}") 
        return response
    
    async def _test_line(self, request, response):
        x = 0.25
        y = 0.0
        dy = 0.05
        z = 0.188
        pts = [[x,y+dy,z], [x, y-dy, z]]
        await self._execute_waypoints(pts)

        return response

def main():
    rclpy.init()
    node = RoutePlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
