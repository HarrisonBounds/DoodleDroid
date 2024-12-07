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

import numpy as np

from doodle_droid.paper_height_model import PaperHeightModel, FlatPaperHeightModel, PlanePaperHeightModel
from doodle_droid.stroke_router_utils import stroke_dist, lines_to_endpoints, get_cost_matrix, tour_to_robot_waypoints, tsp_nearest_neighbor
from doodle_droid.robot_state import RobotState
from doodle_droid.motion_planner import MotionPlanner



class RoutePlannerNode(Node):
    def __init__(self):
        super().__init__('route_planner_node')
        self._robot_state = RobotState(self)
        self._motion_planner = MotionPlanner(self)

        self._test_server = self.create_service(Empty, "_test_line", self._test_line)

        self.paper_height_model = None

        self._draw_waypoints = None
        self._draw_server = self.create_service(Empty, "_draw", self._draw_callback)

        self._brush_strokes_subscription = self.create_subscription(String, "/new_image", self._route_callback,10)

        # PlanePaperHeightModel(0, 0, 1, -0.188) # default to flat paper
        self.surface_pose = None
        self.drawing_dims = None

        self._surface_pose_subscription = self.create_subscription(Pose, 'surface_pose', self._surface_pose_callback, 10)
        self._drawing_dims_subscription = self.create_subscription(Vector3, 'drawing_dims', self._drawing_dims_callback, 10)
        

    def _surface_pose_callback(self, msg):
        self.get_logger().info(f"Received surface pose: {msg}")
        self.surface_pose = msg
        self._paper_height_callback()

    def _drawing_dims_callback(self, msg):
        self.get_logger().info(f"Received drawing dimensions: {msg}")
        self.drawing_dims = msg
        self._paper_height_callback()
    
    def _paper_height_callback(self):
        if self.surface_pose is not None and self.drawing_dims is not None:
            self.paper_height_model = PlanePaperHeightModel(self.surface_pose.position.x,
                                                            self.surface_pose.position.y,
                                                            self.surface_pose.position.z,
                                                            -self.drawing_dims.z)
            self.get_logger().info(f"Paper height model: {self.paper_height_model}")
        else:
            self.get_logger().info("Waiting for surface pose and drawing dimensions")
    
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

    async def _draw_callback(self, request, response):
        if self._draw_waypoints is None:
            self.get_logger().info("No waypoints to draw")
            return response
        
        await self._execute_waypoints(self._draw_waypoints)
        return response
    
    def _route_callback(self, request, response):
        lines = json.loads(request.data)
        pen_down_dists = [stroke_dist(np.array(line)) for line in lines]
        pen_down_dist = sum(pen_down_dists)

        stroke_segments = lines_to_endpoints(lines)

        cost_matrix = get_cost_matrix(stroke_segments)
        tour, _ = tsp_nearest_neighbor(cost_matrix)

        pen_up_dists, robot_xyz_waypoints = tour_to_robot_waypoints(lines, tour, paper_height_fn=self.paper_height_model.get_paper_height, pen_clearance=1.0)
        self._draw_waypoints = robot_xyz_waypoints

        pen_up_dist = sum(pen_up_dists)
        total_distance = pen_down_dist + pen_up_dist
        
        self.get_logger().info(f"Total distance: {total_distance}")
        self.get_logger().info(f"Pen down distance: {pen_down_dist} ({100 * pen_down_dist/total_distance:.2f}% of total)")
        self.get_logger().info(f"Pen up distance: {pen_up_dist} ({100 * pen_up_dist/total_distance:.2f}% of total)")
    
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
