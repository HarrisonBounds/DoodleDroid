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
from builtin_interfaces.msg import Duration

from action_msgs.msg import GoalStatus
import time
import json

from ament_index_python.packages import get_package_share_directory

import numpy as np

from doodle_droid.paper_height_model import PaperHeightModel, FlatPaperHeightModel, PlanePaperHeightModel
from doodle_droid.stroke_router_utils import stroke_dist, lines_to_endpoints, get_cost_matrix, tour_to_robot_waypoints, tsp_nearest_neighbor, plot_robot_waypoints
from doodle_droid.robot_state import RobotState
from doodle_droid.motion_planner import MotionPlanner
from doodle_droid.path_visualizer import PathVisualizer

from scipy.spatial.transform import Rotation as R

class RoutePlannerNode(Node):
    def __init__(self):
        super().__init__('route_planner_node')
        self._verbosity = 0

        self.pkg_name = "doodle_droid"

        self.pkg_share = get_package_share_directory(self.pkg_name)
        
        self._robot_state = RobotState(self)
        self._path_visualizer = PathVisualizer(self)
        self._motion_planner = MotionPlanner(self)

        self.pen_clearance = 0.01 # 1 cm
        self.paper_size = Vector3(x=0.2, y=0.2, z=0.0)
        self.pose_offset = Pose(position=Point(x=0.4, y=0.0, z=0.20))
        self._calibation_pose_sub = self.create_subscription(Pose, "/surface_pose", self._update_offset, 10)
        # self._test_server = self.create_service(Empty, "/test_line", self._test_line)

        # self.paper_height_model = PlanePaperHeightModel(0, 0, 1, -0.156) # default to flat paper
        self.paper_height_model = PlanePaperHeightModel(0, 0, 1, 0) # default to flat paper

        self._max_vel = 0.02
        self._draw_waypoints = None
        self._cached_joint_traj = None
        self._plot_waypoints = None
        self._draw_server = self.create_service(Empty, "/draw", self._draw_callback)
        self._plot_server = self.create_service(Empty, "/plot", self._plot_callback)
        self._outline_paper_server = self.create_service(Empty, "/outline_paper", self._outline_paper)


        self._drawing_time_publisher = self.create_publisher(Duration, "/drawing_time", 10)
        self._brush_strokes_subscription = self.create_subscription(String, "/new_image", self._route_callback,10)

        return # OVERRIDE LACK OF FULL SERVICE SIGNATURES OF THE BELOW!
        raise NotImplementedError("coordinate msg type of a paper height model topic")
        self._paper_height_subscription = self.create_subscription(String, "paper_height", self._paper_height_callback, 10)


    async def _paper_size_callback(self, msg):
        self.paper_size = msg
        await self._update_joint_traj()


    async def _update_offset(self, msg):
        self.get_logger().info(f"Received new surface pose: {msg}")
        self.pose_offset = msg
        await self._update_joint_traj()
    
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
    
    async def _waypoints_to_joint_traj(self, waypoints):
        dx = self.pose_offset.position.x
        dy = self.pose_offset.position.y
        dz = self.pose_offset.position.z  + 0.01 # 1cm above
         
        paper_width = self.paper_size.x
        paper_height = self.paper_size.y

        # scale
        np_waypoints = np.array(self._draw_waypoints) * np.array([paper_width, paper_height, 1])

        # rotate
        quat = self.pose_offset.orientation
        quat = (quat.x, quat.y, quat.z, quat.w)
        rotation = R.from_quat(quat)
        rotated_points = rotation.apply(np_waypoints)

        # translate
        translated_waypoints = rotated_points + np.array([dx, dy, 0])
        offset_waypoints = [(x,y,z+dz) for ((_, _, z), (x, y, _)) in zip(self._draw_waypoints, translated_waypoints)] # don't rotate z
        self._plot_waypoints = offset_waypoints
        
        waypoints = self.pose_waypoints_from_xyz(offset_waypoints)
        self._path_visualizer.set_visualizing_waypoints(waypoints)

        return await self._motion_planner._construct_joint_trajectory_from_waypoints(waypoints, self._max_vel)

    async def _update_joint_traj(self):
        if self._draw_waypoints is None:
            return
        self._cached_joint_traj =  await self._waypoints_to_joint_traj(self._draw_waypoints)
        self.get_logger().info(f"")
        final_time = self._cached_joint_traj.points[-1].time_from_start
        self._drawing_time_publisher.publish(final_time)
        self.get_logger().info(f"Drawing will take: {final_time.sec + final_time.nanosec*1e-9:8.4f} seconds")

    async def _execute_traj(self):
        
        if self._verbosity > 0:
            self.get_logger().info("going to ready pose")

        # start_pose = self._cached_joint_traj.points[0]
        # start_pose = JointState()
        # start_pose.name = self._cached_joint_traj.joint_names
        # start_pose.position = self._cached_joint_traj.points[0].positions
        # await self._motion_planner.plan_j(start_pose, execute=True)
        start_point = self._plot_waypoints[0]
        pose = Pose()
        pose.position.x = start_point[0]
        pose.position.y = start_point[1]
        pose.position.z = start_point[2] 
        pose.orientation = Quaternion(x=0.9238792,
                                        y=-0.3826833,
                                        z=0.0003047,
                                        w=0.0007357)
        await self._motion_planner.plan_c(pose, execute=True)

        if self._verbosity > 0:
            self.get_logger().info("at start pose. Drawing")

        await self._motion_planner.execute_joint_trajectory(self._cached_joint_traj) # traverse all waypoints
        if self._verbosity > 0:
            self.get_logger().info("done drawing")

    async def _plot_callback(self, request, response):
        self.get_logger().info("plotting waypoints")
        if self._plot_waypoints is not None:
            fig, ax = plot_robot_waypoints(self._plot_waypoints, paper_height_fn=self.paper_height_model.get_paper_height)
            fig.savefig(f"{self.pkg_share}/output.png")
            self.get_logger().info("done plotting waypoints")
        
        return response

    async def _draw_callback(self, request, response):
        if self._draw_waypoints is None:
            self.get_logger().info("No waypoints to draw")
            return response
        
        await self._execute_traj()
        
        return response
    
    async def _route_callback(self, request, response):
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
                                                                    paper_width=1.0,
                                                                    paper_height=1.0,
                                                                    xoffset=-.50,
                                                                    yoffset=-.50,
                                                                    paper_height_fn=self.paper_height_model.get_paper_height,
                                                                    pen_clearance=self.pen_clearance,)
        self._draw_waypoints = robot_xyz_waypoints
        await self._update_joint_traj()

        if self._verbosity > 0:
            pen_up_dist = sum(pen_up_dists)
            total_distance = pen_down_dist + pen_up_dist
            
            self.get_logger().info(f"Received {len(lines)} lines")
            self.get_logger().info(f"Total distance: {total_distance}")
            self.get_logger().info(f"Pen down distance: {pen_down_dist} ({100 * pen_down_dist/total_distance:.2f}% of total)")
            self.get_logger().info(f"Pen up distance: {pen_up_dist} ({100 * pen_up_dist/total_distance:.2f}% of total)")
        return response
    
    async def _outline_paper(self, request, response):
        joint_traj = await self._waypoints_to_joint_traj([(0,0,self.pen_clearance),
                                                    (0,1,self.pen_clearance),
                                                    (1,1,self.pen_clearance),
                                                    (1,0,self.pen_clearance),
                                                    (0,0,self.pen_clearance)])
        
        start_pose = self._cached_joint_traj.points[0]
        start_pose = JointState()
        start_pose.name = self._cached_joint_traj.joint_names
        start_pose.position = self._cached_joint_traj.points[0]
        await self._motion_planner.plan_j(start_pose, execute=True)

        await self._motion_planner.execute_joint_trajectory(joint_traj)
        
    async def _test_line(self, request, response):
        raise NotImplementedError
        # await self._motion_planner.plan_n('ready', execute=True)
        pose = Pose()
        pose.orientation = Quaternion(x=0.9238792,
                                        y=-0.3826833,
                                        z=0.0003047,
                                        w=0.0007357)
        
        await self._motion_planner.plan_c(pose, execute=True)
        # await self._execute_waypoints(pts)

        return response

def main():
    rclpy.init()
    node = RoutePlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
