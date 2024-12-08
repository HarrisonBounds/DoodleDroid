#             DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
#                     Version 2, December 2004

#  Copyright (C) 2004 Sam Hocevar <sam@hocevar.net>

#  Everyone is permitted to copy and distribute verbatim or modified
#  copies of this license document, and changing it is allowed as long
#  as the name is changed.

#             DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
#    TERMS AND CONDITIONS FOR COPYING, DISTRIBUTION AND MODIFICATION

#   0. You just DO WHAT THE FUCK YOU WANT TO.

"""
Motion Planner Class.

The Motion Planner Class communicates through several ROS 2 protocols:

ACTIONS:
+ /move_action (moveit_msgs.action.MoveGroup) - To plan or execute to a goal
state.
+ /fer_arm_controller/follow_joint_trajectory
(control_msgs.action.FollowJointTrajectory) - To execute a joint trajectory.

SERVICES:
+ /compute_cartesian_path (moveit_msgs.srv.GetCartesianPath) - To compute
the cartesian path.
"""

import math
from os import path
import xml.etree.ElementTree as ET

from action_msgs.msg import GoalStatus

from ament_index_python.packages import get_package_share_directory

from builtin_interfaces.msg import Duration

from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance

from geometry_msgs.msg import Point, Pose, PoseArray, \
    PoseStamped, Quaternion, Vector3

from doodle_droid.robot_state import RobotState\
    as RobotStateWrapper

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, GenericTrajectory, \
    JointConstraint, MotionPlanRequest, MoveItErrorCodes, \
    OrientationConstraint, PlanningOptions, \
    PositionConstraint, RobotState, WorkspaceParameters
from moveit_msgs.srv import GetCartesianPath

from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node

from sensor_msgs.msg import JointState

from shape_msgs.msg import SolidPrimitive

from std_msgs.msg import Header

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import xacro


class MotionPlanner():
    """
    Motion Planner Class.

    The Motion Planner Class communicates through several ROS 2 protocols:

    ACTIONS:
    + /move_action (moveit_msgs.action.MoveGroup) - To plan or execute to a
    goal state.
    + /fer_arm_controller/follow_joint_trajectory
    (control_msgs.action.FollowJointTrajectory) - To execute a joint
    trajectory.

    SERVICES:
    + /compute_cartesian_path (moveit_msgs.srv.GetCartesianPath) - To compute
    the cartesian path.
    """

    def __init__(self,
                 node: Node):
        """
        Initialize the MotionPlanner.

        Sets up ROS 2 services along with related assets.

        :param node: The ROS 2 node
        :type node: rclpy.node
        """
        # Inherit Node
        self._node = node

        # Setup parameters
        self._arm_group_name = 'fer_manipulator'
        self._joint_names = ['fer_joint1', 'fer_joint2', 'fer_joint3',
                             'fer_joint4', 'fer_joint5', 'fer_joint6',
                             'fer_joint7']

        self._hand_group_name = 'hand'
        self._hand_joint_names = ['fer_finger_joint1', 'fer_finger_joint2']

        self._joint_tolerance = 0.0005
        self._position_tolerance = 0.01
        self._orientation_tolerance = 0.01
        self._base_frame = 'base'
        self._ee_frame = 'fer_link8'

        # Load SRDF and read predefined states
        self._group_states = {}
        xacro_file_path = path.join(get_package_share_directory(
            'franka_fer_moveit_config'), 'srdf', 'fer_arm.srdf.xacro')
        urdf = xacro.process_file(xacro_file_path).toxml()
        root = ET.fromstring(urdf)
        for group_state in root.findall('.//group_state'):
            state_name = group_state.get('name')
            group_name = group_state.get('group')
            joint_values = {}

            for joint in group_state.findall('joint'):
                joint_name = joint.get('name')  # Joint name
                joint_value = float(joint.get('value'))  # Joint value
                joint_values[joint_name] = joint_value

            if (group_name == self._arm_group_name):
                self._group_states[state_name] = {'joints': joint_values}

        # Setup callback group
        self._callback_group = MutuallyExclusiveCallbackGroup()

        # Setup services
        self._compute_cartesian_path_client = self._node.create_client(
            GetCartesianPath, 'compute_cartesian_path',
            callback_group=self._callback_group)

        # Create MoveGroup.action client
        self._move_group_action_client = ActionClient(
            self._node,
            MoveGroup,
            'move_action',
            callback_group=self._callback_group
        )
        self._follow_joint_trajectory_action_client = ActionClient(
            self._node,
            FollowJointTrajectory,
            'fer_arm_controller/follow_joint_trajectory',
            callback_group=self._callback_group
        )

        # Get the robot state
        self._robot_state = RobotStateWrapper(self._node)

    def print_status(self,
                     status: int):
        """
        Print the status of the planning.

        :param status: The status of the planning
        :type status: int
        """
        if (status == GoalStatus.STATUS_UNKNOWN):
            self._node.get_logger().info('Planning status: STATUS_UNKNOWN')
        elif (status == GoalStatus.STATUS_ACCEPTED):
            self._node.get_logger().info('Planning status: STATUS_ACCEPTED')
        elif (status == GoalStatus.STATUS_EXECUTING):
            self._node.get_logger().info('Planning status: STATUS_EXECUTING')
        elif (status == GoalStatus.STATUS_CANCELING):
            self._node.get_logger().info('Planning status: STATUS_CANCELING')
        elif (status == GoalStatus.STATUS_SUCCEEDED):
            self._node.get_logger().info('Planning status: STATUS_SUCCEEDED')
        elif (status == GoalStatus.STATUS_CANCELED):
            self._node.get_logger().info('Planning status: STATUS_CANCELED')
        elif (status == GoalStatus.STATUS_ABORTED):
            self._node.get_logger().info('Planning status: STATUS_ABORTED')
        else:
            self._node.get_logger().info(
                f'Unknown Status: {status}')

    async def plan_gripper(self,
                           goal_joint_states: JointState,
                           start_hand_state=None,
                           execute=False):
        """
        Plan the gripper.

        :param goal_joint_states: The goal joint states
        :type goal_joint_states: sensor_msgs.msg.JointState
        :param start_hand_state: The start hand state
        :type start_hand_state: moveit_msgs.msg.RobotState
        :param execute: Execute the plan
        :type execute: bool

        :return: The result and status
        :rtype: tuple
        """
        # Check if the Move Group server is available
        goal = MoveGroup.Goal()
        goal.planning_options = self._construct_planning_options(
            execute=execute)

        # Construct the start state
        if (start_hand_state is None):
            hand_start_state = self._construct_robot_state_by_joint_states(
                self._robot_state.get_hand_joint_states())

        # 0.0 - 0.35 is joint range so default tolerance for rest of joints
        # does not good sense.
        goal_constraints = self._construct_goal_constraints_by_joint_states(
            goal_joint_states, joint_names=self._hand_joint_names,
            joint_tolerance=0.001)
        goal.request = self._construct_move_group_request_by_constraints(
            hand_start_state, goal_constraints,
            group_name=self._hand_group_name)
        goal_handle = await self._move_group_action_client.send_goal_async(
            goal)
        result = await goal_handle.get_result_async()
        return result.result, result.status

    async def plan_n(self,
                     goal_state_name: str,
                     start_robot_state=None,
                     execute=False):
        """
        Plan based on the predefined state from robot SRDF.

        :param goal_state_name: The goal state name
        :type goal_state_name: str
        :param start_robot_state: The start robot state
        :type start_robot_state: moveit_msgs.msg.RobotState
        :param execute: Execute the plan
        :type execute: bool

        :return: The result and status
        :rtype: tuple
        """
        # Check if the Move Group server is available
        if not self._move_group_action_client.wait_for_server(timeout_sec=5.0):
            self._node.get_logger().error(
                'Move Group server is not available.')
            return

        # Check if the state is in the predefined states
        found_state = False
        for state in self._group_states.keys():
            if state == goal_state_name:
                found_state = True
                break
        if not found_state:
            self._node.get_logger().error(
                'State not found in predefined states.')
            return

        # Construct the goal joint states
        goal_joint_states = JointState()
        goal_joint_states.name = self._joint_names
        goal_joint_states.position = []
        for joint_name in self._joint_names:
            goal_joint_states.position.append(
                self._group_states[goal_state_name]['joints'][joint_name])
        return await self.plan_j(goal_joint_states, start_robot_state, execute)

    async def plan_j(self,
                     goal_joint_states: JointState,
                     start_robot_state=None,
                     execute=False):
        """
        Plan based on joint configuration.

        :param goal_joint_states: The goal joint states
        :type goal_joint_states: sensor_msgs.msg.JointState
        :param start_robot_state: The start robot state
        :type start_robot_state: moveit_msgs.msg.RobotState
        :param execute: Execute the plan
        :type execute: bool

        :return: The result and status
        :rtype: tuple
        """
        # Check if the Move Group server is available
        if not self._move_group_action_client.wait_for_server(timeout_sec=5.0):
            self._node.get_logger().error(
                'Move Group server is not available.')
            return

        # Construct the goal
        goal = MoveGroup.Goal()
        goal.planning_options = self._construct_planning_options(
            execute=execute)
        if (start_robot_state is None):
            start_robot_state =\
                self._construct_robot_state_by_joint_states(
                    self._robot_state.get_arm_joint_states())
        goal_constraints =\
            self._construct_goal_constraints_by_joint_states(
                goal_joint_states, joint_names=self._joint_names)
        goal.request = self._construct_move_group_request_by_constraints(
            start_robot_state, goal_constraints,
            group_name=self._arm_group_name)
        goal_handle = await self._move_group_action_client.send_goal_async(
            goal)
        result = await goal_handle.get_result_async()
        return result.result, result.status

    async def plan_p(self,
                     goal_position: Point = None,
                     goal_orientation: Quaternion = None,
                     start_robot_state=None,
                     execute=False):
        """
        Plan based on pose.

        :param goal_position: The goal position
        :type goal_position: geometry_msgs.msg.Point
        :param goal_orientation: The goal orientation
        :type goal_orientation: geometry_msgs.msg.Quaternion
        :param start_robot_state: The start robot state
        :type start_robot_state: moveit_msgs.msg.RobotState

        :return: The result and status
        :rtype: tuple
        """
        # Check if the Move Group server is available
        if not self._move_group_action_client.wait_for_server(timeout_sec=5.0):
            self._node.get_logger().error(
                'Move Group server is not available.')
            return

        # Construct the goal
        goal = MoveGroup.Goal()
        goal.planning_options = self._construct_planning_options(
            execute=execute)
        if (start_robot_state is None):
            start_robot_state =\
                self._construct_robot_state_by_joint_states(
                    self._robot_state.get_arm_joint_states())
        goal_constraints = self._construct_goal_constraints_by_pose(
            goal_position, goal_orientation)
        goal.request = self._construct_move_group_request_by_constraints(
            start_robot_state, goal_constraints,
            group_name=self._arm_group_name)
        goal_handle = await self._move_group_action_client.send_goal_async(
            goal)
        result = await goal_handle.get_result_async()
        return result.result, result.status

    async def plan_c(self,
                     goal_pose: Pose,
                     start_robot_state: RobotState = None,
                     execute: bool = False):
        """
        Plan based on cartesian path.

        :param goal_pose: The goal pose
        :type goal_pose: geometry_msgs.msg.Pose
        :param start_robot_state: The start robot state
        :type start_robot_state: moveit_msgs.msg.RobotState
        :param execute: Execute the plan
        :type execute: bool

        :return: The result and status
        :rtype: tuple
        """
        # Check if the Move Group server is available
        if not self._compute_cartesian_path_client.wait_for_service(
                timeout_sec=5.0):
            self._node.get_logger().error(
                'Compute Cartesian Path server is not available.')
            return

        # Construct the cartesian path request
        cartesian_request = await self._construct_cartesian_path_request(
            goal_pose)
        cartesian_res = await self._compute_cartesian_path_client.call_async(
            cartesian_request)
        self._print_cartesian_request_error_code(cartesian_res.error_code.val)
        joint_traj = cartesian_res.solution.joint_trajectory

        # Construct the goal
        if (start_robot_state is None):
            start_robot_state =\
                self._construct_robot_state_by_joint_states(
                    self._robot_state.get_arm_joint_states())
        move_group_req =\
            self._construct_move_group_request_by_reference_joint_traj(
                start_robot_state,
                joint_traj)
        goal = MoveGroup.Goal()
        goal.planning_options = self._construct_planning_options(
            execute=execute)
        goal.request = move_group_req
        goal_handle = await self._move_group_action_client.send_goal_async(
            goal)
        result = await goal_handle.get_result_async()

        # For unknown reasons, the joint states are not updated during
        # the execution, so we need to update it manually.
        joint_states = JointState()
        joint_states.name = joint_traj.joint_names
        joint_states.position = joint_traj.points[-1].positions
        self._robot_state.update_arm_joint_states(joint_states)
        return result.result, result.status

    async def execute_waypoints(self,
                                waypoints: PoseArray,
                                velocity: float = 0.02,
                                start_robot_state: RobotState = None):
        """
        Execute the waypoints.

        :param waypoints: The waypoints
        :type waypoints: geometry_msgs.msg.PoseArray
        :param velocity: The velocity
        :type velocity: float
        :param start_robot_state: The start robot state
        :type start_robot_state: moveit_msgs.msg.RobotState

        :return: The result and status
        :rtype: tuple
        """
        # Go to the first waypoint
        await self.plan_c(waypoints.poses[0],
                          start_robot_state,
                          execute=True)

        # Construct the joint trajectory
        joint_traj =\
            await self._construct_joint_trajectory_from_waypoints(waypoints,
                                                                  velocity)
        # Construct the goal
        if (start_robot_state is None):
            start_robot_state =\
                self._construct_robot_state_by_joint_states(
                    self._robot_state.get_arm_joint_states())
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = joint_traj
        # goal.path_tolerance = self._construct_tolerance_list(
        #     position_tolerance=0.005
        # )
        # goal.goal_tolerance = self._construct_tolerance_list(
        #     position_tolerance=0.005)
        goal.goal_time_tolerance = Duration(sec=int(10), nanosec=int(0))
        goal_handle =\
            await self._follow_joint_trajectory_action_client.send_goal_async(
                goal)
        result = await goal_handle.get_result_async()

        # For unknown reasons, the joint states are not updated during
        # the execution, so we need to update it manually.
        joint_states = JointState()
        joint_states.name = joint_traj.joint_names
        joint_states.position = joint_traj.points[-1].positions
        self._robot_state.update_arm_joint_states(joint_states)
        return result.result, result.status

    def _construct_tolerance_list(self,
                                  position_tolerance: float = 0.01,
                                  velocity_tolerance: float = 0.01,
                                  acceleration_tolerance: float = 0.01):
        """
        Construct the path tolerance.

        :param path_tolerance: Position tolerance of the joint.
        :type path_tolerance: float
        :param velocity_tolerance: Velocity tolerance of the joint.
        :type velocity_tolerance: float
        :param acceleration_tolerance: Acceleration tolerance of the joint.
        :type acceleration_tolerance: float

        :return: The path tolerance
        :rtype: trajectory_msgs.msg.JointTolerance
        """
        tolerance_list = []
        for joint_name in self._joint_names:
            tolerance = JointTolerance()
            tolerance.name = joint_name
            tolerance.position = position_tolerance
            tolerance.velocity = velocity_tolerance
            tolerance.acceleration = acceleration_tolerance
            tolerance_list.append(tolerance)
        return tolerance_list

    async def _construct_joint_trajectory_from_waypoints(
            self,
            waypoints: PoseArray,
            velocity: float = 0.02):
        """
        Construct the cartesian path from waypoints.

        :param waypoints: The waypoints
        :type waypoints: geometry_msgs.msg.PoseArray
        :param velocity: The velocity
        :type velocity: float

        :return: The joint trajectory
        :rtype: trajectory_msgs.msg.JointTrajectory
        """
        # Construct the joint trajectory
        joint_traj = JointTrajectory()
        joint_traj.joint_names = self._joint_names
        joint_states = self._robot_state.get_arm_joint_states()
        t = 0.0
        pose_cache = None
        for waypoint in waypoints.poses:
            # Add current joint states to the trajectory
            traj_point = JointTrajectoryPoint()
            traj_point.positions = joint_states.position
            joint_traj.points.append(traj_point)

            # Solve the next waypoint IK
            waypoint_stampted = PoseStamped()
            waypoint_stampted.pose = waypoint
            waypoint_stampted.header.frame_id = waypoints.header.frame_id
            waypoint_stampted.header.stamp = waypoints.header.stamp
            ik_res = await self._robot_state.perform_IK(
                pose=waypoint_stampted,
                robot_state=self._construct_robot_state_by_joint_states(
                    joint_states)
                )
            if (ik_res.error_code.val != MoveItErrorCodes.SUCCESS):
                self._node.get_logger().info(
                    f'IK failed. Error code: {ik_res.error_code.val}, '
                    + 'pose: {str(waypoint)}')
                return None
            joint_states = ik_res.solution.joint_state
            joint_states = self._filter_out_arm_joint_states(joint_states)
            if pose_cache is None:
                t = 0.0
            else:
                distance = math.sqrt(
                    (waypoint.position.x - pose_cache.position.x) ** 2 +
                    (waypoint.position.y - pose_cache.position.y) ** 2 +
                    (waypoint.position.z - pose_cache.position.z) ** 2)
                t += distance/velocity
            sec = int(math.floor(t))
            nanosec = int((t - sec) * 1e9)
            traj_point.time_from_start = Duration(sec=sec, nanosec=nanosec)
            pose_cache = waypoint
        joint_traj.header.frame_id = waypoints.header.frame_id
        joint_traj.header.stamp = waypoints.header.stamp
        return joint_traj

    def _filter_out_arm_joint_states(self,
                                     joint_states: JointState):
        """
        Filter out the arm joint states.

        :param joint_states: The joint states
        :type joint_states: sensor_msgs.msg.JointState

        :return: The arm joint states
        :rtype: sensor_msgs.msg.JointState
        """
        arm_joint_states = JointState()
        arm_joint_states.name = self._joint_names
        arm_joint_states.position = []
        for idx, joint_name in enumerate(joint_states.name):
            if joint_name in self._joint_names:
                arm_joint_states.position.append(joint_states.position[idx])
        return arm_joint_states

    async def _construct_cartesian_path_request(
            self,
            goal_pose: Pose,
            max_step: float = 0.01,
            avoid_collisions: bool = True,
            max_acceleration_scaling_factor: float = 0.1,
            max_velocity_scaling_factor: float = 0.1):
        """
        Construct the cartesian path request.

        :param goal_pose: The goal pose
        :type goal_pose: geometry_msgs.msg.Pose
        :param max_step: The maximum step
        :type max_step: float
        :param avoid_collisions: Avoid collisions
        :type avoid_collisions: bool
        :param max_acceleration_scaling_factor: The maximum acceleration
            scaling factor
        :type max_acceleration_scaling_factor: float
        :param max_velocity_scaling_factor: The maximum velocity scaling factor
        :type max_velocity_scaling_factor: float

        :return: The cartesian path request
        :rtype: moveit_msgs.srv.GetCartesianPath.Request
        """
        # Construct the waypoints
        waypoints = []
        # Get the current pose by performing FK
        current_pose = await self._robot_state.perform_FK(
            self._construct_robot_state_by_joint_states(
                self._robot_state.get_arm_joint_states()))
        # Append the current pose and the goal pose
        waypoints.append(current_pose.pose_stamped[0].pose)
        waypoints.append(goal_pose)

        # Construct the request
        request = GetCartesianPath.Request()
        request.header.stamp = self._node.get_clock().now().to_msg()
        request.header.frame_id = self._base_frame
        request.group_name = self._arm_group_name
        request.link_name = self._ee_frame
        request.waypoints = waypoints
        request.max_step = max_step
        request.avoid_collisions = avoid_collisions
        request.max_acceleration_scaling_factor =\
            max_acceleration_scaling_factor
        request.max_velocity_scaling_factor =\
            max_velocity_scaling_factor
        return request

    def _construct_goal_constraints_by_pose(self,
                                            position: Point = None,
                                            orientation: Quaternion = None):
        """
        Construct the goal constraints by pose.

        :param position: The position
        :type position: geometry_msgs.msg.Point
        :param orientation: The orientation
        :type orientation: geometry_msgs.msg.Quaternion

        :return: The goal constraints
        :rtype: list
        """
        # Construct the constraints
        constraints = Constraints()
        if (position is not None):
            position_constraint = PositionConstraint()
            position_constraint.header = Header()
            position_constraint.header.frame_id = self._base_frame
            position_constraint.link_name = self._ee_frame
            position_constraint.target_point_offset = Vector3()
            position_constraint.weight = 1.0
            position_constraint.constraint_region.primitives = [
                SolidPrimitive(type=SolidPrimitive.SPHERE,
                               dimensions=[self._position_tolerance])]
            pose = Pose()
            pose.position = position
            position_constraint.constraint_region.primitive_poses = [pose]
            constraints.position_constraints = [position_constraint]
        if (orientation is not None):
            orientation_constraint = OrientationConstraint()
            orientation_constraint.header = Header()
            orientation_constraint.header.frame_id = self._base_frame
            orientation_constraint.link_name = self._ee_frame
            orientation_constraint.orientation = orientation
            orientation_constraint.absolute_x_axis_tolerance =\
                self._orientation_tolerance
            orientation_constraint.absolute_y_axis_tolerance =\
                self._orientation_tolerance
            orientation_constraint.absolute_z_axis_tolerance =\
                self._orientation_tolerance
            orientation_constraint.weight = 1.0
            constraints.orientation_constraints = [orientation_constraint]

        return [constraints]

    def _construct_goal_constraints_by_joint_states(
            self,
            joint_states: JointState,
            joint_names: list[str],
            joint_tolerance: float = None):
        """
        Construct the goal constraints by joint states.

        :param joint_states: The joint states
        :type joint_states: sensor_msgs.msg.JointState
        :param joint_names: The joint names
        :type joint_names: list[str]
        :param joint_tolerance: The joint tolerance
        :type joint_tolerance: float

        :return: The goal constraints
        :rtype: list
        """
        # allow overriding of base joint tolerance
        if joint_tolerance is None:
            joint_tolerance = self._joint_tolerance

        # Construct the constraints
        constraints = Constraints()
        joint_constraints = [
            JointConstraint() for _ in range(len(joint_names))]
        for idx, j_constraint in enumerate(joint_constraints):
            j_constraint.joint_name = joint_names[idx]
            joint_name_matched = False
            for idx_js, position_js in enumerate(joint_states.position):
                if j_constraint.joint_name == joint_states.name[idx_js]:
                    joint_name_matched = True
                    j_constraint.position = position_js
                    j_constraint.tolerance_above = joint_tolerance/2
                    j_constraint.tolerance_below = joint_tolerance/2
                    j_constraint.weight = 1.0
                    break
            if not joint_name_matched:
                self._node.get_logger().error(
                    f'Joint name mismatch: {j_constraint.joint_name}\
                            != {joint_states.name[idx_js]}')
                return
        constraints.joint_constraints = joint_constraints
        return [constraints]

    def _construct_robot_state_by_joint_states(self,
                                               joint_states: JointState):
        """
        Construct the robot state by joint states.

        :param joint_states: The joint states
        :type joint_states: sensor_msgs.msg.JointState

        :return: The robot state
        :rtype: moveit_msgs.msg.RobotState
        """
        robot_state = RobotState()
        robot_state.joint_state = joint_states
        return robot_state

    def _construct_workspace_parameter(self,
                                       lb: list[float] = [-2.0, -2.0, -2.0],
                                       ub: list[float] = [2.0, 2.0, 2.0]):
        """
        Construct the workspace parameter.

        :param lb: The lower bound
        :type lb: list[float]
        :param ub: The upper bound
        :type ub: list[float]

        :return: The workspace parameter
        """
        wp = WorkspaceParameters()
        wp.header.frame_id = self._base_frame
        wp.header.stamp = self._node.get_clock().now().to_msg()
        wp.min_corner.x = lb[0]
        wp.min_corner.y = lb[1]
        wp.min_corner.z = lb[2]
        wp.max_corner.x = ub[0]
        wp.max_corner.y = ub[1]
        wp.max_corner.z = ub[2]
        return wp

    def _construct_planning_options(self,
                                    execute: bool = False):
        """
        Construct the planning options.

        :param execute: Execute the plan
        :type execute: bool

        :return: The planning options
        :rtype: moveit_msgs.msg.PlanningOptions
        """
        options = PlanningOptions()
        options.plan_only = (not execute)
        return options

    def _construct_move_group_request_by_reference_joint_traj(
            self,
            start_robot_state: RobotState,
            reference_joint_traj: JointTrajectory,
            num_planning_attempts: int = 100,
            max_velocity_scaling_factor: float = 0.1,
            max_acceleration_scaling_factor: float = 0.1,
            allowed_planning_time: float = 100.0,
            planner_id: str = 'move_group'):
        """
        Construct the Move Group request by reference joint trajectory.

        :param start_robot_state: The start robot state
        :type start_robot_state: moveit_msgs.msg.RobotState
        :param reference_joint_traj: The reference joint trajectory
        :type reference_joint_traj: trajectory_msgs.msg.JointTrajectory
        :param num_planning_attempts: The number of planning attempts
        :type num_planning_attempts: int
        :param max_velocity_scaling_factor: The maximum velocity scaling factor
        :type max_velocity_scaling_factor: float
        :param max_acceleration_scaling_factor: The maximum acceleration
            scaling factor
        :type max_acceleration_scaling_factor: float
        :param allowed_planning_time: The allowed planning time
        :type allowed_planning_time: float
        :param planner_id: The planner ID
        :type planner_id: str

        :return: The Move Group request
        :rtype: moveit_msgs.msg.MotionPlanRequest
        """
        # Construct the request
        request = MotionPlanRequest()
        request.group_name = self._arm_group_name
        request.num_planning_attempts = num_planning_attempts
        request.max_velocity_scaling_factor = max_velocity_scaling_factor
        request.max_acceleration_scaling_factor =\
            max_acceleration_scaling_factor
        request.allowed_planning_time = allowed_planning_time
        request.num_planning_attempts = num_planning_attempts
        request.planner_id = planner_id
        request.workspace_parameters = self._construct_workspace_parameter()
        request.start_state = start_robot_state

        # Reference trajectory
        ref_traj = GenericTrajectory()
        ref_traj.header.stamp = self._node.get_clock().now().to_msg()
        ref_traj.header.frame_id = self._base_frame
        ref_traj.joint_trajectory = [reference_joint_traj]
        request.reference_trajectories = [ref_traj]

        # Goal Constraints
        traj_joint_names = reference_joint_traj.joint_names
        traj_joint_goals = reference_joint_traj.points[-1].positions
        goal_joint_states = JointState()
        goal_joint_states.name = traj_joint_names
        goal_joint_states.position = traj_joint_goals
        request.goal_constraints =\
            self._construct_goal_constraints_by_joint_states(
                goal_joint_states, joint_names=self._joint_names)

        return request

    def _construct_move_group_request_by_constraints(
            self,
            start_robot_state: RobotState,
            goal_constraints: Constraints,
            num_planning_attempts: int = 10,
            max_velocity_scaling_factor: float = 0.1,
            max_acceleration_scaling_factor: float = 0.1,
            allowed_planning_time: float = 10.0,
            planner_id: str = 'move_group',
            group_name: str = None):
        """
        Construct the Move Group request by constraints.

        :param start_robot_state: The start robot state
        :type start_robot_state: moveit_msgs.msg.RobotState
        :param goal_constraints: The goal constraints
        :type goal_constraints: list
        :param num_planning_attempts: The number of planning attempts
        :type num_planning_attempts: int
        :param max_velocity_scaling_factor: The maximum velocity scaling factor
        :type max_velocity_scaling_factor: float
        :param max_acceleration_scaling_factor: The maximum acceleration
            scaling factor
        :type max_acceleration_scaling_factor: float
        :param allowed_planning_time: The allowed planning time
        :type allowed_planning_time: float
        :param planner_id: The planner ID
        :type planner_id: str
        :param group_name: The group name
        :type group_name: str

        :return: The Move Group request
        :rtype: moveit_msgs.msg.MotionPlanRequest
        """
        request = MotionPlanRequest()
        request.group_name = group_name
        request.num_planning_attempts = num_planning_attempts
        request.max_velocity_scaling_factor = max_velocity_scaling_factor
        request.max_acceleration_scaling_factor =\
            max_acceleration_scaling_factor
        request.allowed_planning_time = allowed_planning_time
        request.num_planning_attempts = num_planning_attempts
        request.planner_id = planner_id
        request.workspace_parameters = self._construct_workspace_parameter()
        request.start_state = start_robot_state
        request.goal_constraints = goal_constraints
        return request

    def _print_cartesian_request_error_code(
            self,
            error_code: int):
        """
        Print the Cartesian Path error code.

        :param error_code: The error code
        :type error_code: int
        """
        if (error_code == MoveItErrorCodes.SUCCESS):
            self._node.get_logger().info('Cartesian Path Success')
        elif (error_code == MoveItErrorCodes.UNDEFINED):
            self._node.get_logger().info('Cartesian Path Undefined')
        elif (error_code == MoveItErrorCodes.FAILURE):
            self._node.get_logger().info('Cartesian Path Failure')
        elif (error_code == MoveItErrorCodes.PLANNING_FAILED):
            self._node.get_logger().info('Cartesian Path Planning Failed')
        elif (error_code == MoveItErrorCodes.INVALID_MOTION_PLAN):
            self._node.get_logger().info('Cartesian Path Invalid Motion Plan')
        elif (error_code ==
              MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE):
            self._node.get_logger().info(
                'Cartesian Path Motion Plan Invalidated By Environment Change')
        elif (error_code == MoveItErrorCodes.CONTROL_FAILED):
            self._node.get_logger().info('Cartesian Path Control Failed')
        elif (error_code == MoveItErrorCodes.UNABLE_TO_AQUIRE_SENSOR_DATA):
            self._node.get_logger().info(
                'Cartesian Path Unable To Acquire Sensor Data')
        elif (error_code == MoveItErrorCodes.TIMED_OUT):
            self._node.get_logger().info('Cartesian Path Timed Out')
        elif (error_code == MoveItErrorCodes.PREEMPTED):
            self._node.get_logger().info('Cartesian Path Preempted')
        elif (error_code == MoveItErrorCodes.START_STATE_IN_COLLISION):
            self._node.get_logger().info(
                'Cartesian Path Start State In Collision')
        elif (error_code ==
              MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS):
            self._node.get_logger().info(
                'Cartesian Path Start State Violates Path Constraints')
        elif (error_code == MoveItErrorCodes.START_STATE_INVALID):
            self._node.get_logger().info('Cartesian Path Start State Invalid')
        elif (error_code == MoveItErrorCodes.GOAL_IN_COLLISION):
            self._node.get_logger().info('Cartesian Path Goal In Collision')
        elif (error_code == MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS):
            self._node.get_logger().info(
                'Cartesian Path Goal Violates Path Constraints')
        elif (error_code == MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED):
            self._node.get_logger().info(
                'Cartesian Path Goal Constraints Violated')
        elif (error_code == MoveItErrorCodes.GOAL_STATE_INVALID):
            self._node.get_logger().info('Cartesian Path Goal State Invalid')
        elif (error_code == MoveItErrorCodes.UNRECOGNIZED_GOAL_TYPE):
            self._node.get_logger().info(
                'Cartesian Path Unrecognized Goal Type')
        elif (error_code == MoveItErrorCodes.INVALID_GROUP_NAME):
            self._node.get_logger().info('Cartesian Path Invalid Group Name')
        elif (error_code == MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS):
            self._node.get_logger().info(
                'Cartesian Path Invalid Goal Constraints')
        elif (error_code == MoveItErrorCodes.INVALID_ROBOT_STATE):
            self._node.get_logger().info('Cartesian Path Invalid Robot State')
        elif (error_code == MoveItErrorCodes.INVALID_LINK_NAME):
            self._node.get_logger().info('Cartesian Path Invalid Link Name')
        elif (error_code == MoveItErrorCodes.INVALID_OBJECT_NAME):
            self._node.get_logger().info('Cartesian Path Invalid Object Name')
        elif (error_code == MoveItErrorCodes.FRAME_TRANSFORM_FAILURE):
            self._node.get_logger().info(
                'Cartesian Path Frame Transform Failure')
        elif (error_code == MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE):
            self._node.get_logger().info(
                'Cartesian Path Collision Checking Unavailable')
        elif (error_code == MoveItErrorCodes.ROBOT_STATE_STALE):
            self._node.get_logger().info('Cartesian Path Robot State Stale')
        elif (error_code == MoveItErrorCodes.SENSOR_INFO_STALE):
            self._node.get_logger().info('Cartesian Path Sensor Info Stale')
        elif (error_code == MoveItErrorCodes.COMMUNICATION_FAILURE):
            self._node.get_logger().info(
                'Cartesian Path Communication Failure')
        elif (error_code == MoveItErrorCodes.CRASH):
            self._node.get_logger().info('Cartesian Path Crash')
        elif (error_code == MoveItErrorCodes.ABORT):
            self._node.get_logger().info('Cartesian Path Abort')
        elif (error_code == MoveItErrorCodes.NO_IK_SOLUTION):
            self._node.get_logger().info('Cartesian Path No IK Solution')
        else:
            self._node.get_logger().info(
                f'Unknown Error Code: {error_code}')
