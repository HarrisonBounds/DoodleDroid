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
RobotState Class.

The RobotState communicates through several ROS 2 protocols:

SUBSCRIBERS:
+ /joint_states (sensor_msgs.msg.JointState) - Joint states of the robot

SERVICES:
+ /compute_fk (moveit_msgs.srv.GetPositionFK) - Compute forward kinematics
+ /compute_ik (moveit_msgs.srv.GetPositionIK) - Compute inverse kinematics
"""

from builtin_interfaces.msg import Duration

from geometry_msgs.msg import Pose, PoseStamped

from moveit_msgs.msg import RobotState as RS
from moveit_msgs.srv import GetPositionFK, GetPositionIK

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node

from sensor_msgs.msg import JointState


class RobotState():
    """
    RobotState Class.

    The RobotState communicates through several ROS 2 protocols:

    SUBSCRIBERS:
    + /joint_states (sensor_msgs.msg.JointState) - Joint states of the robot

    SERVICES:
    + /compute_fk (moveit_msgs.srv.GetPositionFK) - Compute forward kinematics
    + /compute_ik (moveit_msgs.srv.GetPositionIK) - Compute inverse kinematics
    """

    def __init__(self, node: Node):
        """
        Initialize the RoboState.

        :param node: The ROS 2 node
        :type node: rclpy.node.Node
        """
        self._node = node
        self._callback_group = MutuallyExclusiveCallbackGroup()
        self._fk_client = self._node.create_client(
            GetPositionFK, '/compute_fk', callback_group=self._callback_group)
        self._ik_client = self._node.create_client(
            GetPositionIK, '/compute_ik', callback_group=self._callback_group)
        self._joint_sub = self._node.create_subscription(
            JointState, '/joint_states', self._joint_states_callback, 10)
        self._arm_group_name = 'fer_manipulator'
        self._hand_group_name = 'hand'
        self._arm_joint_names = ['fer_joint1', 'fer_joint2', 'fer_joint3',
                                 'fer_joint4', 'fer_joint5', 'fer_joint6',
                                 'fer_joint7']
        self._hand_joint_names = ['fer_finger_joint1', 'fer_finger_joint2']
        self._base_frame = 'base'
        self._ee_frame = 'fer_link8'
        self._fk_request = GetPositionFK.Request()
        self._ik_request = GetPositionIK.Request()
        self._arm_joint_states = JointState()
        self._hand_joint_states = JointState()
        self._ee_pose = Pose()

    async def get_ee_pose(self):
        """
        Get the current end effector pose.

        :return: The current end effector pose
        :rtype: geometry_msgs.msg.Pose
        """
        fk_response = await self.perform_FK()
        if fk_response.error_code.val ==\
                GetPositionFK.Response().error_code.SUCCESS:
            self._ee_pose = fk_response.pose_stamped[0].pose
            return self._ee_pose
        else:
            return None

    def get_arm_joint_states(self):
        """
        Get the current arm joint configuration.

        :return: The current joint configuration
        :rtype: sensor_msgs.msg.JointState
        """
        return self._arm_joint_states

    def update_arm_joint_states(self,
                                joint_states: JointState):
        """
        Update the arm joint states.

        :param joint_states: The joint states
        :type joint_states: sensor_msgs.msg.JointState
        """
        self._arm_joint_states = joint_states

    def get_hand_joint_states(self):
        """
        Get the current hand joint configuration.

        :return: The current joint configuration
        :rtype: sensor_msgs.msg.JointState
        """
        return self._hand_joint_states

    async def perform_IK(self,
                         pose: PoseStamped,
                         robot_state: RS = None):
        """
        Perform Inverse Kinematics.

        :param pose: The pose to compute IK
        :type pose: geometry_msgs.msg.PoseStamped
        :param robot_state: The robot state.
        :type robot_state: moveit_msgs.msg.RobotState

        :return: The result of the IK computation
        :rtype: moveit_msgs.srv.GetPositionIK.Response
        """
        # Create the request
        self._ik_request.ik_request.group_name = self._arm_group_name
        if robot_state is not None:
            self._ik_request.ik_request.robot_state = robot_state
        else:
            self._ik_request.ik_request.robot_state =\
                self._construct_robot_state_by_joint_states(
                    self.get_arm_joint_states())
        self._ik_request.ik_request.avoid_collisions = True
        self._ik_request.ik_request.ik_link_name = self._ee_frame

        self._ik_request.ik_request.pose_stamped = pose
        self._ik_request.ik_request.timeout = Duration(sec=3, nanosec=0)

        # Call the service
        result = await self._ik_client.call_async(self._ik_request)

        return result

    async def perform_FK(self,
                         robot_state: RS = None):
        """
        Perform Forward Kinematics.

        :param robot_state: The robot state.
        :type robot_state: moveit_msgs.msg.RobotState

        :return: The current pose
        :rtype: geometry_msgs.msg.Pose
        """
        self._fk_request.header.stamp = self._node.get_clock().now().to_msg()
        self._fk_request.header.frame_id = self._base_frame
        self._fk_request.fk_link_names = [self._ee_frame]
        if robot_state is not None:
            self._fk_request.robot_state = robot_state
        else:
            self._fk_request.robot_state =\
                self._construct_robot_state_by_joint_states(
                    self.get_arm_joint_states())
        fk_res = await self._fk_client.call_async(self._fk_request)
        return fk_res

    def _construct_robot_state_by_joint_states(self,
                                               joint_states: JointState):
        """
        Construct a robot state message by joint states.

        :param joint_states: The joint states
        :type joint_states: sensor_msgs.msg.JointState

        :return: The robot state message
        :rtype: moveit_msgs.msg.RobotState
        """
        robot_state = RS()
        robot_state.joint_state = joint_states
        return robot_state

    def _joint_states_callback(self,
                               msg: JointState):
        """
        Store the joint state message.

        :param msg: The joint state message
        :type msg: sensor_msgs.msg.JointState
        """
        # Wrap hand and arm joint states
        all_joint_states = [(self._arm_joint_states, self._arm_joint_names),
                            (self._hand_joint_states, self._hand_joint_names)]

        # Reset all joint states
        for js, joint_names in all_joint_states:
            js.header = msg.header
            js.name = []
            js.position = []
            js.velocity = []
            js.effort = []

        # Update joint states
        for idx_msg_name, msg_joint_name in enumerate(msg.name):
            for js, joint_names in all_joint_states:
                if msg_joint_name in joint_names:
                    js.name.append(msg_joint_name)
                    js.position.append(float(msg.position[idx_msg_name]))
                    js.velocity.append(float(0))
                    js.effort.append(float(0))
