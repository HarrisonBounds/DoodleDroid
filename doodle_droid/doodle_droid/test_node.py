from functools import partial
import rclpy
from rclpy.node import Node
# from moveit2_api.robot_state import RobotState
from doodle_droid.motion_planner import MotionPlanner
from geometry_msgs.msg import PoseStamped, PoseArray
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Quaternion, Vector3, Pose
from std_srvs.srv import Empty
from action_msgs.msg import GoalStatus
import time

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        # self._robot_state = RobotState(self)
        self._motion_planner = MotionPlanner(self)

        self._test_server = self.create_service(Empty, "test", self._test_callback)

    def construct_line_waypoints(self, start, end, num_points):
        waypoints = PoseArray()
        waypoints.header.frame_id = "base"
        waypoints.header.stamp = self.get_clock().now().to_msg()

        for i in range(num_points):
            pose = Pose()
            pose.position.x = start.position.x + i * (end.position.x - start.position.x) / (num_points - 1)
            pose.position.y = start.position.y + i * (end.position.y - start.position.y) / (num_points - 1)
            pose.position.z = start.position.z + i * (end.position.z - start.position.z) / (num_points - 1)
            pose.orientation = start.orientation
            waypoints.poses.append(pose)

        return waypoints

    async def _test_callback(self, request, response):
        # arm_js = self._robot_state.get_arm_joint_states()
        # self.get_logger().info(f"Arm joint states: {arm_js}")

        # hand_js = self._robot_state.get_hand_joint_states()
        # self.get_logger().info(f"Hand joint states: {hand_js}")

        # ee_pose = await self._robot_state.get_ee_pose()
        # if ee_pose is not None:
        #     self.get_logger().info(f"End effector pose: {ee_pose}")
        # else:
        #     self.get_logger().info("Failed to get end effector pose.")
        
        z = 0.166
        start1 = Pose()
        start1.position = Point(x=0.315211, y=0.0526, z=z)
        start1.orientation = Quaternion(x=0.9238792,
                                        y=-0.3826833,
                                        z=0.0003047,
                                        w=0.0007357)
        await self._motion_planner.plan_c(start1, execute=True)

        # end1 = Pose()
        # end1.position = Point(x=0.25, y=-0.05, z=z)
        # end1.orientation = Quaternion(x=0.9238792,
        #                               y=-0.3826833,
        #                               z=0.0003047,
        #                               w=0.0007357)
        # waypoints = self.construct_line_waypoints(start1, end1, 10)
        # await self._motion_planner.execute_waypoints(waypoints, 1.0)

        # time.sleep(1)

        # start2 = end1
        # start2.position.x = start2.position.x + 0.1
        # start2.position.z = start2.position.z - 0.001
        # end2 = start1
        # end2.position.x = end2.position.x + 0.1
        # end2.position.z = start2.position.z - 0.001
        # waypoints = self.construct_line_waypoints(start2, end2, 10)
        # await self._motion_planner.execute_waypoints(waypoints, 0.5)

        return response

def main():
    rclpy.init()
    node = TestNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
