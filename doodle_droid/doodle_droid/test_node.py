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


import tf2_ros
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        # self._robot_state = RobotState(self)
        self._motion_planner = MotionPlanner(self)

        self._test_server = self.create_service(Empty, "test", self._test_callback)

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

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
        
        try:
            base_tag_tf = self.buffer.lookup_transform('fer_hand', 'cam_link_cal', rclpy.time.Time())

            self.get_logger().info("x " + str(base_tag_tf.transform.translation.x) )
            self.get_logger().info("y " + str(base_tag_tf.transform.translation.y) )
            self.get_logger().info("z " + str(base_tag_tf.transform.translation.z) )
            self.get_logger().info("\n")

            self.get_logger().info("x " + str(base_tag_tf.transform.rotation.x) )
            self.get_logger().info("y " + str(base_tag_tf.transform.rotation.y) )
            self.get_logger().info("z " + str(base_tag_tf.transform.rotation.z) )
            self.get_logger().info("w " + str(base_tag_tf.transform.rotation.w) )
            self.get_logger().info("\n")




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
        # z = 0.166
        # start1 = Pose()
        # start1.position = Point(x=0.315211, y=0.0526, z=z)
        # start1.orientation = Quaternion(x=0.9238792,
        #                                 y=-0.3826833,
        #                                 z=0.0003047,
        #                                 w=0.0007357)
        # await self._motion_planner.plan_c(start1, execute=True)

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
