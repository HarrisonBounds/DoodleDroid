"""
Calibrator
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

import numpy as np
from apriltag import apriltag
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import Point, Quaternion, Pose, TransformStamped, Vector3

import tf2_ros
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# from tf2_msgs.

from doodle_droid.motion_planner import MotionPlanner
from doodle_droid.robot_state import RobotState
from std_srvs.srv import Empty
from action_msgs.msg import GoalStatus



def angle_from_centroid(pt, centroid):
    dx = pt[0] - centroid[0]
    dy = pt[1] - centroid[1]
    return np.arctan2(dy, dx)

def distance(p1,p2):
        return np.linalg.norm(p1-p2)


class Calibrator(Node):
    """

    """

    def __init__(self):
        """Run initializer."""
        super().__init__('calibrator')

        self.frequency = 1000.0

        # Timers
        self.create_timer(1/self.frequency, self.timer_callback)

        self.surface_publisher = self.create_publisher(Marker, 'surface_marker', 10)
        self.drawing_dims_publisher = self.create_publisher(Vector3, "drawing_dims", 10)
        self.surface_pose_publisher = self.create_publisher(Pose, "surface_pose", 10)
        self.cam_sub = self.create_subscription(Image, '/camera/camera/color/image_rect', self.get_image_callback, 10)

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

        self.tagsize = 0.1016  # using 4 inch apriltags
        self.pen_offset = 0.138


        # self.static_broadcaster = StaticTransformBroadcaster(self)
        # base_aruco_tf = TransformStamped()
        # base_aruco_tf.header.stamp = self.get_clock().now().to_msg()
        # base_aruco_tf.header.frame_id = 'base'
        # base_aruco_tf.child_frame_id = 'tag'
        # base_aruco_tf.transform.translation.x = 0.5 # change to match camera mounting
        # base_aruco_tf.transform.translation.y = 0.0
        # base_aruco_tf.transform.translation.z = 0.02

        # self.static_broadcaster.sendTransform(base_aruco_tf)

        ##### MAGIC NUMBERS
        # self.static_broadcaster = StaticTransformBroadcaster(self)
        # world_camera_tf = TransformStamped()
        # world_camera_tf.header.stamp = self.get_clock().now().to_msg()
        # world_camera_tf.header.frame_id = 'fer_hand'
        # world_camera_tf.child_frame_id = 'camera_link'
        # world_camera_tf.transform.translation.x = 0.05366 # change to match camera mounting
        # world_camera_tf.transform.translation.y = -0.0193
        # world_camera_tf.transform.translation.z = 0.0025

        # world_camera_tf.transform.rotation.x = 0.0 # change to match camera mounting
        # world_camera_tf.transform.rotation.y = -0.7071045
        # world_camera_tf.transform.rotation.z = 0.0
        # world_camera_tf.transform.rotation.w = 0.7071045

        # self.static_broadcaster.sendTransform(world_camera_tf)

        ##### FROM THIRD CALIBRATION (PLEASE WORK)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        world_camera_tf = TransformStamped()
        world_camera_tf.header.stamp = self.get_clock().now().to_msg()
        world_camera_tf.header.frame_id = 'fer_hand'
        world_camera_tf.child_frame_id = 'camera_link'
        world_camera_tf.transform.translation.x = 0.04077700478326539
        world_camera_tf.transform.translation.y = 0.01631535438771024
        world_camera_tf.transform.translation.z = 0.015104079163571737 + 0.0135


        world_camera_tf.transform.rotation.x = -0.007100127498884945
        world_camera_tf.transform.rotation.y = -0.711166685920879
        world_camera_tf.transform.rotation.z = -0.005910717653049716
        world_camera_tf.transform.rotation.w = 0.7029627276340044

        self.static_broadcaster.sendTransform(world_camera_tf)

        ########## OBTAINED FROM HAND EYE CALIBRATION
        # self.static_broadcaster = StaticTransformBroadcaster(self)
        # ee_camcal = TransformStamped()
        # ee_camcal.header.stamp = self.get_clock().now().to_msg()
        # ee_camcal.header.frame_id = 'fer_hand'
        # # ee_camcal.child_frame_id = 'camera_color_optical_frame'
        # ee_camcal.child_frame_id = 'calibrated'
        # ee_camcal.transform.translation.x =  0.040942 # change to match camera mounting
        # ee_camcal.transform.translation.y = 0.030943
        # ee_camcal.transform.translation.z = 0.014650

        # ee_camcal.transform.rotation.x = -0.001400 # change to match camera mounting
        # ee_camcal.transform.rotation.y = -0.006516
        # ee_camcal.transform.rotation.z = -0.711002
        # ee_camcal.transform.rotation.w = 0.703158

        # self.static_broadcaster.sendTransform(ee_camcal)

        # ## FOR DETERMINING CAMERA_LINK TF
        # self.static_broadcaster = StaticTransformBroadcaster(self)
        # cal_link = TransformStamped()
        # cal_link.header.stamp = self.get_clock().now().to_msg()
        # cal_link.header.frame_id = 'calibrated'
        # # cal_link.child_frame_id = 'camera_color_optical_frame'
        # cal_link.child_frame_id = 'cam_link_cal'
        # cal_link.transform.translation.x =  0.014632730869929307 # change to match camera mounting
        # cal_link.transform.translation.y = 0.0
        # cal_link.transform.translation.z = 0.00029088615521028535

        # cal_link.transform.rotation.x = 0.5015941997979191# change to match camera mounting
        # cal_link.transform.rotation.y = -0.5005221105941958
        # cal_link.transform.rotation.z = 0.4947025076383143
        # cal_link.transform.rotation.w = 0.5031404420952272
        # self.static_broadcaster.sendTransform(cal_link)

        self.motion_planner = MotionPlanner(self)
        self.robot_state = RobotState(self)
        self.in_position = False
        self.surface_published = False

        self.calibrate_server = self.create_service(Empty, "calibrate", self.calibrate_callback)
        self.manual_calibrate_server = self.create_service(Empty, "manual_calibrate", self.manual_calibrate_callback)
        self.test_calibrate_server = self.create_service(Empty, "test_calibrate", self.test_calibrate_callback)


        self.get_logger().info("calibrator initialized")

    

    def timer_callback(self):
        """
        Run the main timer for controlling the calibrator.

        """
        # self.get_logger().info("not in position")
        pass
        if self.in_position and not self.surface_published:
            # self.get_logger().info("IN POSITION")
            if self.current_image is not None:
                # cv_image = self.bridge.compressed_imgmsg_to_cv2(self.current_image, desired_encoding='passthrough')
                # gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                try:
                    base_tag_tf = self.buffer.lookup_transform('base', 'tag0', rclpy.time.Time())
                    pose = Pose()
                    pose.position.x = base_tag_tf.transform.translation.x
                    pose.position.y = base_tag_tf.transform.translation.y
                    pose.position.z = base_tag_tf.transform.translation.z
                    pose.orientation = base_tag_tf.transform.rotation

                    self.positions.append(np.array([pose.position.x, pose.position.y, pose.position.z]))
                    self.orientations.append(np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]))

                    base_tag_tf = self.buffer.lookup_transform('base', 'tag1', rclpy.time.Time())
                    pose.position.x = base_tag_tf.transform.translation.x
                    pose.position.y = base_tag_tf.transform.translation.y
                    pose.position.z = base_tag_tf.transform.translation.z
                    pose.orientation = base_tag_tf.transform.rotation

                    self.positions.append(np.array([pose.position.x, pose.position.y, pose.position.z]))
                    self.orientations.append(np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]))


                    base_tag_tf = self.buffer.lookup_transform('base', 'tag2', rclpy.time.Time())
                    pose.position.x = base_tag_tf.transform.translation.x
                    pose.position.y = base_tag_tf.transform.translation.y
                    pose.position.z = base_tag_tf.transform.translation.z
                    pose.orientation = base_tag_tf.transform.rotation

                    self.positions.append(np.array([pose.position.x, pose.position.y, pose.position.z]))
                    self.orientations.append(np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]))


                    base_tag_tf = self.buffer.lookup_transform('base', 'tag3', rclpy.time.Time())
                    pose.position.x = base_tag_tf.transform.translation.x
                    pose.position.y = base_tag_tf.transform.translation.y
                    pose.position.z = base_tag_tf.transform.translation.z
                    pose.orientation = base_tag_tf.transform.rotation

                    self.positions.append(np.array([pose.position.x, pose.position.y, pose.position.z]))
                    self.orientations.append(np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]))

                    # self.surface_pose_publisher.publish(pose)

                    if len(self.positions) > 119:

                        pose = Pose()

                        avg_position = np.mean(self.positions, axis=0) # calculate average position of four april tags (hopefully more accurate)
                        self.pose.position.x = avg_position[0]
                        self.pose.position.y = avg_position[1]
                        self.pose.position.z = avg_position[2]
                


                        avg_orientation = np.mean(self.orientations, axis=0) # calculate average orientation of four april tags (hopefully more accurate)
                        avg_orientation /= np.linalg.norm(avg_orientation)
                        self.pose.orientation.x = avg_orientation[0]
                        self.pose.orientation.y = avg_orientation[1]
                        self.pose.orientation.z = avg_orientation[2]
                        self.pose.orientation.w = avg_orientation[3]

                        self.get_logger().info("x " + str(self.pose.position.x) )
                        self.get_logger().info("y " + str(self.pose.position.y) )
                        self.get_logger().info("z " + str(self.pose.position.z) )
                        self.get_logger().info("\n")

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
            
       
    async def calibrate_callback(self,request, response):
        # z = 0.188


        # try:
        #     cam_optical_tf = self.buffer.lookup_transform('base','tag', rclpy.time.Time())
        #     pose = Pose()
        #     self.pose.position.x = cam_optical_tf.transform.translation.x
        #     self.pose.position.y = cam_optical_tf.transform.translation.y
        #     self.pose.position.z = cam_optical_tf.transform.translation.z
        #     self.pose.orientation = cam_optical_tf.transform.rotation

        #     self.get_logger().info("x " + str(cam_optical_tf.transform.translation.x) )
        #     self.get_logger().info("y " + str(cam_optical_tf.transform.translation.y) )
        #     self.get_logger().info("z " + str(cam_optical_tf.transform.translation.z) )
        #     self.get_logger().info("\n")


        #     self.get_logger().info("x " + str(cam_optical_tf.transform.rotation.x) )
        #     self.get_logger().info("y " + str(cam_optical_tf.transform.rotation.y) )
        #     self.get_logger().info("z " + str(cam_optical_tf.transform.rotation.z) )
        #     self.get_logger().info("w " + str(cam_optical_tf.transform.rotation.w) )
        #     self.get_logger().info("\n")

        #     self.surface_pose_publisher.publish(self.pose)
        #     self.surface_published = True




        # except tf2_ros.LookupException as e:
        #     # the frames don't exist yet
        #     self.get_logger().info(f'Lookup exception: {e}')
        # except tf2_ros.ConnectivityException as e:
        #     # the tf tree has a disconnection
        #     self.get_logger().info(f'Connectivity exception: {e}')
        # except tf2_ros.ExtrapolationException as e:
        #     # the times are two far apart to extrapolate
        #     self.get_logger().info(f'Extrapolation exception: {e}')
        # pass


        start1 = Pose()
        start1.position = Point(x=0.45, y=0.05, z=0.75)
  
        start1.orientation = Quaternion(x=0.9238792,
                                        y=-0.3826833,
                                        z=0.0003047,
                                        w=0.0007357)
        result, status = await self.motion_planner.plan_p(start1.position,start1.orientation,execute=True)

        self.motion_planner.print_status(status)

        self.in_position = True
        self.surface_published = False
        self.positions = []
        self.orientations = []

        return response


    async def test_calibrate_callback(self,request, response):
        # if self.pose_determined:
    
        move_pose = self.pose
        # move_position.z += 0.185
        # move_pose.position.z += 0.138
        move_pose.position.z += 0.1405

        move_pose.orientation = Quaternion(x=0.9238792,
                                y=-0.3826833,
                                z=0.0003047,
                                w=0.0007357)
        result, status = await self.motion_planner.plan_c(move_pose,execute=True)

  
        return response
    

    async def manual_calibrate_callback(self,request, response):
        # if self.pose_determined:
    
        self.pose = await self.robot_state.get_ee_pose()

        move_pose = self.pose
        # move_position.z += 0.185
        move_pose.position.z -=0.001

        move_pose.orientation = Quaternion(x=0.9238792,
                                y=-0.3826833,
                                z=0.0003047,
                                w=0.0007357)
        result, status = await self.motion_planner.plan_c(move_pose,execute=True)

        self.get_logger().info("x " + str(self.pose.position.x) )
        self.get_logger().info("y " + str(self.pose.position.y) )
        self.get_logger().info("z " + str(self.pose.position.z) )
        self.get_logger().info("\n")

  
        return response



    def get_image_callback(self, msg):
        self.current_image = msg

    def create_marker(self, m_id, name, frame, pose, scale, col, a):
        """
        Create a standard cube marker object for visualization.

        Args
        ----
            m_id (int) The unique id of the marker
            name (string) The name of the marker
            frame (string) The name of the marker's header frame
            pose (Pose) The pose of the marker
            scale ( [float, float, float] ) The scale of the marker
            col ( [float, float, float] ) Marker color

        Returns
        -------
            marker (visualization_msgs/msg/Marker) The generated marker

        """
        marker = Marker()

        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = frame
        marker.ns = name
        marker.id = m_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose = pose

        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]

        marker.color.r = col[0]
        marker.color.g = col[1]
        marker.color.b = col[2]
        marker.color.a = a

        marker.lifetime = Duration(sec=0, nanosec=0)

        return marker


def main(args=None):
    """Run main function."""
    rclpy.init(args=args)
    node = Calibrator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
