#!/usr/bin/env python3
"""
OptiTrack <-> ArUco comparison node.

Uses aruco_localisation.utils for all quaternion/matrix math, the same
shared module detector.py already imports from -- so there's exactly
one implementation of that conversion logic in the whole package, not
three slightly-different copies.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
import numpy as np

from aruco_localisation.utils import pose_to_matrix, matrix_to_pose


# ============================================================
# Small local adapters -- utils.py's pose_to_matrix/matrix_to_pose
# work with plain (translation, quaternion) tuples, not ROS message
# types directly, so these bridge that gap without adding any new
# math of their own.
# ============================================================

def ros_pose_to_matrix(pose) -> np.ndarray:
    translation = (pose.position.x, pose.position.y, pose.position.z)
    quaternion = (
        pose.orientation.x, pose.orientation.y,
        pose.orientation.z, pose.orientation.w
    )
    return pose_to_matrix(translation, quaternion)


def matrix_to_pose_stamped(T: np.ndarray, stamp, frame_id: str) -> PoseStamped:
    translation, quaternion = matrix_to_pose(T)
    msg = PoseStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.pose.position.x = float(translation[0])
    msg.pose.position.y = float(translation[1])
    msg.pose.position.z = float(translation[2])
    msg.pose.orientation.x = float(quaternion[0])
    msg.pose.orientation.y = float(quaternion[1])
    msg.pose.orientation.z = float(quaternion[2])
    msg.pose.orientation.w = float(quaternion[3])
    return msg


# ============================================================
# NODE
# ============================================================

class OptiTrackTransform(Node):

    def __init__(self):
        super().__init__('optitrack_transform')

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        world_tf = TransformStamped()
        world_tf.header.stamp = self.get_clock().now().to_msg()
        world_tf.header.frame_id = 'map'
        world_tf.child_frame_id = 'world'
        world_tf.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(world_tf)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --------------------------------------------------
        # CALIBRATED OFFSETS -- applied at the data stage (inside
        # each callback below), not during a later composition step.
        # --------------------------------------------------

        self.BC_T_C = np.array([
            [ 0.99931745, -0.01430626,  0.03405815,  0.00422558],
            [ 0.03325054, -0.05333160, -0.99802312, -0.01757960],
            [ 0.01609435,  0.99847437, -0.05281951,  0.01983206],
            [ 0.00000000,  0.00000000,  0.00000000,  1.00000000]
        ])

        # Marker rigid body -> true ArUco tag origin. Translation is
        # your hand measurement; rotation is STILL a placeholder --
        # confirm via toggle testing before trusting results.
        theta_z = np.deg2rad(-90)
        R_z = np.array([
            [np.cos(theta_z), -np.sin(theta_z), 0],
            [np.sin(theta_z),  np.cos(theta_z), 0],
            [0,                0,               1]
        ])

        theta_y = np.deg2rad(-90)
        R_y = np.array([
            [ np.cos(theta_y), 0, np.sin(theta_y)],
            [ 0,               1, 0              ],
            [-np.sin(theta_y), 0, np.cos(theta_y)]
        ])

        self.BM_T_M = np.eye(4)
        self.BM_T_M[:3, :3] = R_z @ R_y   # order matters -- R_z @ R_y != R_y @ R_z
        self.BM_T_M = np.array([
            [-0.01986290,  0.99854390,  0.05015511, -0.10352273],
            [-0.03357170, -0.05080285,  0.99814428,  0.41411328],
            [ 0.99923891,  0.01814225,  0.03453191,  0.75369279],
            [ 0.00000000,  0.00000000,  0.00000000,  1.00000000],
        ])

        # Latest corrected camera optical pose in world.
        self.O_T_C = None

        self.declare_parameter(
            'camera_rigidbody_topic', '/vrpn_mocap/camera_rigidbody/pose')
        self.declare_parameter(
            'marker_rigidbody_topic', '/vrpn_mocap/rigidbody1/pose')
        self.declare_parameter('aruco_topic', '/aruco/pose')

        camera_topic = self.get_parameter(
            'camera_rigidbody_topic').get_parameter_value().string_value
        marker_topic = self.get_parameter(
            'marker_rigidbody_topic').get_parameter_value().string_value
        aruco_topic = self.get_parameter(
            'aruco_topic').get_parameter_value().string_value

        self.create_subscription(
            PoseStamped, camera_topic, self.camera_rigid_body_callback, qos)
        self.create_subscription(
            PoseStamped, marker_topic, self.marker_rigid_body_callback, qos)
        self.create_subscription(
            PoseStamped, aruco_topic, self.aruco_callback, 10)

        self.aruco_world_pub = self.create_publisher(
            PoseStamped, '/aruco/pose_optitrack', 10)
        self.marker_world_pub = self.create_publisher(
            PoseStamped, '/optitrack/marker_pose', 10)
        self.camera_optical_pub = self.create_publisher(
            PoseStamped, '/optitrack/camera_optical_pose', 10)

        self.get_logger().info(
            f"OptiTrack transform running: camera<-'{camera_topic}', "
            f"marker<-'{marker_topic}', aruco<-'{aruco_topic}'"
        )

    def publish_tf(self, T: np.ndarray, parent_frame: str, child_frame: str):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = parent_frame
        tf_msg.child_frame_id = child_frame

        translation, quaternion = matrix_to_pose(T)
        tf_msg.transform.translation.x = float(translation[0])
        tf_msg.transform.translation.y = float(translation[1])
        tf_msg.transform.translation.z = float(translation[2])
        tf_msg.transform.rotation.x = float(quaternion[0])
        tf_msg.transform.rotation.y = float(quaternion[1])
        tf_msg.transform.rotation.z = float(quaternion[2])
        tf_msg.transform.rotation.w = float(quaternion[3])

        self.tf_broadcaster.sendTransform(tf_msg)

    # ========================================================
    # CAMERA RIGID BODY -- BC_T_C applied right here, on arrival
    # ========================================================

    def camera_rigid_body_callback(self, msg):
        O_T_BC = ros_pose_to_matrix(msg.pose)
        self.publish_tf(O_T_BC, 'world', 'camera_rigidbody')

        self.O_T_C = O_T_BC @ self.BC_T_C

        self.publish_tf(self.O_T_C, 'world', 'camera_optical')
        self.camera_optical_pub.publish(
            matrix_to_pose_stamped(self.O_T_C, msg.header.stamp, 'world')
        )

    # ========================================================
    # MARKER RIGID BODY -- BM_T_M applied right here, on arrival
    # ========================================================

    def marker_rigid_body_callback(self, msg):
        O_T_BM = ros_pose_to_matrix(msg.pose)
        self.publish_tf(O_T_BM, 'world', 'marker_rigidbody')

        O_T_M_groundtruth = O_T_BM @ self.BM_T_M

        self.publish_tf(O_T_M_groundtruth, 'world', 'aruco_marker_groundtruth')
        self.marker_world_pub.publish(
            matrix_to_pose_stamped(
                O_T_M_groundtruth, msg.header.stamp, 'world')
        )

    # ========================================================
    # ARUCO -- composes two already-corrected transforms
    # ========================================================

    def aruco_callback(self, msg):
        if self.O_T_C is None:
            self.get_logger().warn(
                'Waiting for camera rigid-body pose...',
                throttle_duration_sec=2.0
            )
            return

        C_T_M = ros_pose_to_matrix(msg.pose)
        O_T_M = self.O_T_C @ C_T_M

        self.publish_tf(O_T_M, 'world', 'aruco_marker_vision')
        self.aruco_world_pub.publish(
            matrix_to_pose_stamped(O_T_M, msg.header.stamp, 'world')
        )


def main(args=None):
    rclpy.init(args=args)
    node = OptiTrackTransform()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()