"""
OptiTrack calibration and comparison node.

Combines plan steps 10-12:
  10. subscribe to /aruco/pose and /vrpn_mocap/rigidbody1/pose,
      define C_T_M, O_T_B, B_T_M and the rigid transform relationship
          C_T_M = C_T_O * O_T_B * B_T_M
  11. solve for the camera-to-OptiTrack calibration transform
          C_T_O = C_T_M * inv(B_T_M) * inv(O_T_B)
      averaged over `calibration_samples` synchronized pose pairs,
      then held fixed and reused.
  12. once C_T_O is known, publish:
          C_T_B      = C_T_O * O_T_B          (rigid body in camera frame)
          C_T_M_opti = C_T_B * B_T_M          (OptiTrack-derived marker pose,
                                                in camera frame)
      for comparison against the ArUco-estimated /aruco/pose.

Frame naming follows the plan: C = camera, O = OptiTrack world, B = rigid
body, M = marker. "X_T_Y" reads as "transform of Y expressed in X",
i.e. it maps a point in frame Y into frame X.
"""

import os
import yaml
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import PoseStamped

from aruco_localisation.utils import (
    pose_to_matrix,
    matrix_to_pose,
    invert_transform,
)


class OptitrackTransformNode(Node):

    def __init__(self):
        super().__init__('optitrack_transform')

        self._declare_parameters()
        self._load_parameters()

        # B_T_M is a fixed, hand-measured calibration input (step 10).
        self.B_T_M = pose_to_matrix(
            self.rigid_body_translation, self.rigid_body_rotation_xyzw)

        # Latest received poses, used to pair up synchronized samples.
        self.latest_aruco_pose = None
        self.latest_optitrack_pose = None

        # Calibration state (step 11).
        self.calibration_samples = []
        self.C_T_O = None  # solved once len(samples) >= target, then frozen
        self.calibrated = False

        # --- Publishers (step 12 outputs) ---
        self.rigid_body_in_camera_pub = self.create_publisher(
            PoseStamped, '/optitrack/rigid_body_in_camera', 10)
        self.marker_in_camera_opti_pub = self.create_publisher(
            PoseStamped, '/optitrack/marker_in_camera', 10)

        # --- Subscribers (step 10) ---
        qos = QoSProfile(depth=10)
        self.aruco_sub = self.create_subscription(
            PoseStamped, self.aruco_pose_topic, self.aruco_callback, qos)
        self.optitrack_sub = self.create_subscription(
            PoseStamped, self.optitrack_pose_topic, self.optitrack_callback, qos)

        self.get_logger().info(
            f"Subscribed to '{self.aruco_pose_topic}' and "
            f"'{self.optitrack_pose_topic}'. Collecting "
            f"{self.target_samples} samples to solve C_T_O."
        )

    # ------------------------------------------------------------------
    # Parameters
    # ------------------------------------------------------------------
    def _declare_parameters(self):
        self.declare_parameter('aruco_pose_topic', '/aruco/pose')
        self.declare_parameter(
            'optitrack_pose_topic', '/vrpn_mocap/rigidbody1/pose')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('rigid_body_translation', [0.0, 0.0, 0.0])
        self.declare_parameter('rigid_body_rotation_xyzw', [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter('calibration_samples', 30)
        self.declare_parameter(
            'calibration_output_path', 'config/camera_to_optitrack.yaml')

    def _load_parameters(self):
        gp = lambda name: self.get_parameter(name).get_parameter_value()

        self.aruco_pose_topic = gp('aruco_pose_topic').string_value
        self.optitrack_pose_topic = gp('optitrack_pose_topic').string_value
        self.world_frame = gp('world_frame').string_value
        self.camera_frame = gp('camera_frame').string_value
        self.rigid_body_translation = list(
            gp('rigid_body_translation').double_array_value)
        self.rigid_body_rotation_xyzw = list(
            gp('rigid_body_rotation_xyzw').double_array_value)
        self.target_samples = gp('calibration_samples').integer_value
        self.calibration_output_path = gp('calibration_output_path').string_value

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------
    def aruco_callback(self, msg: PoseStamped):
        self.latest_aruco_pose = msg
        self._try_process_pair()

    def optitrack_callback(self, msg: PoseStamped):
        self.latest_optitrack_pose = msg
        self._try_process_pair()

    def _try_process_pair(self):
        if self.latest_aruco_pose is None or self.latest_optitrack_pose is None:
            return

        if not self.calibrated:
            self._accumulate_calibration_sample()
        else:
            self._publish_calibrated_outputs()

    # ------------------------------------------------------------------
    # Step 11: solve C_T_O = C_T_M * inv(B_T_M) * inv(O_T_B)
    # ------------------------------------------------------------------
    def _accumulate_calibration_sample(self):
        C_T_M = pose_to_matrix(
            _position_to_list(self.latest_aruco_pose.pose.position),
            _orientation_to_list(self.latest_aruco_pose.pose.orientation),
        )
        O_T_B = pose_to_matrix(
            _position_to_list(self.latest_optitrack_pose.pose.position),
            _orientation_to_list(self.latest_optitrack_pose.pose.orientation),
        )

        B_T_M_inv = invert_transform(self.B_T_M)
        O_T_B_inv = invert_transform(O_T_B)

        C_T_O_sample = C_T_M @ B_T_M_inv @ O_T_B_inv
        self.calibration_samples.append(C_T_O_sample)

        self.get_logger().info(
            f'Collected calibration sample '
            f'{len(self.calibration_samples)}/{self.target_samples}'
        )

        if len(self.calibration_samples) >= self.target_samples:
            self._solve_and_freeze_calibration()

    def _solve_and_freeze_calibration(self):
        # Average translations directly; average rotations via quaternion
        # mean (sign-aligned) for a simple, adequate approximation. For
        # higher accuracy, replace with an SVD-based rotation averaging
        # method later.
        translations = []
        quaternions = []
        for m in self.calibration_samples:
            t, q = matrix_to_pose(m)
            translations.append(t)
            quaternions.append(q)

        translations = np.array(translations)
        mean_translation = translations.mean(axis=0)

        quaternions = np.array(quaternions)
        # Align signs so we don't average antipodal quaternions to zero.
        ref = quaternions[0]
        for i in range(len(quaternions)):
            if np.dot(quaternions[i], ref) < 0:
                quaternions[i] = -quaternions[i]
        mean_quaternion = quaternions.mean(axis=0)
        mean_quaternion /= np.linalg.norm(mean_quaternion)

        self.C_T_O = pose_to_matrix(mean_translation, mean_quaternion)
        self.calibrated = True

        self.get_logger().info(
            'Camera-to-OptiTrack calibration solved and frozen (C_T_O). '
            f'translation={mean_translation.tolist()}, '
            f'quaternion_xyzw={mean_quaternion.tolist()}'
        )

        self._save_calibration(mean_translation, mean_quaternion)

    def _save_calibration(self, translation, quaternion_xyzw):
        data = {
            'camera_to_optitrack': {
                'translation': [float(v) for v in translation],
                'rotation_xyzw': [float(v) for v in quaternion_xyzw],
            }
        }
        try:
            out_path = self.calibration_output_path
            os.makedirs(os.path.dirname(out_path) or '.', exist_ok=True)
            with open(out_path, 'w') as f:
                yaml.safe_dump(data, f)
            self.get_logger().info(f'Saved calibration to {out_path}')
        except OSError as e:
            self.get_logger().error(f'Failed to save calibration: {e}')

    # ------------------------------------------------------------------
    # Step 12: publish calibrated OptiTrack outputs
    # ------------------------------------------------------------------
    def _publish_calibrated_outputs(self):
        O_T_B = pose_to_matrix(
            _position_to_list(self.latest_optitrack_pose.pose.position),
            _orientation_to_list(self.latest_optitrack_pose.pose.orientation),
        )

        C_T_B = self.C_T_O @ O_T_B
        C_T_M_opti = C_T_B @ self.B_T_M

        stamp = self.latest_optitrack_pose.header.stamp

        self.rigid_body_in_camera_pub.publish(
            self._matrix_to_pose_msg(C_T_B, stamp))
        self.marker_in_camera_opti_pub.publish(
            self._matrix_to_pose_msg(C_T_M_opti, stamp))

    def _matrix_to_pose_msg(self, matrix, stamp):
        translation, quaternion = matrix_to_pose(matrix)

        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.camera_frame

        msg.pose.position.x = float(translation[0])
        msg.pose.position.y = float(translation[1])
        msg.pose.position.z = float(translation[2])

        msg.pose.orientation.x = float(quaternion[0])
        msg.pose.orientation.y = float(quaternion[1])
        msg.pose.orientation.z = float(quaternion[2])
        msg.pose.orientation.w = float(quaternion[3])

        return msg


def _position_to_list(position):
    return [position.x, position.y, position.z]


def _orientation_to_list(orientation):
    return [orientation.x, orientation.y, orientation.z, orientation.w]


def main(args=None):
    rclpy.init(args=args)
    node = OptitrackTransformNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
