#!/usr/bin/env python3

import time
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from scipy.spatial.transform import Rotation

# ============================================================
# USER SETTINGS
# ============================================================

# Number of INTERNAL checkerboard corners
CHECKERBOARD_COLS = 10       # <-- CHANGE
CHECKERBOARD_ROWS = 7       # <-- CHANGE

# Physical checkerboard square size [m]
SQUARE_SIZE = 0.028         # <-- CHANGE


# ------------------------------------------------------------
# CAMERA INTRINSICS
#
# These are your previously measured values.
# IMPORTANT:
# Only use them if this is the SAME camera/resolution/config.
# ------------------------------------------------------------

CAMERA_MATRIX = np.array([
            [386.2490539550781, 0.0, 325.697021484375],
            [0.0, 385.8675537109375, 247.0854034423828],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)

DIST_COEFFS = np.array([
            -0.054759636521339417,
            0.06454092264175415,
            -0.0003352328494656831,
            0.0011062893318012357,
            -0.02064184844493866
        ], dtype=np.float64)


# ============================================================
# TRANSFORM FUNCTIONS
# ============================================================

def quaternion_to_rotation_matrix(qx, qy, qz, qw):

    return np.array([
        [
            1 - 2*(qy*qy + qz*qz),
            2*(qx*qy - qz*qw),
            2*(qx*qz + qy*qw)
        ],
        [
            2*(qx*qy + qz*qw),
            1 - 2*(qx*qx + qz*qz),
            2*(qy*qz - qx*qw)
        ],
        [
            2*(qx*qz - qy*qw),
            2*(qy*qz + qx*qw),
            1 - 2*(qx*qx + qy*qy)
        ]
    ], dtype=np.float64)


def rotation_matrix_to_quaternion(R):

    trace = np.trace(R)

    if trace > 0.0:

        s = np.sqrt(trace + 1.0) * 2.0

        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s

    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:

        s = np.sqrt(
            1.0 + R[0, 0] - R[1, 1] - R[2, 2]
        ) * 2.0

        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s

    elif R[1, 1] > R[2, 2]:

        s = np.sqrt(
            1.0 + R[1, 1] - R[0, 0] - R[2, 2]
        ) * 2.0

        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s

    else:

        s = np.sqrt(
            1.0 + R[2, 2] - R[0, 0] - R[1, 1]
        ) * 2.0

        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s

    q = np.array([qx, qy, qz, qw])

    return q / np.linalg.norm(q)


def quaternion_to_matrix(q):

    return quaternion_to_rotation_matrix(
        q[0], q[1], q[2], q[3]
    )


def pose_to_matrix(pose):
    """
    Convert geometry_msgs/Pose to a 4x4 transformation matrix.
    """

    R = quaternion_to_rotation_matrix(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    )

    T = np.eye(4)

    T[:3, :3] = R

    T[:3, 3] = np.array([
        pose.position.x,
        pose.position.y,
        pose.position.z
    ])

    return T


def average_quaternions(quaternions):
    """
    Calculate an average quaternion from multiple samples.
    """

    A = np.zeros((4, 4))

    for q in quaternions:

        q = q / np.linalg.norm(q)
        q = q.reshape(4, 1)

        A += q @ q.T

    eigenvalues, eigenvectors = np.linalg.eigh(A)

    q_avg = eigenvectors[:, np.argmax(eigenvalues)]

    if q_avg[3] < 0:
        q_avg *= -1

    return q_avg / np.linalg.norm(q_avg)


# ============================================================
# CALIBRATION NODE
# ============================================================

class CameraRigidBodyExtrinsicCalibrator(Node):

    def __init__(self):

        super().__init__(
            'camera_rigidbody_extrinsic_calibrator'
        )

        self.bridge = CvBridge()
        
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # ----------------------------------------------------
        # Latest OptiTrack poses
        # ----------------------------------------------------

        self.camera_rb_pose = None
        self.board_rb_pose = None

        self.camera_rb_time = None
        self.board_rb_time = None

        self.samples = []

        self.last_sample_time = 0.0


        # ====================================================
        # VRPN QoS
        #
        # Your OptiTrack VRPN topics use BEST_EFFORT.
        # ====================================================

        vrpn_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )


        # ====================================================
        # TOPICS
        #
        # CHANGE THESE IF YOUR NAMES ARE DIFFERENT
        # ====================================================

        CAMERA_RB_TOPIC = \
            '/vrpn_mocap/camera_rigidbody/pose'

        BOARD_RB_TOPIC = \
            '/vrpn_mocap/checkerboard_rigidbody/pose'

        IMAGE_TOPIC = \
            '/camera/camera/color/image_raw'


        # ====================================================
        # SUBSCRIBERS
        # ====================================================

        self.create_subscription(
            PoseStamped,
            CAMERA_RB_TOPIC,
            self.camera_rb_callback,
            vrpn_qos
        )

        self.create_subscription(
            PoseStamped,
            BOARD_RB_TOPIC,
            self.board_rb_callback,
            vrpn_qos
        )

        self.create_subscription(
            Image,
            IMAGE_TOPIC,
            self.image_callback,
            10
        )
        


        self.get_logger().info(
            'Camera rigid-body extrinsic calibrator started.'
        )

        self.get_logger().info(
            'Waiting for camera, checkerboard and OptiTrack poses...'
        )
        
    


    # ========================================================
    # OPTITRACK CALLBACKS
    # ========================================================

    def camera_rb_callback(self, msg):

        self.camera_rb_pose = msg.pose

        self.camera_rb_time = (
            msg.header.stamp.sec
            + msg.header.stamp.nanosec * 1e-9
        )


    def board_rb_callback(self, msg):

        self.board_rb_pose = msg.pose

        self.board_rb_time = (
            msg.header.stamp.sec
            + msg.header.stamp.nanosec * 1e-9
        )


    # ========================================================
    # IMAGE CALLBACK
    # ========================================================

    def image_callback(self, msg):

        if self.camera_rb_pose is None:
            return

        if self.board_rb_pose is None:
            return


        # ----------------------------------------------------
        # ROS image -> OpenCV
        # ----------------------------------------------------

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        gray = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2GRAY
        )


        # ----------------------------------------------------
        # Find checkerboard
        # ----------------------------------------------------

        pattern_size = (
            CHECKERBOARD_COLS,
            CHECKERBOARD_ROWS
        )

        found, corners = cv2.findChessboardCornersSB(
            gray,
            pattern_size
        )


        if found:
            
                    # OpenCV checkerboard origin = first detected internal corner
            origin = tuple(corners[0].ravel().astype(int))

            cv2.circle(
                frame,
                origin,
                10,
                (0, 0, 255),
                -1
            )

            cv2.putText(
                frame,
                "ORIGIN",
                (origin[0] + 15, origin[1]),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2
            )

            # Refine corner locations
            corners = cv2.cornerSubPix(
                gray,
                corners,
                (11, 11),
                (-1, -1),
                (
                    cv2.TERM_CRITERIA_EPS
                    + cv2.TERM_CRITERIA_MAX_ITER,
                    30,
                    0.001
                )
            )


            # =================================================
            # CREATE CHECKERBOARD 3D POINTS
            #
            # Checkerboard frame P:
            #
            # origin = first internal checkerboard corner
            #
            # Z = 0 because checkerboard is planar
            # =================================================

            object_points = np.zeros(
                (
                    CHECKERBOARD_ROWS
                    * CHECKERBOARD_COLS,
                    3
                ),
                dtype=np.float32
            )

            object_points[:, :2] = (
                np.mgrid[
                    0:CHECKERBOARD_COLS,
                    0:CHECKERBOARD_ROWS
                ]
                .T
                .reshape(-1, 2)
            )

            object_points *= SQUARE_SIZE


            # =================================================
            # CAMERA -> CHECKERBOARD
            #
            # solvePnP returns:
            #
            # C_T_P
            # =================================================

            success, rvec, tvec = cv2.solvePnP(
                object_points,
                corners,
                CAMERA_MATRIX,
                DIST_COEFFS
            )


            if success:

                R_C_P, _ = cv2.Rodrigues(rvec)

                C_T_P = np.eye(4)

                C_T_P[:3, :3] = R_C_P

                C_T_P[:3, 3] = (
                    tvec.reshape(3)
                )


                # =============================================
                # OPTITRACK TRANSFORMS
                # =============================================

                O_T_BC = pose_to_matrix(
                    self.camera_rb_pose
                )

                O_T_BP = pose_to_matrix(
                    self.board_rb_pose
                )
                
                # ==========================================================
                # TRANSFORM DIRECTION DEBUG
                # ==========================================================

                BC_T_BP = np.linalg.inv(O_T_BC) @ O_T_BP

                print("\n========== FRAME DEBUG ==========")

                print("\nOptiTrack: camera RB -> board RB")
                print(BC_T_BP)

                print("\nOpenCV: camera optical -> checkerboard")
                print(C_T_P)

                print("\nOptiTrack translation:")
                print(BC_T_BP[:3, 3])

                print("\nOpenCV translation:")
                print(C_T_P[:3, 3])

                print("\nOptiTrack rotation:")
                print(BC_T_BP[:3, :3])

                print("\nOpenCV rotation:")
                print(C_T_P[:3, :3])

                print("=================================\n")
                
                # -------------------------------------------------
                # DEBUG: compare camera-to-board measurements
                # -------------------------------------------------

                # OptiTrack: camera rigid body -> checkerboard rigid body
                BC_T_BP = np.linalg.inv(O_T_BC) @ O_T_BP

                # Translation measured by OptiTrack
                opti_translation = BC_T_BP[:3, 3]

                # Translation measured by OpenCV solvePnP
                opencv_translation = C_T_P[:3, 3]

                print("\n----- CALIBRATION DEBUG -----")

                print("OptiTrack camera RB -> board RB:")
                print(opti_translation)

                print(
                    "OptiTrack distance:",
                    np.linalg.norm(opti_translation),
                    "m"
                )

                print()

                print("OpenCV camera -> checkerboard:")
                print(opencv_translation)

                print(
                    "OpenCV distance:",
                    np.linalg.norm(opencv_translation),
                    "m"
                )

                print("-----------------------------\n")


                # =============================================
                # CHECKERBOARD RIGID BODY -> CHECKERBOARD FRAME
                #
                # INITIAL ASSUMPTION:
                #
                # The OptiTrack board rigid-body frame has been
                # deliberately aligned with the checkerboard
                # coordinate frame.
                #
                # If not, THIS MUST BE REPLACED later.
                # =============================================

                BP_T_P = np.array([
                    [1.0, 0.0, 0.0, 0.036],
                    [0.0, 0.0, -1.0, 0.015],
                    [0.0, 1.0, 0.0,  0.036],
                    [0.0, 0.0, 0.0,  1.000]
                ])


                # Checkerboard pose in OptiTrack world

                O_T_P = (
                    O_T_BP
                    @ BP_T_P
                )
                
                # R_BC_C_expected = np.array([
                #     [1.0, 0.0,  0.0],
                #     [0.0, 0.0, -1.0],
                #     [0.0, 1.0,  0.0]
                # ])

                # opencv_in_camera_rb = R_BC_C_expected @ C_T_P[:3, 3]

                # print("\n===== AXIS TEST =====")
                # print("OptiTrack camera RB -> board:")
                # print(BC_T_BP[:3, 3])

                # print("OpenCV transformed into approximate camera RB axes:")
                # print(opencv_in_camera_rb)
                # print("=====================\n")


                # =============================================
                # SOLVE:
                #
                # Camera rigid body -> camera optical frame
                #
                # BC_T_C =
                #
                # inv(O_T_BC)
                # @ O_T_P
                # @ inv(C_T_P)
                # =============================================

                BC_T_C = (
                    np.linalg.inv(O_T_BC)
                    @ O_T_P
                    @ np.linalg.inv(C_T_P)
                )


                # =============================================
                # SAVE ONE SAMPLE PER SECOND
                # =============================================

                current_time = time.time()

                if (
                    current_time
                    - self.last_sample_time
                    >= 1.0
                ):

                    self.samples.append(
                        BC_T_C.copy()
                    )

                    self.last_sample_time = (
                        current_time
                    )

                    translation = BC_T_C[:3, 3]

                    self.get_logger().info(
                        f'Sample {len(self.samples)} | '
                        f'x={translation[0]:.4f}, '
                        f'y={translation[1]:.4f}, '
                        f'z={translation[2]:.4f} m'
                    )


            # Draw detected checkerboard
            cv2.drawChessboardCorners(
                frame,
                pattern_size,
                corners,
                found
            )
            
            cv2.drawFrameAxes(
            frame,
            CAMERA_MATRIX,
            DIST_COEFFS,
            rvec,
            tvec,
            SQUARE_SIZE * 3
            )


        # ----------------------------------------------------
        # Display sample count
        # ----------------------------------------------------

        cv2.putText(
            frame,
            f'Samples: {len(self.samples)}',
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 0),
            2
        )

        cv2.imshow(
            'Camera Rigid Body Extrinsic Calibration',
            frame
        )

        cv2.waitKey(1)
        
    
    def publish_calibrated_camera_tf(self, BC_T_C):

            t = TransformStamped()

            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'camera_rigidbody'
            t.child_frame_id = 'camera_optical_frame'

            # Translation
            t.transform.translation.x = float(BC_T_C[0, 3])
            t.transform.translation.y = float(BC_T_C[1, 3])
            t.transform.translation.z = float(BC_T_C[2, 3])

            # Rotation matrix -> quaternion
            quat = Rotation.from_matrix(BC_T_C[:3, :3]).as_quat()

            t.transform.rotation.x = float(quat[0])
            t.transform.rotation.y = float(quat[1])
            t.transform.rotation.z = float(quat[2])
            t.transform.rotation.w = float(quat[3])

            self.static_tf_broadcaster.sendTransform(t)

            self.get_logger().info(
                'Published camera_rigidbody -> camera_optical_frame TF'
            )


    # ========================================================
    # CALCULATE FINAL RESULT
    # ========================================================

    def calculate_result(self):

        if len(self.samples) < 5:

            print()
            print(
                'ERROR: Not enough calibration samples.'
            )

            print(
                f'Only {len(self.samples)} samples collected.'
            )

            return


        translations = []

        quaternions = []


        for T in self.samples:

            translations.append(
                T[:3, 3]
            )

            quaternions.append(
                rotation_matrix_to_quaternion(
                    T[:3, :3]
                )
            )


        translations = np.array(
            translations
        )

        quaternions = np.array(
            quaternions
        )


        # ----------------------------------------------------
        # Translation statistics
        # ----------------------------------------------------

        translation_mean = np.mean(
            translations,
            axis=0
        )

        translation_std = np.std(
            translations,
            axis=0
        )
        
        


        # ----------------------------------------------------
        # Average rotation
        # ----------------------------------------------------

        quaternion_mean = average_quaternions(
            quaternions
        )

        R_mean = quaternion_to_matrix(
            quaternion_mean
        )


        # ----------------------------------------------------
        # Final homogeneous transform
        # ----------------------------------------------------

        BC_T_C = np.eye(4)

        BC_T_C[:3, :3] = R_mean

        BC_T_C[:3, 3] = translation_mean
        
    


        # ====================================================
        # PRINT RESULT
        # ====================================================

        print()
        print('==============================================')
        print('CAMERA EXTRINSIC CALIBRATION RESULT')
        print('==============================================')

        print()
        print(
            'Transform: CAMERA RIGID BODY -> CAMERA OPTICAL'
        )

        print()
        print(
            f'Number of samples: {len(self.samples)}'
        )


        print()
        print('Translation [m]:')

        print(
            f'x = {translation_mean[0]:.8f}'
        )

        print(
            f'y = {translation_mean[1]:.8f}'
        )

        print(
            f'z = {translation_mean[2]:.8f}'
        )


        print()
        print('Translation standard deviation [m]:')

        print(
            f'x std = {translation_std[0]:.8f}'
        )

        print(
            f'y std = {translation_std[1]:.8f}'
        )

        print(
            f'z std = {translation_std[2]:.8f}'
        )


        print()
        print('Quaternion [x, y, z, w]:')

        print(
            f'qx = {quaternion_mean[0]:.8f}'
        )

        print(
            f'qy = {quaternion_mean[1]:.8f}'
        )

        print(
            f'qz = {quaternion_mean[2]:.8f}'
        )

        print(
            f'qw = {quaternion_mean[3]:.8f}'
        )


        print()
        print('BC_T_C =')

        print(BC_T_C)


        print()
        print('==============================================')
        print(
            'COPY THESE VALUES INTO optitrack_transform.py'
        )
        print('==============================================')

        print()

        print(
            'CAMERA_OFFSET_TRANSLATION = '
            f'[{translation_mean[0]:.8f}, '
            f'{translation_mean[1]:.8f}, '
            f'{translation_mean[2]:.8f}]'
        )

        print(
            'CAMERA_OFFSET_QUATERNION = '
            f'[{quaternion_mean[0]:.8f}, '
            f'{quaternion_mean[1]:.8f}, '
            f'{quaternion_mean[2]:.8f}, '
            f'{quaternion_mean[3]:.8f}]'
        )

        print()
        
        self.publish_calibrated_camera_tf(BC_T_C)


# ============================================================
# MAIN
# ============================================================

def main(args=None):

    rclpy.init(args=args)

    node = CameraRigidBodyExtrinsicCalibrator()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        print()
        print(
            'Calibration stopped - calculating result...'
        )

    finally:

        node.calculate_result()

        cv2.destroyAllWindows()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()