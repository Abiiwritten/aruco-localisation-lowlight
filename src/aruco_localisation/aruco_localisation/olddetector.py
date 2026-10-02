"""
ArUco marker detector node.

Combines plan steps 4-8:
  4. camera subscriber
  5. OpenCV ArUco marker detection
  6. camera calibration (loaded from params, not performed live here)
  7. solvePnP marker pose estimation
  8. publish the pose as a ROS message

Publishes:
  /aruco/pose             (geometry_msgs/PoseStamped)
  /aruco/id               (std_msgs/Int32)
  /aruco/total_frames     (std_msgs/Int32)
  /aruco/detected_frames  (std_msgs/Int32)
  /aruco/annotated_image  (sensor_msgs/Image)
"""

import numpy as np
import cv2

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from cv_bridge import CvBridge, CvBridgeError

from aruco_localisation.utils import rvec_to_quaternion


# Map plan's requested flag name -> OpenCV constant. Extend as needed.
SOLVEPNP_FLAGS = {
    'SOLVEPNP_IPPE_SQUARE': cv2.SOLVEPNP_IPPE_SQUARE,
    'SOLVEPNP_ITERATIVE': cv2.SOLVEPNP_ITERATIVE,
}


class ArucoDetectorNode(Node):

    def __init__(self):
        super().__init__('detector')

        self._declare_parameters()
        self._load_parameters()

        self.bridge = CvBridge()

        # --- Step 5: predefined ArUco dictionary + detector ---
        dictionary_id = getattr(cv2.aruco, self.dictionary_name, None)
        if dictionary_id is None:
            self.get_logger().error(
                f"Unknown ArUco dictionary '{self.dictionary_name}', "
                f"falling back to DICT_6X6_50."
            )
            dictionary_id = cv2.aruco.DICT_6X6_50
        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(
            self.aruco_dictionary, self.aruco_params
        )

        # 3D model points of the marker corners in its own local frame,
        # ordered to match cv2.aruco's corner ordering
        # (top-left, top-right, bottom-right, bottom-left).
        half = self.marker_length_m / 2.0
        self.object_points = np.array([
            [-half,  half, 0.0],
            [ half,  half, 0.0],
            [ half, -half, 0.0],
            [-half, -half, 0.0],
        ], dtype=np.float64)

        # Frame counters for /aruco/total_frames and /aruco/detected_frames
        self.total_frames = 0
        self.detected_frames = 0

        # --- Publishers ---
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco/pose', 10)
        self.id_pub = self.create_publisher(Int32, '/aruco/id', 10)
        self.total_frames_pub = self.create_publisher(
            Int32, '/aruco/total_frames', 10)
        self.detected_frames_pub = self.create_publisher(
            Int32, '/aruco/detected_frames', 10)
        if self.publish_annotated_image:
            self.annotated_pub = self.create_publisher(
                Image, '/aruco/annotated_image', 10)

        # --- Step 4: camera subscriber ---
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10)

        self.get_logger().info(
            f"ArUco detector listening on '{self.image_topic}', "
            f"dictionary={self.dictionary_name}, "
            f"marker_length={self.marker_length_m} m"
        )

    def _declare_parameters(self):
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('marker_frame_prefix', 'aruco_marker')
        self.declare_parameter('aruco_dictionary', 'DICT_6X6_50')        
        self.declare_parameter('marker_length_m', 0.10)
        self.declare_parameter('publish_annotated_image', True)
        self.declare_parameter('detection_flag', 'SOLVEPNP_IPPE_SQUARE')
        self.declare_parameter('camera_matrix',
            [386.2490539550781, 0.0, 325.697021484375,
            0.0, 385.8675537109375, 247.0854034423828,
            0.0, 0.0, 1.0])
        self.declare_parameter('distortion_coefficients',
            [-0.054759636521339417, 0.06454092264175415,
            -0.0003352328494656831, 0.0011062893318012357,
            -0.02064184844493866])

    def _load_parameters(self):
        self.image_topic = self.get_parameter(
            'image_topic').get_parameter_value().string_value
        self.camera_frame = self.get_parameter(
            'camera_frame').get_parameter_value().string_value
        self.marker_frame_prefix = self.get_parameter(
            'marker_frame_prefix').get_parameter_value().string_value
        self.dictionary_name = self.get_parameter(
            'aruco_dictionary').get_parameter_value().string_value
        self.marker_length_m = self.get_parameter(
            'marker_length_m').get_parameter_value().double_value
        self.publish_annotated_image = self.get_parameter(
            'publish_annotated_image').get_parameter_value().bool_value

        flag_name = self.get_parameter(
            'detection_flag').get_parameter_value().string_value
        self.solvepnp_flag = SOLVEPNP_FLAGS.get(
            flag_name, cv2.SOLVEPNP_IPPE_SQUARE)

        cam_matrix_flat = self.get_parameter(
            'camera_matrix').get_parameter_value().double_array_value
        self.camera_matrix = np.array(
            cam_matrix_flat, dtype=np.float64).reshape(3, 3)

        dist_coeffs = self.get_parameter(
            'distortion_coefficients').get_parameter_value().double_array_value
        self.dist_coeffs = np.array(dist_coeffs, dtype=np.float64)

    def image_callback(self, msg: Image):
        self.total_frames += 1

        # --- Step 4: convert ROS Image -> OpenCV image, handle errors ---
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'cv_bridge conversion failed: {e}')
            self._publish_frame_counts()
            return

        # --- Step 5: detect markers ---
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _rejected = self.aruco_detector.detectMarkers(gray)

        # Reject empty/invalid detections
        if ids is None or len(ids) == 0 or len(corners) == 0:
            self._publish_frame_counts()
            if self.publish_annotated_image:
                self._publish_annotated(cv_image, msg.header)
            return

        self.detected_frames += 1

        # Draw all detected markers for debugging
        annotated = cv_image.copy()
        cv2.aruco.drawDetectedMarkers(annotated, corners, ids)

        # --- Step 7: solvePnP pose estimation, per detected marker ---
        for marker_corners, marker_id in zip(corners, ids.flatten()):
            image_points = marker_corners.reshape(4, 2).astype(np.float64)

            success, rvec, tvec = cv2.solvePnP(
                self.object_points,
                image_points,
                self.camera_matrix,
                self.dist_coeffs,
                flags=self.solvepnp_flag,
            )

            if not success:
                self.get_logger().warn(
                    f'solvePnP failed for marker id={marker_id}')
                continue

            quaternion = rvec_to_quaternion(rvec)

            # --- Step 8: build and publish PoseStamped ---
            pose_msg = PoseStamped()
            pose_msg.header.stamp = msg.header.stamp
            pose_msg.header.frame_id = msg.header.frame_id or self.camera_frame

            pose_msg.pose.position.x = float(tvec[0])
            pose_msg.pose.position.y = float(tvec[1])
            pose_msg.pose.position.z = float(tvec[2])

            pose_msg.pose.orientation.x = float(quaternion[0])
            pose_msg.pose.orientation.y = float(quaternion[1])
            pose_msg.pose.orientation.z = float(quaternion[2])
            pose_msg.pose.orientation.w = float(quaternion[3])

            self.pose_pub.publish(pose_msg)
            self.id_pub.publish(Int32(data=int(marker_id)))

            cv2.drawFrameAxes(
                annotated, self.camera_matrix, self.dist_coeffs,
                rvec, tvec, self.marker_length_m * 0.5)

        self._publish_frame_counts()
        if self.publish_annotated_image:
            self._publish_annotated(annotated, msg.header)

    def _publish_frame_counts(self):
        self.total_frames_pub.publish(Int32(data=self.total_frames))
        self.detected_frames_pub.publish(Int32(data=self.detected_frames))

    def _publish_annotated(self, cv_image, header):
        try:
            img_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            img_msg.header = header
            self.annotated_pub.publish(img_msg)
        except CvBridgeError as e:
            self.get_logger().error(f'Failed to publish annotated image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
