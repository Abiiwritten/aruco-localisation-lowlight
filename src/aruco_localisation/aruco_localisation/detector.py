#!/usr/bin/env python3

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Int32


class ArucoDetectorNode(Node):
    """Detect ArUco markers and publish their camera-relative pose."""

    def __init__(self):
        super().__init__('aruco_detector')

        # Parameters that can be changed from the command line or a launch file.
        self.declare_parameter('image_topic', '/camera/camera/camera/image_raw')

        # Marker size in meters.
        self.declare_parameter('marker_length', 0.15)
        self.declare_parameter('camera_frame', 'camera_frame')
        self.declare_parameter('display_window', True)

        image_topic = (
            self.get_parameter('image_topic')
            .get_parameter_value()
            .string_value
        )

        self.marker_length = (
            self.get_parameter('marker_length')
            .get_parameter_value()
            .double_value
        )

        self.camera_frame = (
            self.get_parameter('camera_frame')
            .get_parameter_value()
            .string_value
        )

        self.display_window = (
            self.get_parameter('display_window')
            .get_parameter_value()
            .bool_value
        )
        
        self.detection_publisher = self.create_publisher(
            Int32,
            '/aruco/detected',
            10
        )

        self.bridge = CvBridge()
        
        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_6X6_250
        )
        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_6X6_50
        )

        self.detector_parameters = cv2.aruco.DetectorParameters()

        self.aruco_detector = cv2.aruco.ArucoDetector(
            self.dictionary,
            self.detector_parameters
        )

        # ArUco detector configuration.
        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50
        )

        detector_parameters = cv2.aruco.DetectorParameters()
        self.detector_parameters = detector_parameters

        # Your camera calibration.
        self.camera_matrix = np.array([
            [386.2490539550781, 0.0, 325.697021484375],
            [0.0, 385.8675537109375, 247.0854034423828],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)

        self.dist_coeffs = np.array([
            -0.054759636521339417,
            0.06454092264175415,
            -0.0003352328494656831,
            0.0011062893318012357,
            -0.02064184844493866
        ], dtype=np.float64)

        # Subscribe to images published by the camera node.
        self.image_subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        # Publish the detected pose.
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/aruco/pose',
            10
        )
        self.total_frames_publisher = self.create_publisher(
            Int32,
            '/aruco/total_frames',
            10
        )

        self.detected_frames_publisher = self.create_publisher(
            Int32,
            '/aruco/detected_frames',
            10
        )

        # Publish the ID corresponding to the pose.
        self.id_publisher = self.create_publisher(
            Int32,
            '/aruco/id',
            10
        )

        # Publish the annotated camera image.
        self.annotated_image_publisher = self.create_publisher(
            Image,
            '/aruco/annotated_image',
            10
        )

        self.total_frames = 0
        self.detected_frames = 0
        self.max_frames = 1000
        self.finsihed = False

        self.get_logger().info('ArUco detector started')
        self.get_logger().info(f'Subscribing to: {image_topic}')
        self.get_logger().info('Publishing pose to: /aruco/pose')
        self.get_logger().info('Publishing marker ID to: /aruco/id')

    def image_callback(self, image_message: Image) -> None:
        """Process one ROS camera image."""
        
        if self.finsihed:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(
                image_message,
                desired_encoding='bgr8'
            )
        except Exception as error:
            self.get_logger().error(
                f'Could not convert ROS image to OpenCV: {error}'
            )
            return

        self.total_frames += 1
        if self.total_frames > self.max_frames:
            self.finsihed = True
            self.get_logger().info(
                f'Maximum frame count of {self.max_frames} reached. '
                'Stopping detection.'
            )
            return
        
        

        gray = cv2.cvtColor(
        frame,
        cv2.COLOR_BGR2GRAY
    )

        corners, ids, _ = self.aruco_detector.detectMarkers(gray)
        
        # ==========================================
        # RECORD DETECTION RESULT FOR EVERY FRAME
        # ==========================================

        detected_msg = Int32()

        if ids is not None and len(ids) > 0:
            detected_msg.data = 1
        else:
            detected_msg.data = 0

        self.detection_publisher.publish(detected_msg)
        
        if ids is not None and len(ids) > 0:
            self.detected_frames += 1

            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            marker_points = np.array([
            [-self.marker_length / 2,  self.marker_length / 2, 0],
            [ self.marker_length / 2,  self.marker_length / 2, 0],
            [ self.marker_length / 2, -self.marker_length / 2, 0],
            [-self.marker_length / 2, -self.marker_length / 2, 0]
        ], dtype=np.float32)

        rvecs = []
        tvecs = []

        for marker_corners in corners:
            success, rvec, tvec = cv2.solvePnP(
                marker_points,
                marker_corners,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )

            if success:
                rvecs.append(rvec)
                tvecs.append(tvec)

            for index, marker_id_array in enumerate(ids):
                marker_id = int(marker_id_array[0])

                rotation_vector = rvecs[index]
                translation_vector = tvecs[index]
                
                # ==========================================
                # VIEWING ANGLE RELATIVE TO CAMERA
                # ==========================================

                # Convert marker rotation vector to rotation matrix
                R_marker, _ = cv2.Rodrigues(rotation_vector)

                # Marker plane normal (+Z axis)
                marker_normal = R_marker @ np.array([0.0, 0.0, 1.0])

                # Camera optical axis (+Z in OpenCV camera frame)
                camera_axis = np.array([0.0, 0.0, 1.0])

                # Angle between marker normal and camera optical axis
                cos_angle = np.dot(marker_normal, camera_axis)
                cos_angle = np.clip(abs(cos_angle), -1.0, 1.0)

                viewing_angle = np.degrees(np.arccos(cos_angle))

                print(f"Viewing angle relative to camera: {viewing_angle:.2f} deg")
                
                rvec = np.asarray(rvec, dtype=np.float64).reshape(3, 1)
                tvec = np.asarray(tvec, dtype=np.float64).reshape(3, 1)

                # cv2.drawFrameAxes(
                #     frame,
                #     self.camera_matrix,
                #     self.dist_coeffs,
                #     rotation_vector,
                #     translation_vector,
                #     self.marker_length * 0.75
                # )

                distance = float(np.linalg.norm(translation_vector))

                centre_x = int(corners[index][0][:, 0].mean())
                centre_y = int(corners[index][0][:, 1].mean())

                cv2.putText(
                    frame,
                    f'ID: {marker_id} Dist: {distance:.3f}m Angle: {viewing_angle:.2f}deg',
                    (centre_x - 200, centre_y - 50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2
                )
                
                cv2.drawFrameAxes(
                frame,
                self.camera_matrix,
                self.dist_coeffs,
                rvec,
                tvec,
                0.05)

                pose_message = self.create_pose_message(
                    image_message,
                    rotation_vector,
                    translation_vector
                )

                id_message = Int32()
                id_message.data = marker_id

                self.pose_publisher.publish(pose_message)
                self.id_publisher.publish(id_message)

        # detection_rate = (
        #     100.0 * self.detected_frames / self.total_frames
        #     if self.total_frames > 0
        #     else 0.0
        # )
        total_msg = Int32()
        total_msg.data = self.total_frames
        self.total_frames_publisher.publish(total_msg)

        detected_msg = Int32()
        detected_msg.data = self.detected_frames
        self.detected_frames_publisher.publish(detected_msg)
        
        detection_rate = (
            self.detected_frames / self.total_frames
        ) * 100

        cv2.putText(
            frame,
            f'Detection rate: {detection_rate:.1f}%',
            (30, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2
        )

        cv2.putText(
            frame,
            f'Frames: {self.total_frames}/{self.max_frames}',
            (30, 75),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2
        )

        self.publish_annotated_image(frame, image_message)

        if self.display_window:
            cv2.imshow('ArUco Pose Estimation', frame)
            cv2.waitKey(1)
            
        if self.total_frames >= self.max_frames:
            self.get_logger().info(
                f'Trial complete: {self.detected_frames}/{self.total_frames} '
                f'detected ({detection_rate:.1f}%)'
            )

            self.finished = True
            rclpy.shutdown()
            return
            


    def create_pose_message(
        self,
        image_message: Image,
        rotation_vector: np.ndarray,
        translation_vector: np.ndarray
    ) -> PoseStamped:
        """Convert OpenCV rotation and translation vectors to PoseStamped."""

        rotation_matrix, _ = cv2.Rodrigues(rotation_vector)
        quaternion = self.rotation_matrix_to_quaternion(rotation_matrix)

        pose_message = PoseStamped()

        # Preserve the camera image timestamp.
        pose_message.header.stamp = image_message.header.stamp

        # Pose is expressed relative to the camera coordinate frame.
        if image_message.header.frame_id:
            pose_message.header.frame_id = image_message.header.frame_id
        else:
            pose_message.header.frame_id = self.camera_frame

        pose_message.pose.position.x = float(translation_vector[0])
        pose_message.pose.position.y = float(translation_vector[1])
        pose_message.pose.position.z = float(translation_vector[2])

        pose_message.pose.orientation.x = quaternion[0]
        pose_message.pose.orientation.y = quaternion[1]
        pose_message.pose.orientation.z = quaternion[2]
        pose_message.pose.orientation.w = quaternion[3]

        return pose_message

    def publish_annotated_image(
        self,
        frame: np.ndarray,
        original_message: Image
    ) -> None:
        """Publish the image containing marker outlines and axes."""

        try:
            annotated_message = self.bridge.cv2_to_imgmsg(
                frame,
                encoding='bgr8'
            )

            annotated_message.header = original_message.header
            self.annotated_image_publisher.publish(annotated_message)

        except Exception as error:
            self.get_logger().error(
                f'Could not publish annotated image: {error}'
            )

    @staticmethod
    def rotation_matrix_to_quaternion(
        rotation_matrix: np.ndarray
    ) -> tuple[float, float, float, float]:
        """Convert a 3x3 rotation matrix to an x, y, z, w quaternion."""

        matrix = rotation_matrix
        trace = float(np.trace(matrix))

        if trace > 0.0:
            scale = np.sqrt(trace + 1.0) * 2.0
            qw = 0.25 * scale
            qx = (matrix[2, 1] - matrix[1, 2]) / scale
            qy = (matrix[0, 2] - matrix[2, 0]) / scale
            qz = (matrix[1, 0] - matrix[0, 1]) / scale

        elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
            scale = np.sqrt(
                1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]
            ) * 2.0

            qw = (matrix[2, 1] - matrix[1, 2]) / scale
            qx = 0.25 * scale
            qy = (matrix[0, 1] + matrix[1, 0]) / scale
            qz = (matrix[0, 2] + matrix[2, 0]) / scale

        elif matrix[1, 1] > matrix[2, 2]:
            scale = np.sqrt(
                1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]
            ) * 2.0

            qw = (matrix[0, 2] - matrix[2, 0]) / scale
            qx = (matrix[0, 1] + matrix[1, 0]) / scale
            qy = 0.25 * scale
            qz = (matrix[1, 2] + matrix[2, 1]) / scale

        else:
            scale = np.sqrt(
                1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]
            ) * 2.0

            qw = (matrix[1, 0] - matrix[0, 1]) / scale
            qx = (matrix[0, 2] + matrix[2, 0]) / scale
            qy = (matrix[1, 2] + matrix[2, 1]) / scale
            qz = 0.25 * scale

        quaternion = np.array([qx, qy, qz, qw], dtype=np.float64)
        norm = np.linalg.norm(quaternion)

        if norm > 0.0:
            quaternion /= norm

        return (
            float(quaternion[0]),
            float(quaternion[1]),
            float(quaternion[2]),
            float(quaternion[3])
        )

    def destroy_node(self) -> None:
        """Close the OpenCV window when the ROS node stops."""

        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = ArucoDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()