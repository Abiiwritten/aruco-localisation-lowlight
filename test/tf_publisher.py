#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster

# THIS CODE IS USED TO TRANSFORM THE POSE FROM detector.py INTO THE SAME RAME AS THE OPTITRACK

class ArucoTFPublisher(Node):

    def __init__(self):
        super().__init__('aruco_tf_publisher')

        self.tf_broadcaster = TransformBroadcaster(self)

        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/aruco/pose',
            self.pose_callback,
            10
        )

        self.get_logger().info(
            'Subscribed to /aruco/pose and broadcasting ArUco TF'
        )

    def pose_callback(self, pose_msg: PoseStamped) -> None:
        transform = TransformStamped()

        # Use the timestamp and camera frame from the received pose.
        transform.header.stamp = pose_msg.header.stamp

        # This should be "default_cam".
        transform.header.frame_id = pose_msg.header.frame_id

        # Name of the marker frame.
        transform.child_frame_id = 'aruco_marker'

        # Copy position from PoseStamped into the TF translation.
        transform.transform.translation.x = pose_msg.pose.position.x
        transform.transform.translation.y = pose_msg.pose.position.y
        transform.transform.translation.z = pose_msg.pose.position.z

        # Copy quaternion orientation into the TF rotation.
        transform.transform.rotation.x = pose_msg.pose.orientation.x
        transform.transform.rotation.y = pose_msg.pose.orientation.y
        transform.transform.rotation.z = pose_msg.pose.orientation.z
        transform.transform.rotation.w = pose_msg.pose.orientation.w

        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)

    node = ArucoTFPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()