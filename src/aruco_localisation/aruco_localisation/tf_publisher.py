"""
TF broadcaster node.

Plan step 9: subscribes to /aruco/pose, builds a TransformStamped and
broadcasts it so the marker is visible in RViz and usable by other
TF-based nodes.
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster


class TfPublisherNode(Node):

    def __init__(self):
        super().__init__('tf_publisher')

        self.declare_parameter('pose_topic', '/aruco/pose')
        self.declare_parameter('parent_frame', 'camera_link')
        self.declare_parameter('child_frame', 'aruco_marker')

        self.pose_topic = self.get_parameter(
            'pose_topic').get_parameter_value().string_value
        self.parent_frame = self.get_parameter(
            'parent_frame').get_parameter_value().string_value
        self.child_frame = self.get_parameter(
            'child_frame').get_parameter_value().string_value

        self.broadcaster = TransformBroadcaster(self)

        self.pose_sub = self.create_subscription(
            PoseStamped, self.pose_topic, self.pose_callback, 10)

        self.get_logger().info(
            f"Broadcasting TF '{self.parent_frame}' -> '{self.child_frame}' "
            f"from '{self.pose_topic}'"
        )

    def pose_callback(self, msg: PoseStamped):
        t = TransformStamped()

        t.header.stamp = msg.header.stamp
        # Prefer the frame carried by the pose message if it's set,
        # otherwise fall back to the configured parent frame.
        t.header.frame_id = msg.header.frame_id or self.parent_frame
        t.child_frame_id = self.child_frame

        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z

        t.transform.rotation = msg.pose.orientation

        self.broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = TfPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
