#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseArray


class ListenerNode(Node):
    """Listener node for subscribing to both marker IDs and their corner positions."""

    def __init__(self):
        super().__init__('listener')
        
        # Subscriptions
        self.id_subscription = self.create_subscription(
            Int32,
            'detected_marker_id',
            self.id_callback,
            10)
        
        self.corners_subscription = self.create_subscription(
            PoseArray,
            'detected_marker_corners',
            self.corners_callback,
            10)

        # Data storage
        self.latest_id = None
        self.latest_corners = None

    def id_callback(self, msg):
        """Callback for received marker IDs."""
        self.latest_id = msg.data
        self.get_logger().info(f"Received Marker ID: {self.latest_id}")

    def corners_callback(self, msg):
        """Callback for received marker corners."""
        self.latest_corners = msg.poses
        self.get_logger().info(f"Received {len(self.latest_corners)} corners.")
        
        # Log each corner's position
        for i, pose in enumerate(self.latest_corners):
            self.get_logger().info(f"Corner {i}: x={pose.position.x}, y={pose.position.y}")


def main(args=None):
    rclpy.init(args=args)
    node = ListenerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
