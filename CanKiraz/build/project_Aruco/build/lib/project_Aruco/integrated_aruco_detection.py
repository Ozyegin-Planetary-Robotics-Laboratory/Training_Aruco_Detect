#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
import numpy as np
import cv2

# Dictionary of supported ArUco marker types
ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}


def aruco_display(corners, ids, rejected, image):
    """Display detected ArUco markers on the image."""
    if len(corners) > 0:
        ids = ids.flatten()
        for (markerCorner, markerID) in zip(corners, ids):
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners

            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)

            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)

            cv2.putText(image, str(markerID), (topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 0), 2)
    return image


class ArucoDetectionNode(Node):
    """ROS 2 node for ArUco marker detection."""

    def __init__(self):
        super().__init__('aruco_detection_node')
        self.get_logger().info('Aruco Detection Node started.')

        # ArUco type
        self.aruco_type = "DICT_4X4_100"
        if self.aruco_type not in ARUCO_DICT:
            self.get_logger().error(f"Invalid ArUco type: {self.aruco_type}")
            rclpy.shutdown()
            return

        # Load ArUco dictionary and parameters
        self.aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT[self.aruco_type])
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # Publisher for marker IDs
        self.marker_id_publisher = self.create_publisher(Int32, 'detected_marker_id', 10)
        self.corners_publisher = self.create_publisher(PoseArray, 'detected_marker_corners', 10)

        # Initialize processed IDs set
        self.processed_ids = set()        

        # Start video capture
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    def process_frame(self):
        """Process a single frame for ArUco marker detection."""
        ret, img = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture image.")
            return

        h, w, _ = img.shape
        width = 1000
        height = int(width * (h / w))
        img = cv2.resize(img, (width, height), interpolation=cv2.INTER_CUBIC)

        # Detect ArUco markers
        corners, ids, rejected = cv2.aruco.detectMarkers(img, self.aruco_dict, parameters=self.aruco_params)
        detected_markers = aruco_display(corners, ids, rejected, img)

        if ids is not None:
            for marker_id, marker_corners in zip(ids.flatten(), corners):
                # Check if the marker ID has already been processed
                if marker_id in self.processed_ids:
                    continue

                # Publish marker ID
                id_msg = Int32()
                id_msg.data = int(marker_id)
                self.marker_id_publisher.publish(id_msg)

                # Publish marker corners
                pose_array = PoseArray()
                pose_array.header.frame_id = "camera_frame"
                pose_array.header.stamp = self.get_clock().now().to_msg()

                for corner in marker_corners[0]:
                    pose = Pose()
                    pose.position.x = float(corner[0])  # X-coordinate
                    pose.position.y = float(corner[1])  # Y-coordinate
                    pose_array.poses.append(pose)

                self.corners_publisher.publish(pose_array)

                # Log the published data
                self.get_logger().info(f"Published marker ID: {marker_id}")
                self.get_logger().info(f"Published {len(pose_array.poses)} corners.")

                # Mark this ID as processed
                self.processed_ids.add(marker_id)

        # If no markers are detected, do not publish PoseArray
        else:
            self.get_logger().debug("No markers detected in this frame.")

        # Show image with detected markers
        img_with_markers = cv2.aruco.drawDetectedMarkers(img.copy(), corners, ids)
        cv2.imshow("ArUco Detection", detected_markers)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            self.get_logger().info("Quitting ArUco Detection.")
            cv2.destroyAllWindows()
            self.cap.release()
            rclpy.shutdown()

    def run(self):
        """Run the detection loop."""
        while rclpy.ok():
            self.process_frame()


def main(args=None):
    """Main function to initialize and run the node."""
    rclpy.init(args=args)
    node = ArucoDetectionNode()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
