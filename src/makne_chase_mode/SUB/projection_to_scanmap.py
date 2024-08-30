#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
from ultralytics import YOLO
import math
import tf2_ros
from tf2_geometry_msgs import do_transform_point

# Camera intrinsic parameters (to be adjusted as needed)
CAMERA_FX = 226.5603790283203
CAMERA_FY = 226.5603790283203
CAMERA_CX = 164.27264404296875
CAMERA_CY = 123.12284088134765

class PersonDetection3DNode(Node):
    def __init__(self):
        super().__init__('person_detection_3d_node')
        
        self.bridge = CvBridge()
        
        # Subscribe to resized color image and depth image
        self.color_sub = self.create_subscription(Image, '/converted_color_image', self.color_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/converted_depth_image', self.depth_callback, 10)
        
        # Subscribe to LaserScan
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Publisher for 3D point, Marker, and modified LaserScan
        self.point_pub = self.create_publisher(PointStamped, '/detected_person_3d', 10)
        self.marker_pub = self.create_publisher(Marker, '/person_marker', 10)
        self.modified_scan_pub = self.create_publisher(LaserScan, '/modified_scan', 10)
        
        # Initialize YOLOv8 model
        self.model = YOLO('yolov8n.pt')
        
        # Latest color and depth images
        self.latest_color_image = None
        self.latest_depth_image = None
        self.latest_scan = None
        
        # TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Use CUDA if available
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.get_logger().info(f"Using device: {self.device}")

    def color_callback(self, msg):
        self.latest_color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.process_images()

    def depth_callback(self, msg):
        self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def scan_callback(self, msg):
        self.latest_scan = msg

    def process_images(self):
        if self.latest_color_image is None or self.latest_depth_image is None or self.latest_scan is None:
            return

        # Detect persons using YOLOv8
        results = self.model(self.latest_color_image)

        person_positions = []

        for result in results:
            boxes = result.boxes
            for box in boxes:
                if box.cls == 0:  # class 0 is person in COCO dataset
                    x1, y1, x2, y2 = box.xyxy[0]
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                    # Draw bounding box
                    cv2.rectangle(self.latest_color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

                    # Calculate and draw center point
                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)
                    cv2.circle(self.latest_color_image, (center_x, center_y), 5, (0, 0, 255), -1)

                    # Get depth value at center point
                    depth = self.latest_depth_image[center_y, center_x]

                    if depth > 0:
                        # Calculate 3D coordinates
                        x = (center_x - CAMERA_CX) * depth / CAMERA_FX
                        y = (center_y - CAMERA_CY) * depth / CAMERA_FY
                        z = depth

                        # Create and publish 3D point
                        point_msg = PointStamped()
                        point_msg.header.frame_id = "camera_color_optical_frame"
                        point_msg.header.stamp = self.get_clock().now().to_msg()
                        point_msg.point.x = x / 1000.0  # convert to meters
                        point_msg.point.y = y / 1000.0
                        point_msg.point.z = z / 1000.0

                        # Transform point to LiDAR frame
                        try:
                            transform = self.tf_buffer.lookup_transform('front_base_scan', 'camera_color_optical_frame', rclpy.time.Time())
                            transformed_point = do_transform_point(point_msg, transform)
                            self.point_pub.publish(transformed_point)

                            # Store transformed point for LaserScan modification
                            person_positions.append((transformed_point.point.x, transformed_point.point.y))

                            # Publish marker for visualization
                            self.publish_marker(transformed_point)

                        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                            self.get_logger().warning('Failed to transform point')

        # Modify and publish LaserScan
        self.modify_and_publish_scan(person_positions)

        # # Display the image (optional, remove in production)
        # cv2.imshow('Detected Persons', self.latest_color_image)
        # cv2.waitKey(1)

    def publish_marker(self, point):
        marker = Marker()
        marker.header = point.header
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = point.point
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2  # Size of the marker
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.marker_pub.publish(marker)

    def modify_and_publish_scan(self, person_positions):
        if not self.latest_scan:
            return

        modified_scan = LaserScan()
        modified_scan.header = self.latest_scan.header
        modified_scan.angle_min = self.latest_scan.angle_min
        modified_scan.angle_max = self.latest_scan.angle_max
        modified_scan.angle_increment = self.latest_scan.angle_increment
        modified_scan.time_increment = self.latest_scan.time_increment
        modified_scan.scan_time = self.latest_scan.scan_time
        modified_scan.range_min = self.latest_scan.range_min
        modified_scan.range_max = self.latest_scan.range_max
        modified_scan.ranges = list(self.latest_scan.ranges)

        for x, y in person_positions:
            # Convert to polar coordinates
            r = math.sqrt(x*x + y*y)
            theta = math.atan2(y, x)

            # Find the corresponding index in the LaserScan
            index = int((theta - modified_scan.angle_min) / modified_scan.angle_increment)
            
            if 0 <= index < len(modified_scan.ranges):
                # Set the range to the person's position
                modified_scan.ranges[index] = r

        self.modified_scan_pub.publish(modified_scan)

def main(args=None):
    rclpy.init(args=args)
    node = PersonDetection3DNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()