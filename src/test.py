import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class CompressedImagePublisher(Node):
    def __init__(self):
        super().__init__('compressed_image_publisher')

        self.bridge = CvBridge()

        # 퍼블리셔 생성 (압축된 이미지 전송을 위한 토픽)
        self.color_publisher_ = self.create_publisher(CompressedImage, '/camera/color/compressed', 10)

        # 원본 이미지 구독자
        self.color_subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.color_listener_callback,
            10)

    def color_listener_callback(self, data):
        try:
            # ROS 이미지 메시지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            # 이미지를 JPEG로 압축 (매개변수 설정 가능)
            ret, buffer = cv2.imencode('.jpg', cv_image, [int(cv2.IMWRITE_JPEG_QUALITY), 80])  # 품질을 80으로 설정 (0-100)

            if not ret:
                self.get_logger().error("Failed to encode image")
                return

            # 압축된 이미지를 ROS 메시지로 변환
            color_compressed_image = CompressedImage()
            color_compressed_image.header = data.header
            color_compressed_image.format = "jpeg"
            color_compressed_image.data = buffer.tobytes()

            # 압축된 이미지 퍼블리시
            self.color_publisher_.publish(color_compressed_image)

        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert image: {e}")

def main(args=None):
    rclpy.init(args=args)
    compressed_image_publisher = CompressedImagePublisher()
    rclpy.spin(compressed_image_publisher)
    compressed_image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    

