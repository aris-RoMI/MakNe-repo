import cv2
import numpy as np
import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32
from cv_bridge import CvBridge


class AccuracyOfDepth(Node):
    def __init__(self):
        super().__init__(node_name='accuracy_of_depth')
        
        # 구독자
        self.sub_original_depth_image = self.create_subscription(msg_type=Image, topic='/camera/depth/image_raw', callback=self.original_depth_callback, qos_profile=10)
        self.sub_converted_depth_image = self.create_subscription(msg_type=CompressedImage, topic='/converted_depth_image/compressed', callback=self.converted_depth_callback, qos_profile=10)

        # 발행자
        self.pub_original_depth = self.create_publisher(msg_type=Float32, topic='/original_depth', qos_profile=10)
        self.pub_converted_depth = self.create_publisher(msg_type=Float32, topic='/converted_depth', qos_profile=10)

        # 깊이 값 저장 변수
        self.original_depth = None
        self.converted_depth = None

        # 타이머 설정
        self.timer = self.create_timer(timer_period_sec=1.0, callback=self.timer_callback)

        self.bridge = CvBridge()

        self.get_logger().info(message='노드 초기화 완료')

    def original_depth_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(img_msg=msg, desired_encoding='passthrough')
        height, width = cv_image.shape
        self.original_depth = float(cv_image[height//2, width//2])

    def converted_depth_callback(self, msg):
        np_arr = np.frombuffer(buffer=msg.data, dtype=np.uint8)
        cv_image = cv2.imdecode(buf=np_arr, flags=cv2.IMREAD_ANYDEPTH)
        height, width = cv_image.shape
        self.converted_depth = float(cv_image[height//2, width//2])

    def timer_callback(self):
        if self.original_depth is not None and self.converted_depth is not None:
            # 오차 계산
            error = abs(self.original_depth - self.converted_depth)
            percent_error = (error / self.original_depth) * 100 if self.original_depth != 0 else float('inf')

            # 로그 출력
            self.get_logger().info(message=f'원본 깊이 값: {self.original_depth:.6f}')
            self.get_logger().info(message=f'변환 깊이 값: {self.converted_depth:.6f}')
            self.get_logger().info(message=f'오차: {error:.6f}')
            self.get_logger().info(message=f'오차(%): {percent_error:.6f}')

            # 값 초기화
            self.original_depth = None
            self.converted_depth = None


def main(args=None):
        rp.init(args=args)
        accuracy = AccuracyOfDepth()
        rp.spin(accuracy)
        accuracy.destroy_node()
        rp.shutdown()


if __name__ == '__main__':
     main()