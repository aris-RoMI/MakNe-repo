import cv2
import numpy as np
import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

class AccuracyOfDepth(Node):
    def __init__(self):
        super().__init__(node_name='accuracy_of_depth')
        
        # 구독자
        self.sub_original_depth_image = self.create_subscription(msg_type=Image, topic='/camera/depth/image_raw', callback=self.original_depth_callback, qos_profile=10)
        self.sub_converted_depth_image = self.create_subscription(msg_type=CompressedImage, topic='/converted_depth_image/compressed', callback=self.converted_depth_callback, qos_profile=10)

        # 깊이 값 저장 변수
        self.original_depth_image = None
        self.converted_depth_image = None

        # 타이머 설정
        self.timer = self.create_timer(timer_period_sec=3.0, callback=self.timer_callback)

        self.bridge = CvBridge()

        self.get_logger().info(message='노드 초기화 완료')

    def original_depth_callback(self, msg):
        try:
            self.original_depth_image = self.bridge.imgmsg_to_cv2(img_msg=msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'원본 이미지 변환 오류: {str(e)}')

    def converted_depth_callback(self, msg):
        try:
            np_arr = np.frombuffer(buffer=msg.data, dtype=np.uint8)
            self.converted_depth_image = cv2.imdecode(buf=np_arr, flags=cv2.IMREAD_UNCHANGED)
        except Exception as e:
            self.get_logger().error(f'변환된 이미지 디코딩 오류: {str(e)}')

    def timer_callback(self):
        if self.original_depth_image is None or self.converted_depth_image is None:
            self.get_logger().warn('이미지가 아직 수신되지 않았습니다.')
            return

        try:
            orig_height, orig_width = self.original_depth_image.shape
            conv_height, conv_width = self.converted_depth_image.shape

            self.get_logger().info(f'원본 이미지 크기: {orig_width}x{orig_height}')
            self.get_logger().info(f'변환 이미지 크기: {conv_width}x{conv_height}')

            # 원본 이미지를 변환된 이미지 크기로 리사이징
            resized_original = cv2.resize(self.original_depth_image, (conv_width, conv_height))

            # 여러 지점에서 깊이 값 비교
            points = [(conv_width//2, conv_height//2), (conv_width//4, conv_height//4), (3*conv_width//4, 3*conv_height//4)]
            for x, y in points:
                original_depth = resized_original[y, x]
                converted_depth = self.converted_depth_image[y, x]
                error = abs(float(original_depth) - float(converted_depth))
                percent_error = (error / float(original_depth)) * 100 if original_depth != 0 else float('inf')

                self.get_logger().info(f'위치 ({x}, {y}):')
                self.get_logger().info(f'  원본 깊이 값: {original_depth}')
                self.get_logger().info(f'  변환 깊이 값: {converted_depth}')
                self.get_logger().info(f'  오차: {error:.6f}')
                self.get_logger().info(f'  오차(%): {percent_error:.6f}')

            # 전체 이미지 비교
            mse = np.mean((resized_original.astype(float) - self.converted_depth_image.astype(float))**2)
            self.get_logger().info(f'전체 이미지 MSE: {mse:.6f}')

            # 데이터 타입 확인
            self.get_logger().info(f'원본 이미지 타입: {self.original_depth_image.dtype}')
            self.get_logger().info(f'변환 이미지 타입: {self.converted_depth_image.dtype}')

        except Exception as e:
            self.get_logger().error(f'이미지 처리 중 오류 발생: {str(e)}')

        # 값 초기화
        self.original_depth_image = None
        self.converted_depth_image = None

def main(args=None):
    rp.init(args=args)
    accuracy = AccuracyOfDepth()
    rp.spin(accuracy)
    accuracy.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()