import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import numpy as np
import cv2

class ImageDecompressor(Node):
    def __init__(self):
        super().__init__('image_converter')
        
        # 압축된 이미지 구독
        self.sub_compressed_color = self.create_subscription(msg_type=CompressedImage, topic='/converted_color_image/compressed', callback=self.color_callback, qos_profile=10)
        self.sub_compressed_depth = self.create_subscription(msg_type=CompressedImage, topic='/converted_depth_image/compressed', callback=self.depth_callback, qos_profile=10)
        
        # 변환된 이미지 발행
        self.pub_color = self.create_publisher(msg_type=Image, topic='/decompressed_color_image', qos_profile=10)
        self.pub_depth = self.create_publisher(msg_type=Image, topic='/decompressed_depth_image', qos_profile=10)
        
        self.br = CvBridge()

    def color_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        ros_image = self.br.cv2_to_imgmsg(cv_image, "bgr8")
        self.pub_color.publish(ros_image)

    def depth_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_ANYDEPTH)
        ros_image = self.br.cv2_to_imgmsg(cv_image, "16UC1")
        self.pub_depth.publish(ros_image)


def main(args=None):
    rp.init(args=args)
    decompressor = ImageDecompressor()
    rp.spin(decompressor)
    decompressor.destroy_node()
    rp.shutdown()


if __name__ == '__main__':
    main()