import numpy as np
import cv2
import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge


class ImageConverter(Node):
    # Constants
    NODE_NAME = 'image_converter_main'
    SUB_COLOR_TOPIC = '/camera/color/image_raw'
    PUB_COLOR_TOPIC = '/converted_color_image/compressed'
    SUB_DEPTH_TOPIC = '/camera/depth/image_raw'
    PUB_DEPTH_TOPIC = '/converted_depth_image/compressed'
    QOS_PROFILE = 10
    RESIZE_WIDTH = 320
    RESIZE_HEIGHT = 240
    JPEG_QUALITY = 80
    COLOR_ENCODING = 'bgr8'
    DEPTH_ENCODING = 'passthrough'

    def __init__(self):
        super().__init__(node_name=self.NODE_NAME)
        
        # 컬러 이미지 구독자 및 발행자
        self.sub_color = self.create_subscription(msg_type=Image, 
                                                  topic=self.SUB_COLOR_TOPIC, 
                                                  callback=self.color_callback, 
                                                  qos_profile=self.QOS_PROFILE)
        self.pub_color = self.create_publisher(msg_type=CompressedImage, 
                                               topic=self.PUB_COLOR_TOPIC, 
                                               qos_profile=self.QOS_PROFILE)
        
        # 깊이 이미지 구독자 및 발행자
        self.sub_depth = self.create_subscription(msg_type=Image, 
                                                  topic=self.SUB_DEPTH_TOPIC, 
                                                  callback=self.depth_callback, 
                                                  qos_profile=self.QOS_PROFILE)
        self.pub_depth = self.create_publisher(msg_type=CompressedImage, 
                                               topic=self.PUB_DEPTH_TOPIC, 
                                               qos_profile=self.QOS_PROFILE)
        
        self.bridge = CvBridge()

    def color_callback(self, msg):
        # 1. ROS2 Image 메시지를 OpenCV 이미지 형식으로 변환
        cv_image = self.bridge.imgmsg_to_cv2(img_msg=msg, desired_encoding=self.COLOR_ENCODING)
        
        # 2. 이미지 리사이징 적용
        resized_image = cv2.resize(src=cv_image, dsize=(self.RESIZE_WIDTH, self.RESIZE_HEIGHT))
        
        # 3. JPEG 압축 적용
        _, jpeg_image = cv2.imencode(ext='.jpg', img=resized_image, params=[cv2.IMWRITE_JPEG_QUALITY, self.JPEG_QUALITY])
        
        # 4. CompressedImage 메시지 생성 및 발행
        color_compressed_msg = CompressedImage()
        color_compressed_msg.header = msg.header
        color_compressed_msg.format = 'jpeg'
        color_compressed_msg.data = jpeg_image.tobytes()
        self.pub_color.publish(msg=color_compressed_msg)
        self.get_logger().info(message=f'컬러 이미지 사이즈: {resized_image.shape}')

    def depth_callback(self, msg):
        # 1. ROS2 Image 메시지를 OpenCV 이미지 형식으로 변환
        cv_image = self.bridge.imgmsg_to_cv2(img_msg=msg, desired_encoding=self.DEPTH_ENCODING)
        
        # 2. 이미지 리사이징 적용
        resized_image = cv2.resize(src=cv_image, dsize=(self.RESIZE_WIDTH, self.RESIZE_HEIGHT))
        
        # 3. PNG 압축 적용
        _, png_image = cv2.imencode(ext='.png', img=resized_image)
        
        # 4. CompressedImage 메시지 생성 및 발행
        depth_compressed_msg = CompressedImage()
        depth_compressed_msg.header = msg.header
        depth_compressed_msg.format = "png"
        depth_compressed_msg.data = png_image.tobytes()
        self.pub_depth.publish(msg=depth_compressed_msg)
        self.get_logger().info(message=f'깊이 이미지 사이즈: {resized_image.shape}')


def main(args=None):
    rp.init(args=args)
    converter = ImageConverter()
    rp.spin(converter)
    converter.destroy_node()
    rp.shutdown()


if __name__ == '__main__':
    main()
