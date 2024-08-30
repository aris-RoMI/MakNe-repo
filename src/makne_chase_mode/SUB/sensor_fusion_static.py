import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
# 좌표 변환을 위한 TF2 라이브러리
import tf2_ros
import tf2_geometry_msgs


class SensorFusion(Node):
    def __init__(self):
        super().__init__(node_name='sensor_fusion')
        
        # TF2 버퍼와 리스터 초기화
        # 서로 다른 좌표계 간의 변환 정보를 얻기 위해 사용
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(buffer=self.tf_buffer, node=self)
        
        '''
        [ 구독자 정의 ] 
        * 카메라 이미지
        * 카메라 정보 (내부 파라미터)
        * LiDAR 스캔 데이터
        '''
        self.sub_image_raw = self.create_subscription(msg_type=Image, topic='/camera/color/image_raw', callback=self.image_raw_callback, qos_profile=10)
        self.sub_camera_info = self.create_subscription(msg_type=CameraInfo, topic='/camera/color/camera_info', callback=self.camera_info_callback, qos_profile=10)
        self.sub_laser_scan = self.create_subscription(msg_type=LaserScan, topic='/scan', callback=self.laser_scan_callback, qos_profile=10)
        
        ''' [ 발행자 정의 ] '''
        self.pub_fusion_image = self.create_publisher(msg_type=Image, topic='/fusion_image', qos_profile=10)
        
        self.bridge = CvBridge() # ROS2 이미지와 OpenCV 이미지 간의 변환
        '''
        노드가 시작될 때 아직 정보를 받기 전이므로
        필요한 데이터들을 미리 None으로 초기화
        이를 통해, 오류를 방지하고 상태를 추적하기 용이하게 한다
        '''
        self.latest_image = None # 최신 이미지: fusion 함수에서 이미지 위에 LiDAR 포인트를 그릴 때 사용
        self.camera_matrix = None # 카메라 행렬: fusion 함수에서 3D 점을 2D 이미지 평면에 투영할 때 사용
        self.dist_coeffs = None # 왜곡 계수: fusion 함수에서 3D 점을 2D 이미지 평면에 투영할 때 사용
        self.latest_scan = None # 최신 LiDAR 스캔: fusion 함수에서 LiDAR 데이터를 처리할 때 사용

    def image_raw_callback(self, msg):
        '''
        1.  최신 이미지를 OpenCV 형식으로 변환하고 저장
        2. fusion 함수 호출
        '''
        self.latest_image = self.bridge.imgmsg_to_cv2(img_msg=msg, desired_encoding="bgr8")
        self.fusion()

    def camera_info_callback(self, msg):
        '''
        카메라 행렬과 왜곡 계수 저장 
        k: [453.1207580566406, 0.0, 328.5452880859375, 0.0, 453.1207580566406, 246.2456817626953, 0.0, 0.0, 1.0]
        d: [0.04100747033953667, -0.06024586781859398, 0.0004854919097851962, 8.69563446030952e-05, 0.0, 0.0, 0.0, 0.0]
        '''
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)

    def laser_scan_callback(self, msg):
        '''
        1. 최신 LiDAR 스캔을 저장
        2. fusion 함수 호출
        '''
        self.latest_scan = msg
        self.fusion()


    def fusion(self):
        # 필요한 모든 데이터가 준비되었는지 확인
        if self.latest_scan is None or self.latest_image is None or self.camera_matrix is None:
            return
        
        # LiDAR 프레임에서 카메라 프레임으로의 변환을 찾는다
        # 실패하면 경고를 로그하고 함수를 종료한다
        try:
            trans = self.tf_buffer.lookup_transform(target_frame='camera_color_optical_frame', source_frame='front_base_scan', time=rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().warn(message='TF lookup failed')
            return

        # 원본 이미지를 수정하지 않기 위해 복사본 생성
        image = self.latest_image.copy()
        
        # LiDAR 스캔의 각 포인트에 대해 반복
        # 유효한 범위 내의 값만 처리
        for i, range in enumerate(self.latest_scan.ranges):
            if range < self.latest_scan.range_min or range > self.latest_scan.range_max:
                continue
            
            # 극좌표계(거리, 각도)의 LiDAR 데이터를 3D 카테시안 좌표계로 변환
            angle = self.latest_scan.angle_min + i * self.latest_scan.angle_increment
            x = range * np.cos(angle)
            y = range * np.sin(angle)
            z = np.float_(0)  # 2D LiDAR이기 때문에 높이 값은 0
            
            # ROS2의 PointStamped 메시지를 활용하여 3D 점을 표현
            point = PointStamped()
            point.point.x = x
            point.point.y = y
            point.point.z = z
            point.header.frame_id = 'front_base_scan'
            point.header.stamp = self.get_clock().now().to_msg()
            
            # LiDAR 좌표계의 점을 카메라 좌표계로 변환
            point_transformed = tf2_geometry_msgs.do_transform_point(point=point, transform=trans)
            
            # 3D 점을 2D 이미지 평면에 투영
            point_camera_frame = np.array([point_transformed.point.x, point_transformed.point.y, point_transformed.point.z])
            pixel = cv2.projectPoints(objectPoints=point_camera_frame.reshape(-1, 3), rvec=np.zeros(3), tvec=np.zeros(3), cameraMatrix=self.camera_matrix, distCoeffs=self.dist_coeffs)[0].reshape(-1, 2)
            
            '''
            투영된 점이 이미지 범위 내에 있으면 해당 위치에 작은 원을 그린다
            * pixel[0][0]: pixel의 x좌표
            * pixel[0][1]: pixel의 y좌표
            * image.shape[0]: 이미지의 높이
            * image.shape[1]: 이미지의 너비
            '''
            if 0 <= pixel[0][0] < image.shape[1] and 0 <= pixel[0][1] < image.shape[0]:
                cv2.circle(img=image, center=(int(pixel[0][0]), int(pixel[0][1])), radius=1, color=(212, 255, 127), thickness=-1, lineType=cv2.LINE_8)
        
        # Fusion Image를 ROS2 메시지로 변환하고 발행
        fusion_image_msg = self.bridge.cv2_to_imgmsg(cvim=image, encoding="bgr8")
        self.pub_fusion_image.publish(msg=fusion_image_msg)


def main(args=None):
    '''
    * ROS2 초기화
    * SensorFusion 노드를 생성하여 실행
    * 프로그램이 종료되면 노드를 정리하고 ROS2 종료
    '''
    rclpy.init(args=args)
    sensor_fusion = SensorFusion()
    rclpy.spin(node=sensor_fusion)
    sensor_fusion.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()