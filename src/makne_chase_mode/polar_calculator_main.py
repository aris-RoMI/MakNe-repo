import numpy as np
import math
import cv2
import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point, Pose2D, PointStamped
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import tf2_ros
from tf2_geometry_msgs import do_transform_point


class PolarCalculator(Node):
    # 상수 정의
    NODE_NAME = 'polar_calculator_main'
    SUB_DEPTH_IMAGE_TOPIC = '/converted_depth_image/compressed'
    SUB_CENTER_POINT_TOPIC = '/id1_center_point'
    PUB_POLAR_COORDINATES_TOPIC = '/polar_coordinates'
    PUB_CAMERA_MARKER_TOPIC = '/target_camera_marker'
    PUB_LIDAR_MARKER_TOPIC = '/target_lidar_marker'
    QOS_PROFILE = 10
    CAMERA_FRAME = 'camera_color_optical_frame'
    LIDAR_FRAME = 'front_base_scan'
    TF_TIMEOUT = 1.0  # seconds
    CALCULATION_INTERVAL = 0.2

    # Camera intrinsic parameters
    FX = 478.6218566894531 * 0.5
    FY = 478.6218566894531 * 0.5
    CX = 323.0481262207031 * 0.5
    CY = 201.7069854736328 * 0.8
    print(FX, FY, CX, CY)

    def __init__(self):
        super().__init__(node_name=self.NODE_NAME)
        
        # 구독자
        self.sub_point = self.create_subscription(msg_type=Point, topic=self.SUB_CENTER_POINT_TOPIC, callback=self.point_callback, qos_profile=self.QOS_PROFILE)
        self.sub_depth = self.create_subscription(msg_type=CompressedImage, topic=self.SUB_DEPTH_IMAGE_TOPIC, callback=self.depth_callback, qos_profile=self.QOS_PROFILE)
        
        # 발행자
        self.pub_pose = self.create_publisher(msg_type=Pose2D, topic=self.PUB_POLAR_COORDINATES_TOPIC, qos_profile=self.QOS_PROFILE)
        self.pub_camera_marker = self.create_publisher(msg_type=Marker, topic=self.PUB_CAMERA_MARKER_TOPIC, qos_profile=self.QOS_PROFILE)
        self.pub_lidar_marker = self.create_publisher(msg_type=Marker, topic=self.PUB_LIDAR_MARKER_TOPIC, qos_profile=self.QOS_PROFILE)
         
        # TF2 버퍼와 리스너
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(buffer=self.tf_buffer, node=self)
        
        # OpenCV 브릿지
        self.bridge = CvBridge()
        
        # 상태 변수
        self.latest_point = None
        self.latest_depth = None
        self.last_depth = None
        self.latest_r = None
        self.latest_theta = None
        self.out_of_sight = False

        # 타이머 설정
        self.calculate_timer = self.create_timer(timer_period_sec=self.CALCULATION_INTERVAL, callback=self.process_data)
        
        self.get_logger().info('좌표 변환기 노드 초기화 완료')

    def point_callback(self, msg):
        try:
            self.latest_point = msg
            # self.get_logger().info(f'중심점 수신: ({msg.x}, {msg.y}, {msg.z})')
        except Exception as e:
            self.get_logger().error(message=f'중심점 수신 오류: {str(e)}')

    def depth_callback(self, msg):
        try:
            # 압축된 이미지를 NumPy 배열로 변환 후 디코딩
            np_arr = np.frombuffer(buffer=msg.data, dtype=np.uint8)
            self.latest_depth = cv2.imdecode(buf=np_arr, flags=cv2.IMREAD_ANYDEPTH)
        except Exception as e:
            self.get_logger().error(message=f'깊이 이미지 디코딩 오류: {str(e)}')

    def process_data(self):
        if self.latest_point is None or self.latest_depth is None:
            self.get_logger().error(message='중심점 or 깊이 이미지 수신 오류')
            return
        
        try:
            # 사람이 시야에서 사라졌을 때
            if math.isinf(self.latest_point.x):
                if not self.out_of_sight and self.latest_r is not None and self.latest_theta is not None:
                    self.publish_polar_coordinates(self.latest_r, self.latest_theta)
                    self.get_logger().info(f'사람이 시야에서 사라져 마지막 좌표 발행')
                    self.out_of_sight=True
                return
            
            # 사람이 다시 시야에 들어오면 변수 재설정
            self.out_of_sight = False

            # 깊이 값 추출
            depth_image = self.latest_depth
            height, width = depth_image.shape[:2]
            x = min(max(0, int(self.latest_point.x)), width - 1) # (0부터 319까지 320개)
            y = min(max(0, int(self.latest_point.y)), height - 1) # (0부터 319까지 320개)
            depth = float(depth_image[y, x]) / 1000.0 # mm에서 m로
            self.get_logger().info(f'추출된 깊이 값: {depth}m')

            # 깊이 값이 0인 경우 이전 깊이 값을 사용하여 예외 처리
            if depth == 0:
                if self.last_depth is not None:
                    depth = self.last_depth
                    # self.get_logger().warn(message=f'0이 아닌 마지막 깊이 값 사용: {depth}m')
                else:
                    raise ValueError('마지막 깊이 값이 없습니다.')
            else:
                self.last_depth=depth

            if not np.isfinite(depth):
                raise ValueError("깊이 값이 유효하지 않습니다")
            
            # 카메라 프레임에서의 3D 좌표 계산
            x = float((self.latest_point.x - self.CX) * depth / self.FX)
            y = float((self.latest_point.y - self.CY) * depth / self.FY)
            z = float(depth)
            # self.get_logger().info(f'카메라 프레임 3D 좌표: ({x}m, {y}m, {z}m)')
            
            # 카메라 프레임 기준의 3D 좌표를 PointStamped 메시지에 저장
            camera_point = PointStamped()
            camera_point.header.frame_id = self.CAMERA_FRAME
            camera_point.header.stamp = self.get_clock().now().to_msg()
            camera_point.point.x = x
            camera_point.point.y = y
            camera_point.point.z = z

            # 카메라 프레임 마커 발행
            self.publish_target_camera_marker(camera_point)
            
            # 두 프레임 간의 변환을 위해 변환 행렬 찾기
            lookup_transform = self.tf_buffer.lookup_transform(target_frame=self.LIDAR_FRAME, 
                                                               source_frame=self.CAMERA_FRAME, 
                                                               time=rp.time.Time().to_msg(), 
                                                               timeout=rp.duration.Duration(seconds=self.TF_TIMEOUT))
            
            # 실제 점 변환 수행
            lidar_point = do_transform_point(point=camera_point, transform=lookup_transform)
            self.publish_target_lidar_marker(lidar_point)
            # self.get_logger().info(f'라이다 프레임 3D 좌표: {lidar_point.point.x}m, {lidar_point.point.y}m, {lidar_point.point.z}m')
            
            # 극좌표계로 변환
            r = math.sqrt(lidar_point.point.x**2 + lidar_point.point.y**2)
            theta = math.atan2(lidar_point.point.y, lidar_point.point.x)
            
            # 최신 값 업데이트 및 발행
            self.latest_r = r
            self.latest_theta = theta
            self.publish_polar_coordinates(r, theta)
            

        except (IndexError, ValueError) as e:
            self.get_logger().error(f'깊이 값 추출 오류: {str(e)}')
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'TF2 오류: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'알 수 없는 오류: {str(e)}')

    def publish_polar_coordinates(self, r, theta):
        pose_2d = Pose2D()
        pose_2d.x = r
        pose_2d.theta = theta
        self.pub_pose.publish(pose_2d)
        self.get_logger().info(message=f'''
[극 좌표]
r: {r}m
theta: {theta}rad
''')
    
    def publish_target_camera_marker(self, point):
        marker = Marker()
        marker.header.frame_id = self.CAMERA_FRAME
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "camera_point"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = point.point.x
        marker.pose.position.y = point.point.y
        marker.pose.position.z = point.point.z
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.05 # 5cm 크기의 구체
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.pub_camera_marker.publish(marker)

    def publish_target_lidar_marker(self, point):
        marker = Marker()
        marker.header.frame_id = self.LIDAR_FRAME
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "lidar_point"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = point.point.x
        marker.pose.position.y = point.point.y
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.2 # 10cm 크기의 구체
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.pub_lidar_marker.publish(marker)



def main(args=None):
    rp.init(args=args)
    calculator = PolarCalculator()
    rp.spin(calculator)
    calculator.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()
