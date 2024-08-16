import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose  # turtlesim의 위치 메시지
import math
from std_msgs.msg import Bool

class VelocityControl(Node):
    def __init__(self):
        super().__init__('velocity_control')

        self.cmd_vel_publisher = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)

        self.target_subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.target_callback,
            10
        )
        self.target_subscription = self.create_subscription(
            Pose,
            '/turtle2/pose',
            self.me_callback,
            10
        )
        self.obstacle_flag_subscription = self.create_subscription(
            Bool,  # 장애물 유무를 나타내는 메시지 타입
            '/obstacle',  # 장애물 감지 토픽 이름
            self.obstacle_callback,
            10
        )
        
        self.obstacle_detected = False
        self.start = False
        self.prev_time = self.get_clock().now() # 이전 시간

        # 기본 파라미터
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_angle = 0.0
        self.me_x = 0.0
        self.me_y = 0.0
        self.me_angle = 0.0
        # self.target_vx = 0.0
        # self.target_vy = 0.0
        self.target_distance_threshold = 1.0  # 타겟과 유지할 거리 (이 거리를 넘어서면 출발)
        self.stop_threshold = 1.2 # 타겟과의 거리가 이 기준 이하로 떨어질 경우 속도 0.
        self.prev_vel = 0.0  # 이전 로봇 속도 값
        self.prev_distance = 0.0 # 이전 거리 값
        
        
        # 제어 파라미터
        self.lambda_parameter = 0.6  # parameter (0.4) 
        self.time_gap_parameter = 1.5 # Time-Gap Parameter (1.8)
        # self.linear_kp = 0.5  # 선형 속도 제어를 위한 비례 계수
        self.angular_kp = 2.5  # 각속도 제어를 위한 비례 계수

        self.get_logger().info('VelocityControl node initialized')

    def obstacle_callback(self, msg):
        self.obstacle_detected = msg.data  # 장애물 감지 여부 업데이트
        if self.obstacle_detected:
            self.zero_vel()  # 장애물 감지 시 즉시 멈춤
            self.get_logger().info('Obstacle detected, stopping.')
    
    def me_callback(self,msg):
        self.me_x = msg.x
        self.me_y = msg.y
        self.me_angle = msg.theta

    def target_callback(self, msg):
        if self.obstacle_detected:  # 장애물이 있으면 아무 동작도 하지 않음
            return
        
        current_time = self.get_clock().now()  # 현재 시간 기록
        delta_t = (current_time - self.prev_time).nanoseconds / 1e9  # 초 단위로 변환
        self.prev_time = current_time  # 현재 시간을 다음 루프의 이전 시간으로 저장

        self.target_x = msg.x
        self.target_y = msg.y
        self.target_angle = msg.theta
        self.velocity_control(delta_t)
    
    def velocity_control(self,delta_t):
        if self.obstacle_detected:  # 장애물이 있으면 아무 동작도 하지 않음
            return

        # self.get_logger().info('Executing velocity control')
        
        # turtle2의 현재 위치가 (0, 0)이고, turtle1의 위치로 향한다고 가정합니다.
        distance = math.sqrt((self.target_x-self.me_x)**2 + (self.target_y-self.me_y)**2)
        self.get_logger().info(f'Distance : {distance}')
        if (not self.start) and distance > self.target_distance_threshold :
            self.zero_vel()
            self.start = True
        
        if self.start :
            if distance < self.stop_threshold:
                velocity_msg = Twist()
                self.zero_vel()
                self.get_logger().info('Target reached, stopping.')
                self.start = False
                return

            accel = self.acceleration(distance,delta_t)
            a = math.atan2(self.target_y-self.me_y,self.target_x-self.me_x)
            if a < 0 :
                a = 2*math.pi + a
            rel_angle = a - self.me_angle
            if rel_angle < -math.pi :
                rel_angle += 2*math.pi
            elif rel_angle > math.pi :
                rel_angle -= 2*math.pi
            # move_angle = math.atan2(self.target_y-self.me_y,self.target_x-self.me_x) - self.me_angle

            linear_velocity = self.prev_vel + accel*delta_t
            angular_velocity = self.angular_kp * rel_angle
            
            velocity_msg = Twist()
            velocity_msg.linear.x = linear_velocity
            velocity_msg.angular.z = angular_velocity

            self.cmd_vel_publisher.publish(velocity_msg)
            self.prev_vel = linear_velocity ## update prev_vel
            self.get_logger().info(f'Moving towards target: linear_speed={linear_velocity:.2f}, \
                                   angular_speed={angular_velocity:.2f}')
        else :
            self.get_logger().info('Not Start.......')
    
    def acceleration(self,distance,delta_t):
        try :
            target_velocity = (distance-self.prev_distance)/delta_t
            delta = distance-self.time_gap_parameter*self.prev_vel
            accel = (target_velocity+self.lambda_parameter*delta)/(self.time_gap_parameter)
            self.get_logger().info(f'Acceleration : {accel}')
            self.prev_distance = distance  ## update prev_distance
            return accel
        
        except Exception as e :
            self.get_logger().info('Acceleration Calculation Fail!!!')
    
    def zero_vel(self) :    # 모든 속도를 0으로 설정
        velocity_msg = Twist()
        velocity_msg.linear.x = 0.0; velocity_msg.linear.y = 0.0; velocity_msg.linear.z = 0.0
        velocity_msg.angular.x = 0.0; velocity_msg.angular.y = 0.0; velocity_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(velocity_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VelocityControl()
    rclpy.spin(node)
    node.get_logger().info('Shutting down VelocityControl node')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
