import rclpy as rp
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D, Twist
import numpy as np
import matplotlib.pyplot as plt
from euler_from_quaternion import transformations
from enum import Enum
import time
from collections import deque

class RobotState(Enum):
    IDLE = 0
    FOLLOWING = 1
    SEARCHING = 2

class VFHNavigator(Node):
    # Constants
    HISTOGRAM_RESOLUTION = 1  # 각도
    NUM_SECTORS = int(200 / HISTOGRAM_RESOLUTION)
    THRESHOLD = 0.1  # 장애물 감지 임계값
    WINDOW_SIZE = 40  # 평활화 윈도우 크기
    
    MAX_LINEAR_SPEED = 0.1  # m/s
    MIN_LINEAR_SPEED = 0.05 
    FAST_SPEED = 0.15
    current_speed = 0.0
    MAX_ANGULAR_SPEED = 0.3  # rad/s
    SAFE_DISTANCE_MIN = 0.9  # m
    SAFE_DISTANCE_MAX = 1.2   # m
    GOAL_WEIGHT = 0.6
    OBSTACLE_WEIGHT = 1 - GOAL_WEIGHT
    
    SEARCH_ANGLE = -np.pi/3 # 60도
    SEARCH_DURATION = 1.0  # 2초
    GOAL_TIMEOUT = 7.0  # 5초

    def __init__(self):
        super().__init__('vfh_navigator')
        
        # 구독자
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(Pose2D, '/polar_coordinates', self.goal_callback, 10)
        
        # 발행자
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 모터 제어 발행 주기
        self.cmd_vel = Twist()
        self.timer = self.create_timer(timer_period_sec=0.2, callback=self.timer_callback)

        # VFH 파라미터
        self.histogram = np.zeros(self.NUM_SECTORS)
        
        # 로봇 상태
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        
        # 목표 상태
        self.goal_r = 0.0
        self.goal_r_list = deque(maxlen=6)
        self.goal_theta = 0.0
        self.reached = False  # 사람에게 도달했는지 여부를 나타내는 flag
        self.is_searching = False # 탐색모드가 실행중인지 나타내는 flag
        self.is_reaching = False 
        self.count_goal = 0
        self.front_range = 0.0
        self.front_range_1 = 0.0
        self.min_front = 0.0


        # 새로운 상태 변수
        self.robot_state = RobotState.IDLE
        self.last_goal_time = self.get_clock().now()
        self.search_finish_time = None
        self.search_start_time = None
        
        # 시각화
        # plt.ion()
        # self.fig, self.ax = plt.subplots()
        # self.bar_plot = self.ax.bar(range(self.NUM_SECTORS), self.histogram)
        # self.direction_line, = self.ax.plot([0, 0], [0, 1], color='r', linestyle='--', linewidth=8)
        # self.ax.set_xlim(0, self.NUM_SECTORS)
        # self.ax.set_ylim(0, 1)
        # self.ax.set_title('VFH Histogram')
        # self.ax.set_xlabel('Angle (sectors)')
        # self.ax.set_ylabel('Obstacle Density')

    def laser_callback(self, msg):
        
        if self.is_searching :
            return
        
        if self.robot_state == RobotState.SEARCHING and self.search_finish_time is not None :
            if (self.get_clock().now()-self.search_finish_time).nanoseconds / 1e9 < 1.0 :
                return
        
        self.laser_ranges = np.array(msg.ranges)
        self.laser_angle_min = msg.angle_min
        self.laser_angle_increment = msg.angle_increment
    
        i=0
        sum_front_range=0
        valid_ranges = []
        
        while i<80 :
            range_value = msg.ranges[260+i]
            if range_value > 0.1 :
                sum_front_range += range_value
                valid_ranges.append(range_value)
            i+=1
        
        if valid_ranges:
            self.min_front = min(valid_ranges)
        else:
            self.min_front = float('inf')  

        self.front_range = sum_front_range / len(valid_ranges) if valid_ranges else float('inf')

        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        ranges = np.array(msg.ranges)
        
        self.histogram = np.zeros(self.NUM_SECTORS)
        for angle, range in zip(angles, ranges):
            if msg.range_min <= range <= msg.range_max:
                sector = int((angle - msg.angle_min) / (msg.angle_max - msg.angle_min) * self.NUM_SECTORS)
                if 0 <= sector < self.NUM_SECTORS:
                    self.histogram[sector] += 1 / (range ** 2)
        
        if np.max(self.histogram) > 0:
            self.histogram /= np.max(self.histogram)
        
        kernel = np.ones(self.WINDOW_SIZE) / self.WINDOW_SIZE
        self.histogram = np.convolve(self.histogram, kernel, mode='same')
        
        # self.update_visualization()
        
        if self.is_reaching :
            return
        
        self.navigate()
        

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, self.robot_yaw = transformations.euler_from_quaternion(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)

    def goal_callback(self, msg):
        self.goal_r = msg.x
        self.goal_r_list.append(msg.x)
        self.goal_theta = msg.theta
        self.last_goal_time = self.get_clock().now()

        if not self.reached  :
            if self.robot_state == RobotState.IDLE:
                self.robot_state = RobotState.FOLLOWING
                self.get_logger().info('목표 감지. FOLLOWING 상태로 전환')
            elif self.robot_state == RobotState.SEARCHING:
                self.get_logger().info('Searching Mode Goal Callback flag')
                self.robot_state = RobotState.FOLLOWING
                self.get_logger().info('목표 재감지. FOLLOWING 상태로 전환')
        else :
            if self.front_range > 0.58 and self.goal_r > 0.9 :
                self.count_goal += 1
            else :
                self.stop_robot() ### reached
                self.count_goal = 0

        if self.count_goal == 20 :
            self.reached = False
            self.count_goal = 0 
    
    def navigate(self):
        if self.robot_state == RobotState.IDLE:
            self.get_logger().info('IDLE')
            self.stop_robot()
            return

        if self.robot_state == RobotState.SEARCHING:
            if not self.is_searching :
                self.is_searching = True
                self.perform_search()
                self.get_logger().info('Searching 종료 후 잠시 대기')
            return

        if self.check_goal_timeout():
            self.start_searching()
            return
        
        if self.is_slow() :
            self.get_logger().info(f'안전 거리 도달 (r: {self.goal_r:.2f}m). Lidar : {self.min_front}')
            self.is_reaching = True 
            if self.reaching() :
                self.reached = True 
                self.stop_robot()
                self.get_logger().info('목표에 도달했습니다')
                self.robot_state = RobotState.IDLE
                return

        goal_sector = int((self.goal_theta - self.robot_yaw) / (2 * np.pi) * self.NUM_SECTORS) % self.NUM_SECTORS
        valleys = self.find_valleys()
        
        if not valleys:
            best_sector = goal_sector
        else:
            valley_scores = []
            for valley_center, valley_density in valleys:
                goal_score = 1 - abs(valley_center - goal_sector) / self.NUM_SECTORS
                obstacle_score = 1 - valley_density
                total_score = self.GOAL_WEIGHT * goal_score + self.OBSTACLE_WEIGHT * obstacle_score
                valley_scores.append((valley_center, total_score))
            
            best_sector, _ = max(valley_scores, key=lambda x: x[1])

        desired_angle = (best_sector / self.NUM_SECTORS) * 2 * np.pi + self.robot_yaw
        angle_diff = np.arctan2(np.sin(desired_angle - self.robot_yaw), np.cos(desired_angle - self.robot_yaw))
                
        
        if np.abs(self.goal_theta) < 0.2 and self.goal_r <= 2 :  
            self.get_logger().info('사람이 정면에 위치. 장애물이 있는지 확인중....')
            self.cmd_vel.linear.x = self.MAX_LINEAR_SPEED
            self.cmd_vel.angular.z = 0.0
            if self.target_straight() :
                self.get_logger().info('장애물이 없습니다. 직진합니다.....')
                self.cmd_vel.linear.x = self.MAX_LINEAR_SPEED
                self.cmd_vel.angular.z = 0.0
            else :
                self.get_logger().info('전방에 장애물')
                angular_vel = min(max(-self.MAX_ANGULAR_SPEED, angle_diff), self.MAX_ANGULAR_SPEED)
                self.cmd_vel.angular.z = angular_vel
        
        elif self.goal_r > 2.0 and self.min_front > 1.2 and np.abs(self.goal_theta) < 0.25 :
            if self.is_straight() :
               self.get_logger().info('전방에 장애물이 멀리 떨어져 있습니다')
               self.cmd_vel.linear.x = self.current_speed
            angular_vel = 0.0
            self.cmd_vel.angular.z = angular_vel
            
        elif self.min_front < 0.6 : 
            self.get_logger().info(f'전방에 장애물이 있어 일단 정지합니다 Lidar(front range) : {self.min_front}')
            linear_vel = 0.0 
            angular_vel = min(max(-self.MAX_ANGULAR_SPEED, angle_diff), self.MAX_ANGULAR_SPEED)
            self.cmd_vel.linear.x = linear_vel
            self.cmd_vel.angular.z = angular_vel

        else :
            linear_vel = self.MAX_LINEAR_SPEED
            angular_vel = min(max(-self.MAX_ANGULAR_SPEED, angle_diff), self.MAX_ANGULAR_SPEED)
            # 직접 발행 금지
            self.cmd_vel.linear.x = linear_vel
            self.cmd_vel.angular.z = angular_vel
        
        
        # self.direction_line.set_xdata([best_sector, best_sector])

    def is_target_reached(self):
        self.get_logger().info('reaching to target......')
        current_time = self.get_clock().now()
        i=0 
        while (self.get_clock().now()-current_time).nanoseconds / 1e9  < 0.5 :
            if self.min_front < self.SAFE_DISTANCE_MIN :
                i+=1
            if i == 3 :
                self.get_logger().info('Target Front is True')
                return 1
        
        return 0


    def is_slow(self) :
        if (self.goal_r <= 1.2) and (self.min_front <= self.SAFE_DISTANCE_MAX) and np.abs(self.goal_theta) < 0.3 :
            return 1
        else :
            return 0
    
    
    def reaching(self) :
        self.cmd_vel.linear.x = 0.05
        self.cmd_vel.angular.z = 0.0
        self.get_logger().info('Reaching.....')
        if self.is_target_reached() :
            self.get_logger().info('정면에 사람이 있습니다...')
            self.is_reaching = False
            return 1
        else : 
            self.get_logger().info('정면에 사람이 없습니다...')
            self.is_reaching = False
            return 0

    def target_straight(self):
        if self.min_front > self.SAFE_DISTANCE_MAX :
            return 1
        else :
            return 0
        
    def is_straight(self):
        # 전방에 해당하는 섹터 범위 설정 
        front_sector_start = int(self.NUM_SECTORS * 0.45)
        front_sector_end = int(self.NUM_SECTORS * 0.55)
        
        # 전방 섹터의 히스토그램 값 평균 계산
        front_histogram_values = self.histogram[front_sector_start:front_sector_end + 1]
        avg_density = np.mean(front_histogram_values)
        
        # 최대 밀도 값 설정 
        MAX_DENSITY = 1.0
        MIN_DENSITY = 0.5
        MIN_SPEED = 0.1  
        MAX_SPEED = self.FAST_SPEED  # 최대 속도

        # 밀도 값을 기반으로 FAST_SPEED 설정 
        if avg_density > MIN_DENSITY:
            # 밀도 값이 높을수록 속도를 낮추도록 설정
            self.current_speed = MAX_SPEED * (1 - (avg_density / MAX_DENSITY))
            # 최소 속도를 보장
            self.current_speed = max(self.current_speed, MIN_SPEED)
        else:
            # 밀도 값이 낮을 경우 최대 속도로 설정
            self.current_speed = MAX_SPEED
        self.get_logger().info(f'밀도 값에 따라 속도 조정 중... current_speed : {self.current_speed}')
        # 전방 섹터에 장애물이 없는 경우 True 반환
        return avg_density < MIN_DENSITY
    
    # 타이머를 통해 속도 값 발행
    def timer_callback(self):
        self.pub_cmd_vel.publish(self.cmd_vel)

    def find_valleys(self):
        valleys = []
        in_valley = False
        valley_start = 0
        valley_sum = 0
        for i in range(self.NUM_SECTORS):
            if self.histogram[i] < self.THRESHOLD:
                if not in_valley:
                    valley_start = i
                    valley_sum = 0
                    in_valley = True
                valley_sum += self.histogram[i]
            elif in_valley:
                valley_end = i - 1
                valley_center = (valley_start + valley_end) // 2
                valley_length = valley_end - valley_start + 1
                valley_avg_density = valley_sum / valley_length
                valleys.append((valley_center, valley_avg_density))
                in_valley = False
        if in_valley:
            valley_end = self.NUM_SECTORS - 1
            valley_center = (valley_start + valley_end) // 2
            valley_length = valley_end - valley_start + 1
            valley_avg_density = valley_sum / valley_length
            valleys.append((valley_center, valley_avg_density))
        return valleys

    def stop_robot(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z= 0.0
        self.pub_cmd_vel.publish(cmd_vel)

    def update_visualization(self):
        for rect, h in zip(self.bar_plot, self.histogram):
            rect.set_height(h)
        self.ax.set_ylim(0, 1.1)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def check_goal_timeout(self):
        elapsed_time = (self.get_clock().now() - self.last_goal_time).nanoseconds / 1e9
        return elapsed_time > self.GOAL_TIMEOUT

    def start_searching(self):
        self.robot_state = RobotState.SEARCHING
        self.get_logger().info('목표 감지 실패. SEARCHING 상태로 전환')

    def perform_search(self):
        self.search_start_time = self.get_clock().now()
        self.stop_robot()
        cmd_vel = Twist()
        cmd_vel.angular.z = self.SEARCH_ANGLE / self.SEARCH_DURATION
        while (self.get_clock().now() - self.search_start_time).nanoseconds / 1e9 < self.SEARCH_DURATION :
            self.pub_cmd_vel.publish(cmd_vel)
            self.get_logger().info('rotate')
            time.sleep(0.1)
        self.stop_robot()
        self.get_logger().info('탐색 회전 완료.')
        self.get_logger().info('Searching 종료 후 잠시 대기')
        time.sleep(1.0)
        self.is_searching = False
        self.search_finish_time = self.get_clock().now()



    
def main(args=None):
    rp.init(args=args)
    navigator = VFHNavigator()
    rp.spin(navigator)
    navigator.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()