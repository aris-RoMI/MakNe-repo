import threading
import rclpy
from rclpy.node import Node
from makne_service.checkpoint_calculator import CheckPointCalculator
from makne_service.send_goal_client import SendGoal  # 수정된 SendGoal 클래스 import
from makne_msgs.srv import SetPointList, GetStatus, BoolSignal
from library.Constants import CommandConstants, DBConstants, RobotStatus
from std_msgs.msg import String
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose, Point
from nav_msgs.msg import Path
import math

class RobotManager(Node):
    def __init__(self):
        super().__init__('robot_manager')
        # SetPointList 서비스 생성
        self.point_server = self.create_service(SetPointList, '/set_pointlist', self.set_checkpoint_callback)
        self.status_server = self.create_service(GetStatus, "/get_status", self.get_status_callback)
        self.auto_timer_server = self.create_service(BoolSignal, "/auto_timer_signal", self.auto_timer_callback)
        self.complete_task_client = self.create_client(BoolSignal, "/complete_task_signal")
        self.auto_return_client = self.create_client(BoolSignal, "/auto_return_signal") 
        self.chase_client = self.create_client(SetBool, "/chase_signal")
        
        # 추가된 부분
        self.cancel_event = threading.Event()  # 취소 명령 대기 이벤트
        
        ##################################
        self.amcl_subscription = self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.pose_update_callback, 10)
        self.subscription = self.create_subscription(Path, "/plan", self.path_update_callback, 10)
        self.battery_subscription = self.create_subscription(String, "/battery_voltage", self.battery_update_callback, 10)
        self.robot_pose_x = 0
        self.robot_pose_y = 0
        
        self.robot_path_list = []
        
        self.remain_battery = float(100)
        
        # CheckPointCalculator 초기화
        self.checkpoint_calculator = CheckPointCalculator(DBConstants.DB_NAME)
        
        # AutoReturnTimer 초기화
        self.auto_return_timer = None
        
        # 로봇의 상태 초기화
        self.robot_state = RobotStatus.STATUS_STANDBY
        self.current_user = ""
        self.remain_time = "0"
        
        # 경로 리스트 초기화
        self.optimal_checkpoint = []
        self.return_location = self.checkpoint_calculator.get_point_from_location_name(DBConstants.STANDBY)

        # SendGoal 인스턴스 생성 및 스레드 시작
        self.send_goal = SendGoal(self)
        self.send_goal.start()
        
    def pose_update_callback(self, data):
        self.robot_pose_x = float(data.pose.pose.position.x)
        self.robot_pose_y = float(data.pose.pose.position.y)
        
    def path_update_callback(self, data):
        robot_path_list = []
        if len(data.poses) > 0:
            for pose in data.poses:
                x = float(pose.pose.position.x)
                y = float(pose.pose.position.y)
                robot_path_list.append((x, y))
                
        self.robot_path_list = robot_path_list
        
    def battery_update_callback(self, data):
        self.remain_battery = round(float(data.data), 2)

    def start_auto_return_timer(self, duration = 30.0):
        # 30초 타이머 후 자동 복귀 함수
        self.auto_return_timer = threading.Timer(duration, self.auto_return_to_base)
        self.auto_return_timer.start()
        
    def cancel_auto_return_timer(self):
        # 타이머 취소 함수
        if self.auto_return_timer is not None:
            self.auto_return_timer.cancel()
            self.get_logger().info("Auto-return timer cancelled.")
            self.auto_return_timer = None
        
    def auto_return_to_base(self):
        # 30초 후 자동 복귀 처리 함수
        self.get_logger().info("Auto-return triggered after 30 seconds.")
        standby_point = self.return_location
        self.current_user = ""
        self.send_goal.send_goal(standby_point)
        self.robot_state = RobotStatus.STATUS_RETURN
        request = BoolSignal.Request()
        request.complete_signal = True
        self.auto_return_client.call_async(request)
        self.cancel_auto_return_timer()
        
    def auto_timer_callback(self, request, response):
        self.cancel_auto_return_timer()
        self.start_auto_return_timer()
        
        response.success = True
        
        return response

    def set_checkpoint_callback(self, request, response):
        # 아직 로봇에게 명령을 내린 이용자가 없거나 같은 이용자가 계속 이용할 경우
        if self.current_user == "" or self.current_user == request.user_name:
            self.cancel_auto_return_timer()
            
            if request.command_type == CommandConstants.CANCEL:
                if self.robot_state == RobotStatus.STATUS_RETURN:
                    response.success = False
                    response.message = "Cancel error! There is nothing to cancel."
                    
                elif self.robot_state in [RobotStatus.STATUS_CALL, RobotStatus.STATUS_SEND]:
                    # 여기에서 cancel_event를 설정하여 대기 중인 스레드를 깨움
                    self.cancel_event.set()
                    if self.send_goal._current_goal_handle is not None:
                        self.send_goal._current_goal_handle.cancel_goal_async()
                        self.get_logger().info("Cancelling the current goal and returning to standby.")
                    
                    standby_point = self.return_location
                     # 대기 위치로 이동
                    self.current_user = ""
                    self.robot_state = RobotStatus.STATUS_RETURN
                    self.optimal_checkpoint = []
                    
                    # self.send_goal.send_goal(standby_point)  # 대기 위치로 이동
                    response.success = True
                    response.message = "Request received! The robot is starting to return to the standby position!"
                    self.get_logger().info("Cancelling send(call).")
                    
                elif self.robot_state == RobotStatus.STATUS_FOLLOW:
                    chase_request = SetBool.Request()
                    chase_request.data = False
                    future = self.chase_client.call_async(chase_request)
                    future.add_done_callback(self.handle_chase_response)
                    
                    self.current_user = ""
                    self.robot_state = RobotStatus.STATUS_RETURN
                    response.success = True
                    response.message = "Request received! The robot is starting to return to the standby position!"
                    self.get_logger().info("Cancelling Follow.")
                    
                    standby_point = self.return_location
                    # self.send_goal.send_goal(standby_point)  # 대기 위치로 이동
                    self.current_user = ""
                    self.robot_state = RobotStatus.STATUS_RETURN
                
            elif request.command_type == CommandConstants.SEND_ROBOT:
                starting_point = (float(self.robot_pose_x), float(self.robot_pose_y))
                self.optimal_checkpoint = self.checkpoint_calculator.calculate_optimal_route(request.point_list, starting_point)
                self.get_logger().info(f"Optimal route calculated with {len(self.optimal_checkpoint)} checkpoints")

                # 최적 경로를 순차적으로 처리 시작
                next_point = self.optimal_checkpoint.pop(0)
                self.send_goal.send_goal(next_point)
                
                self.current_user = request.user_name
                self.robot_state = RobotStatus.STATUS_SEND
                response.success = True
                response.message = "Request received! The robot is on task!"
                
            elif request.command_type == CommandConstants.CALL_ROBOT:
                self.optimal_checkpoint = request.point_list
                next_point = self.optimal_checkpoint.pop(0)
                goal_point = self.checkpoint_calculator.get_point_from_location_name(next_point)
                if next_point == "cafe":
                    self.send_goal.send_goal(goal_point, z = math.pi)
                else:
                    self.send_goal.send_goal(goal_point)
                
                self.current_user = request.user_name
                self.robot_state = RobotStatus.STATUS_CALL
                response.success = True
                response.message = "Request received! The robot is on task!"
                
            elif request.command_type == CommandConstants.FOLLOW:
                self.get_logger().info(f"Chase mode starting")
                
                # SetBool 서비스 요청을 생성
                chase_request = SetBool.Request()
                chase_request.data = True
                
                # 비동기로 서비스 호출
                future = self.chase_client.call_async(chase_request)
                
                # 응답 처리를 위한 콜백 함수 추가
                future.add_done_callback(self.handle_chase_response)
                
                # 로봇의 상태를 변경 (콜백에서 완료를 확인하지 않고 상태를 설정)
                self.current_user = request.user_name
                self.robot_state = RobotStatus.STATUS_FOLLOW
                
                # 임시로 성공 응답을 바로 설정 (실제 성공 여부는 콜백에서 확인)
                response.success = True
                response.message = "Request received! The robot is on task!"
                self.get_logger().info(f"Chase mode on")
                
            elif request.command_type == CommandConstants.RETURN:
                self.current_user = ""
                self.send_goal.send_goal(self.return_location)
                self.robot_state = RobotStatus.STATUS_RETURN
                response.success = True
                response.message = "Request received! The robot is on task!"
        
        # 로봇이 이용자 명령을 수행하고 있는 경우
        else:
            response.success = False
            response.message = "Command error! The robot is already on task. Try again later!"

        return response

    def get_status_callback(self, request, response):
        response.success = True
        response.current_user = self.current_user
        response.current_task = self.robot_state
        response.remain_time = self.remain_time
        
        # 현재 로봇의 위치 설정
        current_pose = Pose()
        current_pose.position.x = float(self.robot_pose_x)
        current_pose.position.y = float(self.robot_pose_y)
        current_pose.position.z = 0.0  # 기본값 설정
        current_pose.orientation.w = 1.0  # 기본 quaternion 값
        
        # 현재 경로 설정
        current_path = Path()
        current_path.header.frame_id = "map"  # 프레임 ID 설정
        current_path.header.stamp = self.get_clock().now().to_msg()  # 현재 시간으로 타임스탬프 설정
        
        # PoseStamped 메시지로 변환하여 Path에 추가
        pose_stamped_list = []
        for pose_position in self.robot_path_list:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            
            # Point 타입의 position 설정
            point = Point()
            point.x = pose_position[0]  # pose_position의 첫 번째 요소를 x로 설정
            point.y = pose_position[1]  # pose_position의 두 번째 요소를 y로 설정
            point.z = 0.0  # 기본값 설정
            
            pose_stamped.pose.position = point
            pose_stamped.pose.orientation.w = 1.0  # 기본 orientation 값 설정
            pose_stamped_list.append(pose_stamped)
        
        current_path.poses = pose_stamped_list  # 경로를 담고 있는 PoseStamped 리스트를 설정

        # response에 현재 위치와 경로를 담기
        response.current_pose = current_pose
        response.current_path = current_path
        
        # 배터리 잔량 표시
        response.remain_battery = self.remain_battery
        self.get_logger().info(f"Get Status")
        return response
    
    def goal_completed_callback(self):
        if self.robot_state == RobotStatus.STATUS_RETURN:
            self.get_logger().info("Task cancelled, robot is now at standby location.")
            self.robot_state = RobotStatus.STATUS_STANDBY  # 로봇 상태를 STANDBY로 전환
            self.remain_time = "0"
            
        elif self.robot_state in [RobotStatus.STATUS_SEND, RobotStatus.STATUS_CALL]:
            self.cancel_event.clear()  # 이벤트 플래그를 초기화
            self.get_logger().info("Waiting before proceeding to the next goal...")
            
            wait_duration = 5.0  # 대기 시간 설정 (초 단위)
            if self.cancel_event.wait(timeout=wait_duration):  # 대기 중 취소되었는지 확인
                self.get_logger().info("Task was cancelled during wait time.")
                self.send_goal.send_goal(self.return_location)
                return  # 대기 중에 취소되었으므로 작업을 중단
            
            # 다음 목표로 이동하거나, 모든 목표 완료 후 복귀 처리
            if len(self.optimal_checkpoint) > 0:
                next_goal = self.optimal_checkpoint.pop(0)
                self.get_logger().info(f"Moving to next goal: {next_goal}")
                self.send_goal.send_goal(next_goal)
            else:
                request = BoolSignal.Request()
                request.complete_signal = True
                
                future = self.complete_task_client.call_async(request)
                future.add_done_callback(self.handle_complete_task_response)
                
    def handle_chase_response(self, future):
        try:
            result = future.result()
            if result.success:
                self.get_logger().info("Chase mode successfully started.")
            else:
                self.get_logger().warn("Chase mode failed to start.")
        except Exception as e:
            self.get_logger().error(f"Chase mode service call failed with exception: {str(e)}")
                    
    def handle_complete_task_response(self, future):
        try:
            result = future.result()
            if result.success:
                self.robot_state = RobotStatus.STATUS_STANDBY  # 로봇 상태를 STANDBY로 전환
                self.start_auto_return_timer()
                self.get_logger().info("Task completed successfully. Starting auto-return timer.")
            else:
                self.get_logger().warn("Task completion service call failed.")
        except Exception as e:
            self.get_logger().error(f"Error in task completion callback: {str(e)}")
    
def main():
    rclpy.init()
    
    robot_manager = RobotManager()
    try:

        # ROS 2 이벤트 루프 실행
        rclpy.spin(robot_manager)
    except KeyboardInterrupt:
        rclpy.shutdown()
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
