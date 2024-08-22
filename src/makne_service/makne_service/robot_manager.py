import threading
import rclpy
from rclpy.node import Node
from makne_service.waypoint_calculator import WayPointCalculator
from makne_service.dummy_action_client import SendGoal
from makne_msgs.srv import SetPointList, GetStatus, BoolSignal
from library.Constants import CommandConstants, DBConstants, RobotStatus

class RobotManager(Node):
    def __init__(self):
        super().__init__('robot_manager')
        # SetPointList 서비스 생성
        self.point_server = self.create_service(SetPointList, '/set_pointlist', self.set_waypoint_callback)
        self.status_server = self.create_service(GetStatus, "/get_status", self.get_status_callback)
        self.auto_timer_server = self.create_service(BoolSignal, "/auto_timer_signal", self.auto_timer_callback)
        self.complete_task_client = self.create_client(BoolSignal, "/complete_task_signal")
        self.auto_return_client = self.create_client(BoolSignal, "/auto_return_signal")
        
        # WayPointCalculator 초기화
        self.waypoint_calculator = WayPointCalculator(DBConstants.DB_NAME)
        
        # AutoReturnTimer 초기화
        self.auto_return_timer = None
        
        # 로봇의 상태 초기화
        self.robot_state = RobotStatus.STATUS_STANDBY
        self.current_user = ""
        self.remain_time = "0"
        
        # 경로 리스트 초기화
        self.optimal_waypoint = []
        self.return_location = self.waypoint_calculator.get_point_from_location_name(DBConstants.STANDBY)

        # SendGoal 인스턴스 생성 및 스레드 시작
        self.send_goal = SendGoal(self)
        self.send_goal.start()
        
    def start_auto_return_timer(self):
        # 30초 타이머 후 자동 복귀 함수
        self.auto_return_timer = threading.Timer(30.0, self.auto_return_to_base)
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
        

    def set_waypoint_callback(self, request, response):
        # 아직 로봇에게 명령을 내린 이용자가 없거나 같은 이용자가 계속 이용할 경우
        if self.current_user == "" or self.current_user == request.user_name:
            self.cancel_auto_return_timer()
            if request.command_type == CommandConstants.CANCEL:
                if self.robot_state == CommandConstants.CANCEL:
                    response.success = False
                    response.message = "Cancel error! There is nothing to cancel."
                    
                else:
                    standby_point = self.return_location
                    self.send_goal.send_goal(standby_point)  # 대기 위치로 이동
                    self.current_user = ""
                    self.robot_state = RobotStatus.STATUS_RETURN
                    self.optimal_waypoint = []
                    
                    response.success = True
                    response.message = "Request received! The robot is starting to return to the standby position!"
                
            elif request.command_type == CommandConstants.SEND_ROBOT:
                self.optimal_waypoint = self.waypoint_calculator.calculate_optimal_route(request.point_list)
                self.get_logger().info(f"Optimal route calculated with {len(self.optimal_waypoint)} waypoints")

                # 최적 경로를 순차적으로 처리 시작
                next_point = self.optimal_waypoint.pop(0)
                self.send_goal.send_goal(next_point)
                
                self.current_user = request.user_name
                self.robot_state = RobotStatus.STATUS_SEND
                response.success = True
                response.message = "Request received! The robot is on task!"
                
            elif request.command_type == CommandConstants.CALL_ROBOT:
                self.optimal_waypoint = request.point_list
                next_point = self.optimal_waypoint.pop(0)
                self.send_goal.send_goal(next_point)
                
                self.current_user = request.user_name
                self.robot_state = RobotStatus.STATUS_CALL
                response.success = True
                response.message = "Request received! The robot is on task!"
                
            elif request.command_type == CommandConstants.FOLLOW:
                self.current_user = request.user_name
                self.robot_state = RobotStatus.STATUS_FOLLOW
                response.success = True
                response.message = "Request received! The robot is on task!"
                
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
        
        return response
    
    # 목표 지점에 도달했을 때 호출되는 콜백
    def goal_completed_callback(self):
        # 복귀 명령이 완료된 경우
        if self.robot_state == RobotStatus.STATUS_RETURN:
            
            self.get_logger().info("Task cancelled, robot is now at standby location.")
            self.robot_state = RobotStatus.STATUS_STANDBY  # 로봇 상태를 STANDBY로 전환
            self.remain_time = "0"
            
        elif self.robot_state in [RobotStatus.STATUS_SEND, RobotStatus.STATUS_CALL]:
            # 다음 목표로 이동하거나, 모든 목표 완료 후 복귀 처리
            if len(self.optimal_waypoint) > 0:
                next_goal = self.optimal_waypoint.pop(0)
                self.get_logger().info(f"Moving to next goal: {next_goal}")
                self.send_goal.send_goal(next_goal)
            else:
                request = BoolSignal.Request()
                request.complete_signal = True
                
                future = self.complete_task_client.call_async(request)
                future.add_done_callback(self.handle_complete_task_response)
                
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

# class SendGoal(Thread):
#     def __init__(self, node: Node):
#         super().__init__()
#         self.node = node
#         self._action_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')

#     def run(self):
#         # 스레드 실행: rclpy 스핀을 돌려서 ROS 2 이벤트를 처리함
#         while rclpy.ok():
#             rclpy.spin_once(self.node, timeout_sec=0.1)

#     def send_goal(self, waypoint):
#         # 주어진 목표를 처리
#         self.node.get_logger().info(f"Sending goal to: {waypoint}")
        
#         # 실제 goal_msg 생성
#         goal_msg = NavigateToPose.Goal()
#         goal_msg.pose = self.create_pose_stamped(waypoint)

#         self._action_client.wait_for_server()
#         send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
#         send_goal_future.add_done_callback(self.goal_response_callback)

#     def create_pose_stamped(self, waypoint):
#         pose = PoseStamped()
#         pose.header.frame_id = "map"
#         pose.header.stamp = self.node.get_clock().now().to_msg()
#         pose.pose.position.x = waypoint.x
#         pose.pose.position.y = waypoint.y
#         pose.pose.orientation.w = 1.0  # 적절한 방향으로 설정 필요
#         return pose

#     def feedback_callback(self, feedback_msg):
#         feedback = feedback_msg.feedback
#         self.node.get_logger().info(f'Current robot position: {feedback.current_pose}')

#     def goal_response_callback(self, future):
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.node.get_logger().info('Goal rejected')
#             return

#         self.node.get_logger().info('Goal accepted')
#         result_future = goal_handle.get_result_async()
#         result_future.add_done_callback(self.get_result_callback)

#     def get_result_callback(self, future):
#         result = future.result().result
#         self.node.get_logger().info(f'Navigation result: {result}')
#         # 목표가 완료되면 RobotManager에게 알림
#         self.node.goal_completed_callback()


    
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
