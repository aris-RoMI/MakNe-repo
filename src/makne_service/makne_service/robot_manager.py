import rclpy
from rclpy.node import Node
from makne_service.waypoint_calculator import WayPointCalculator
from makne_service.dummy_action_client import SendGoal
from makne_msgs.srv import SetPointList, GetStatus
from library.Constants import CommandConstants, DBConstants, RobotStatus

class RobotManager(Node):
    def __init__(self):
        super().__init__('robot_manager')
        # SetPointList 서비스 생성
        self.point_server = self.create_service(SetPointList, '/set_pointlist', self.set_waypoint_callback)
        self.status_server = self.create_service(GetStatus, "/get_status", self.get_status_callback)
        
        # WayPointCalculator 초기화
        self.waypoint_calculator = WayPointCalculator(DBConstants.DB_NAME)
        
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

    def set_waypoint_callback(self, request, response):
        # 아직 로봇에게 명령을 내린 이용자가 없는 경우
        if self.current_user:
            if request.command_type == CommandConstants.CANCEL:
                response.success = False
                response.message = "Cancel error! There is nothing to cancel."
                
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
        
        # 로봇이 이용자 명령을 수행하고 있는 경우
        else:
            if request.command_type == CommandConstants.CANCEL and self.current_user == request.user_name:
                standby_point = self.return_location
                self.send_goal.send_goal(standby_point)  # 대기 위치로 이동
                self.current_user = ""
                self.robot_state = RobotStatus.STATUS_RETURN
                self.optimal_waypoint = []
                
                response.success = True
                response.message = "Request received! The robot is starting to return to the standby position!"
                
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
    
    def goal_completed_callback(self):
        # 목표 지점에 도달했을 때 호출되는 콜백
        if self.robot_state == RobotStatus.STATUS_RETURN:
            # 취소 명령이 완료된 경우
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
                standby_point = self.return_location
                self.send_goal.send_goal(standby_point)
                self.get_logger().info("Returning to base.")
                self.current_user = None
                self.robot_state = RobotStatus.STATUS_RETURN  # 로봇 상태를 STANDBY로 전환





# class SendGoal(Thread):
#     def __init__(self, node: Node):
#         super().__init__()
#         self.node = node
#         self.waypoints = []
#         self.current_goal_index = 0  # 현재 목표의 인덱스
#         self._action_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
#         self._is_active = False  # 작업 중인지 여부를 나타내는 플래그

#     def run(self):
#         # 이 스레드는 계속 활성화된 상태로 대기하며, 새로운 목표를 기다림
#         while rclpy.ok():
#             if self._is_active and self.current_goal_index < len(self.waypoints):
#                 self.send_next_goal()
#             else:
#                 rclpy.spin_once(self.node, timeout_sec=0.1)

#     def send_goal(self, waypoints):
#         self.waypoints = waypoints
#         self.current_goal_index = 0
#         self._is_active = True  # 새로운 목표가 설정되면 작업 시작

#     def send_next_goal(self):
#         if self.current_goal_index < len(self.waypoints):
#             waypoint = self.waypoints[self.current_goal_index]
#             goal_msg = NavigateToPose.Goal()
#             goal_msg.pose = self.create_pose_stamped(waypoint)
#             self.send_goal(goal_msg)
#         else:
#             self.node.get_logger().info('All goals have been processed.')
#             self.node.goal_completed_callback()
#             self._is_active = False  # 모든 목표가 완료되면 비활성화

#     def create_pose_stamped(self, waypoint):
#         pose = PoseStamped()
#         pose.header.frame_id = "map"
#         pose.header.stamp = self.node.get_clock().now().to_msg()
#         pose.pose.position.x = waypoint.x
#         pose.pose.position.y = waypoint.y
#         pose.pose.orientation.w = 1.0
#         return pose

#     def send_goal(self, goal_msg):
#         self._action_client.wait_for_server()
#         self.node.get_logger().info(f'Sending goal to {goal_msg.pose.position.x}, {goal_msg.pose.position.y}')
#         send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
#         send_goal_future.add_done_callback(self.goal_response_callback)

#     def feedback_callback(self, feedback_msg):
#         feedback = feedback_msg.feedback
#         self.node.get_logger().info(f'Current robot position: {feedback.current_pose}')

#     def goal_response_callback(self, future):
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.node.get_logger().info('Goal rejected')
#             self._is_active = False
#             return

#         self.node.get_logger().info('Goal accepted')
#         result_future = goal_handle.get_result_async()
#         result_future.add_done_callback(self.get_result_callback)

#     def get_result_callback(self, future):
#         result = future.result().result
#         self.node.get_logger().info(f'Navigation result: {result}')
#         self.current_goal_index += 1
#         self.send_next_goal()  # 다음 목표 전송


    
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
