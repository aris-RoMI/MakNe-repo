import rclpy
import rclpy
from rclpy.node import Node
from threading import Thread
from makne_service.waypoint_calculator import WayPointCalculator
from makne_service.dummy_action_client import SendGoal
from makne_msgs.srv import SetPointList
from makne_msgs.msg import Pose2D
from library.Constants import CommandConstants, DBConstants, RobotStatus

class RobotManager(Node):
    def __init__(self):
        super().__init__('robot_manager')
        # SetPointList 서비스 생성
        self.srv = self.create_service(SetPointList, 'set_pointlist', self.set_waypoint_callback)
        
        # WayPointCalculator 초기화
        self.waypoint_calculator = WayPointCalculator(DBConstants.DB_NAME)
        
        # 로봇의 상태 초기화
        self.robot_state = RobotStatus.STATUS_STANDBY
        self.is_moving_to_cafe = False  # 로봇이 카페로 이동 중인지 여부
        self.is_canceling = False  # 취소 명령이 처리 중인지 여부
        
        # 주문 및 경로 리스트 초기화
        self.points = []
        self.order_list = []
        self.return_location = self.waypoint_calculator.get_point_from_location_name(DBConstants.STANDBY)

        # SendGoal 인스턴스 생성
        self.send_goal = SendGoal(self)

    def set_waypoint_callback(self, request, response):
        # 현재 로봇이 ORDER나 CALL 상태에서 움직이고 있는 경우
        if self.robot_state in [RobotStatus.STATUS_ORDER, RobotStatus.STATUS_CALL]:
            if request.command_type == CommandConstants.CANCEL:
                # 취소 명령이 들어왔을 때
                if not self.is_canceling:
                    # 취소 처리 시작
                    standby_point = self.waypoint_calculator.get_point_from_location_name(DBConstants.STANDBY)
                    self.send_goal.start_new_goal([standby_point])  # 대기 위치로 이동
                    self.send_goal.start()  # 스레드를 시작하여 목표를 처리
                    self.is_canceling = True
                    self.robot_state = RobotStatus.STATUS_CANCEL
                    self.order_list = []  # 모든 남은 목표 제거
                    response.success = True
                    response.message = "Request canceled. Robot is moving to the standby location."
                else:
                    # 이미 취소 처리가 진행 중인 경우
                    response.success = False
                    response.message = "Request denied. Robot is already moving to the standby location."

            elif request.command_type == CommandConstants.ORDER and self.is_moving_to_cafe:
                # 카페로 이동 중 새로운 주문이 들어온 경우
                self.points = self.merge_unique_points(self.points, request.point_list)
                self.get_logger().info(f"Received {len(request.point_list)} waypoints")
                response.success = True
                response.message = "Waypoints received and robot is moving to the cafe."

            else:
                # 다른 명령을 처리할 수 없는 상태
                self.get_logger().info("Robot is currently navigating. Please try again later.")
                response.success = False
                response.message = "Robot is currently navigating. Please try again in a few minutes."

        else:
            # 로봇이 STANDBY 상태일 때 새로운 명령 처리
            if request.command_type == CommandConstants.ORDER:
                self.robot_state = RobotStatus.STATUS_ORDER

                # 카페로 이동 전 주문 수집 및 중복 제거
                self.points = self.merge_unique_points(self.points, request.point_list)
                self.get_logger().info(f"Received {len(request.point_list)} waypoints")
                self.is_moving_to_cafe = True
                cafe_location = self.waypoint_calculator.get_point_from_location_name(DBConstants.CAFE)
                self.send_goal.start_new_goal([cafe_location])
                self.send_goal.start()  # 스레드를 시작하여 목표를 처리

                response.success = True
                response.message = "Waypoints received and robot is moving to the cafe."

            elif request.command_type == CommandConstants.CALL:
                # CALL 명령 처리
                self.robot_state = RobotStatus.STATUS_CALL
                request_point = self.waypoint_calculator.get_point_from_location_name(request.point_list[0])
                self.send_goal.start_new_goal([request_point])
                self.send_goal.start()  # 스레드를 시작하여 목표를 처리

                response.success = True
                response.message = "Request received and robot is moving to the location."

            elif request.command_type == CommandConstants.CANCEL:
                # 이미 STANDBY 상태에서 취소 명령이 들어온 경우
                response.success = False
                response.message = "Robot is already in standby mode."

        return response


    def merge_unique_points(self, existing_points, new_points):
        # 기존 포인트 리스트와 새로운 포인트 리스트를 병합하며 중복을 제거
        combined_points = existing_points + [p for p in new_points if p not in existing_points]
        return combined_points

    def goal_completed_callback(self):
        # 목표 지점에 도달했을 때 호출되는 콜백
        if self.robot_state == RobotStatus.STATUS_CANCEL:
            # 취소 명령이 완료된 경우
            self.get_logger().info("Task cancelled, robot is now at standby location.")
            self.robot_state = RobotStatus.STATUS_STANDBY  # 로봇 상태를 STANDBY로 전환
            self.is_moving_to_cafe = False  # 카페 이동 상태 리셋
            self.is_canceling = False  # 취소 상태 리셋
            return

        if self.robot_state == RobotStatus.STATUS_ORDER and self.is_moving_to_cafe:
            # 로봇이 ORDER 상태에서 카페에 도착한 경우
            self.is_moving_to_cafe = False
            self.get_logger().info("Reached cafe, processing orders.")

            # 주문 리스트 최적화 및 경로 설정
            self.order_list = self.waypoint_calculator.calculate_optimal_route(self.points)
            self.get_logger().info(f"Optimal route calculated with {len(self.order_list)} waypoints")

            # 최적 경로를 순회 시작
            self.send_goal.start_new_goal(self.order_list + [self.return_location])
        
        elif self.robot_state in [RobotStatus.STATUS_ORDER, RobotStatus.STATUS_CALL]:
            # 다음 목표로 이동하거나, 모든 목표 완료 후 복귀 처리
            if len(self.order_list) > 0:
                next_goal = self.order_list.pop(0)
                self.get_logger().info(f"Moving to next goal: {next_goal}")
                self.send_goal.start_new_goal([next_goal])
            else:
                # 모든 목표 완료 후 복귀 처리
                standby_point = self.waypoint_calculator.get_point_from_location_name(DBConstants.STANDBY)
                self.send_goal.start_new_goal([standby_point])
                self.get_logger().info("Returning to base.")
                self.robot_state = RobotStatus.STATUS_STANDBY  # 로봇 상태를 STANDBY로 전환
                self.is_moving_to_cafe = False  # 카페 이동 상태 리셋
                self.is_canceling = False  # 취소 상태 리셋



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

#     def start_new_goal(self, waypoints):
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
