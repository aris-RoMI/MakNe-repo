from rclpy.action import ActionClient
from rclpy.node import Node

from threading import Thread
from example_interfaces.action import Fibonacci  # 테스트용 메시지 사용

class SendGoal(Thread):
    def __init__(self, node: Node, waypoints):
        super().__init__()
        self.node = node
        self.waypoints = waypoints
        self._action_client = ActionClient(self.node, Fibonacci, 'test_action')  # 테스트용 액션 서버
        self.current_goal_index = 0  # 현재 목표의 인덱스

    def run(self):
        self.node.get_logger().info('SendGoal thread started.')
        self.send_next_goal()

    def send_next_goal(self):
        if self.current_goal_index < len(self.waypoints):
            waypoint = self.waypoints[self.current_goal_index]
            goal_msg = Fibonacci.Goal()  # 실제 액션 메시지로 변경 필요
            goal_msg.order = 5  # 목표 데이터를 초기화하는 예시 (테스트용)

            self.send_goal(goal_msg)
        else:
            # 모든 목표 완료
            self.node.get_logger().info('All goals have been processed.')
            self.node.is_navigating = False  # 외부에서 관리되는 플래그를 False로 설정

    def send_goal(self, goal_msg):
        self._action_client.wait_for_server()  # 서버가 준비될 때까지 대기
        send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.node.get_logger().info(f'Received feedback: {feedback.sequence}')  # 피드백 처리

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().info('Goal rejected')
            self.node.is_navigating = False  # 목표가 거부된 경우 플래그를 False로 설정
            return

        self.node.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.node.get_logger().info(f'Navigation result: {result.sequence}')
        self.current_goal_index += 1
        self.send_next_goal()  # 다음 목표로 전환
