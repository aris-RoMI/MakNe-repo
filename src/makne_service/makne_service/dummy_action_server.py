import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from example_interfaces.action import Fibonacci  # ROS 2에서 제공하는 기본 Action 메시지
import time

class TestActionServer(Node):

    def __init__(self):
        super().__init__('test_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            '/navigate_to_pose',  # 클라이언트와 일치하는 액션 이름 사용
            self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback  # Cancel callback 추가
        )
        self._current_goal_handle = None
        self.get_logger().info("Test Action Server is ready.")

    def goal_callback(self, goal_request):
        # 새로운 goal이 들어왔을 때 호출됨
        if self._current_goal_handle is not None and self._current_goal_handle.is_active:
            self.get_logger().info('A new goal was received, cancelling the current goal.')
            self._current_goal_handle.abort()  # 현재 goal을 중지시킴
        return rclpy.action.GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        # goal이 수락되었을 때 호출됨
        if self._current_goal_handle is not None and self._current_goal_handle.is_active:
            self.get_logger().info('Cancelling the current goal.')
            self._current_goal_handle.abort()  # 현재 goal을 중지시킴

        self._current_goal_handle = goal_handle
        self._current_goal_handle.execute()

    def cancel_callback(self, goal_handle):
        # 목표가 취소되었을 때 호출됨
        self.get_logger().info('Cancelling goal...')
        if self._current_goal_handle == goal_handle and self._current_goal_handle.is_active:
            self._current_goal_handle = None
            return CancelResponse.ACCEPT
        else:
            return CancelResponse.REJECT

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = Fibonacci.Feedback()
        
        # 피드백 메시지의 올바른 필드 사용
        feedback_sequence = []

        # 예제 시퀀스 생성 및 피드백 전송
        for i in range(5):
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal cancelled.')
                if goal_handle.is_active:
                    goal_handle.canceled()
                return Fibonacci.Result()

            feedback_sequence.append(i)
            feedback_msg.sequence = feedback_sequence  # 올바른 필드 이름 사용
            self.get_logger().info(f'Publishing feedback: {feedback_sequence}')
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)  # 각 피드백 간 1초 대기

        if goal_handle.is_active:
            goal_handle.succeed()
            result = Fibonacci.Result()
            result.sequence = feedback_sequence
            self.get_logger().info('Goal completed.')

            return result

def main(args=None):
    rclpy.init(args=args)

    action_server = TestActionServer()

    executor = MultiThreadedExecutor()
    rclpy.spin(action_server, executor=executor)

    action_server.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
