import rclpy
from rclpy.action import ActionServer
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
            self.execute_callback
        )
        self.get_logger().info("Test Action Server is ready.")

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = Fibonacci.Feedback()
        
        # 피드백 메시지의 올바른 필드 사용
        feedback_sequence = []

        # 예제 시퀀스 생성 및 피드백 전송
        for i in range(5):
            feedback_sequence.append(i)
            feedback_msg.sequence = feedback_sequence  # 올바른 필드 이름 사용
            self.get_logger().info(f'Publishing feedback: {feedback_sequence}')
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)  # 각 피드백 간 1초 대기

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
