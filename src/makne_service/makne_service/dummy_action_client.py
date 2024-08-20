import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from threading import Thread
from example_interfaces.action import Fibonacci  # 더미 액션 메시지 사용

class SendGoal(Thread):
    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        self._action_client = ActionClient(self.node, Fibonacci, 'navigate_to_pose')

    def run(self):
        # 스레드 실행: rclpy 스핀을 돌려서 ROS 2 이벤트를 처리함
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def send_goal(self, waypoint):
        # 주어진 목표를 처리
        self.node.get_logger().info(f"Sending goal to: {waypoint}")
        
        # 더미 goal_msg 생성
        goal_msg = Fibonacci.Goal()
        goal_msg.order = 5  # 더미 데이터, 실제 사용에서는 이 부분을 수정해야 함

        self._action_client.wait_for_server()
        send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.node.get_logger().info(f'Received dummy feedback: {feedback.sequence}')
        self.node.remain_time = feedback.sequence

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().info('Goal rejected')
            return

        self.node.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.node.get_logger().info(f'Dummy Navigation result: {result.sequence}')
        # 목표가 완료되면 RobotManager에게 알림
        self.node.goal_completed_callback()

def main(args=None):
    rclpy.init(args=args)

    node = Node('dummy_action_client')
    dummy_send_goal = SendGoal(node)

    # 임의의 더미 목표 설정
    dummy_send_goal.start_new_goal([PoseStamped(), PoseStamped(), PoseStamped()])
    dummy_send_goal.start()

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
