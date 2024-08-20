import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from threading import Thread
from example_interfaces.action import Fibonacci  # 더미 액션 메시지 사용

class SendGoal(Thread):
    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        self.waypoints = []
        self.current_goal_index = 0  # 현재 목표의 인덱스
        self._action_client = ActionClient(self.node, Fibonacci, 'navigate_to_pose')
        self._is_active = False  # 작업 중인지 여부를 나타내는 플래그

    def run(self):
        # 이 스레드는 계속 활성화된 상태로 대기하며, 새로운 목표를 기다림
        while rclpy.ok():
            if self._is_active and self.current_goal_index < len(self.waypoints):
                self.send_next_goal()
            else:
                rclpy.spin_once(self.node, timeout_sec=0.1)

    def start_new_goal(self, waypoints):
        # 이미 작업 중인 경우 추가 목표를 받지 않음
        if self._is_active:
            self.node.get_logger().info("Already processing a goal. Please wait.")
            return

        self.waypoints = waypoints
        self.current_goal_index = 0
        self._is_active = True  # 새로운 목표가 설정되면 작업 시작

    def send_next_goal(self):
        if self.current_goal_index < len(self.waypoints):
            waypoint = self.waypoints[self.current_goal_index]
            goal_msg = Fibonacci.Goal()
            goal_msg.order = 5  # 더미 데이터

            self.send_goal(goal_msg)
        else:
            self.node.get_logger().info('All goals have been processed.')
            self._is_active = False
            self.node.goal_completed_callback()  # 목표 완료 콜백 호출

    def send_goal(self, goal_msg):
        self._action_client.wait_for_server()
        self.node.get_logger().info(f'Sending dummy goal: {goal_msg.order}')
        send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.node.get_logger().info(f'Received dummy feedback: {feedback.sequence}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().info('Goal rejected')
            self._is_active = False
            return

        self.node.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.node.get_logger().info(f'Dummy Navigation result: {result.sequence}')
        self.current_goal_index += 1
        if self.current_goal_index < len(self.waypoints):
            self.send_next_goal()  # 다음 목표 전송
        else:
            self.node.get_logger().info("All waypoints processed.")
            self._is_active = False
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
