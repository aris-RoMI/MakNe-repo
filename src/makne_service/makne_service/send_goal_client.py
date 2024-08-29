import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from threading import Thread
from nav2_msgs.action import NavigateToPose  # 실제 로봇 내비게이션 액션 사용
from geometry_msgs.msg import PoseStamped  # 목표 위치를 정의하기 위해 필요

class SendGoal(Thread):
    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        self._action_client = ActionClient(self.node, NavigateToPose, '/navigate_to_pose')
        self._current_goal_handle = None

    def run(self):
        # 스레드 실행: rclpy 스핀을 돌려서 ROS 2 이벤트를 처리함
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def send_goal(self, x: float, y: float, z: float = 0.0, qx: float = 0.0, qy: float = 0.0, qz: float = 0.0, qw: float = 1.0):
        # 주어진 목표를 처리
        self.node.get_logger().info(f"Sending goal to: x={x}, y={y}, z={z}, qx={qx}, qy={qy}, qz={qz}, qw={qw}")
        
        # NavigateToPose goal_msg 생성
        goal_msg = NavigateToPose.Goal()

        # 위치 설정
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = z

        # 방향(orientation) 설정
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self._action_client.wait_for_server()

        # 이전 목표가 진행 중이면 취소
        if self._current_goal_handle is not None:
            self.node.get_logger().info('Cancelling the current goal...')
            cancel_future = self._current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
        
        # 새 목표 전송
        send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)


    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.node.get_logger().info(f'Received feedback: {feedback}')
        # 이 부분에서는 피드백 정보를 로깅하거나 필요한 작업을 수행할 수 있음

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().info('Goal rejected')
            return

        self.node.get_logger().info('Goal accepted')
        self._current_goal_handle = goal_handle  # 현재 goal_handle을 저장
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def cancel_done_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.node.get_logger().info('Current goal successfully cancelled.')
        else:
            self.node.get_logger().info('Failed to cancel the current goal.')

    def get_result_callback(self, future):
        result = future.result().result
        self.node.get_logger().info(f'Navigation result: {result}')
        # 목표가 완료되면 추가적인 작업을 수행할 수 있음
        self._current_goal_handle = None  # 목표가 완료되었으므로 handle을 해제
        self.node.goal_completed_callback()  # 목표 완료 후 콜백 호출
        
def main(args=None):
    rclpy.init(args=args)

    # ROS 2 노드 생성
    node = Node('test_nav_goal_sender')

    # SendGoal 클래스 인스턴스 생성 및 스레드 시작
    goal_sender = SendGoal(node)
    goal_sender.start()

    # 특정 포인트로 이동 명령을 전송
    target_pose = {
        'x': 0.6,
        'y': 0.02,
        'z': 0.0,
        'qx': 0.0,
        'qy': 0.0,
        'qz': 0.0,
        'qw': 1.0
    }

    node.get_logger().info('Sending goal to the target point...')
    goal_sender.send_goal(target_pose['x'], target_pose['y'], target_pose['z'], target_pose['qx'], target_pose['qy'], target_pose['qz'], target_pose['qw'])

    # 노드가 종료될 때까지 실행
    rclpy.spin(node)

    # 종료 처리
    goal_sender.join()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
