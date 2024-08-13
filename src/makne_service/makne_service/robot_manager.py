import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
# from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from makne_msgs.srv import SetWaypoint
from makne_msgs.msg import Pose2D

class RobotManager(Node):
    def __init__(self):
        super().__init__('robot_manager')
        # self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 서비스 서버 생성
        self.waypoint_server = self.create_service(SetWaypoint, '/set_waypoint', self.set_waypoint_callback)

    def set_waypoint_callback(self, request, response):
        # 웨이포인트 요청을 처리하고, 각 웨이포인트를 로깅합니다.
        for i, waypoint in enumerate(request.waypoints):
            self.get_logger().info(f'Command Type : {request.command_type}, Waypoint {i + 1}: (x={waypoint.x}, y={waypoint.y})')

        response.success = True
        response.message = "Waypoints received, navigating..."

        # Nav2 액션을 비동기적으로 수행합니다.
        # goal_msg = NavigateToPose.Goal()
        # goal_msg.pose = self.create_pose_stamped(request)
        # self._send_goal(goal_msg)

        return response

    # def create_pose_stamped(self, waypoint):
    #     pose = PoseStamped()
    #     pose.header.frame_id = "map"
    #     pose.header.stamp = self.get_clock().now().to_msg()
    #     pose.pose.position.x = waypoint.x
    #     pose.pose.position.y = waypoint.y
    #     pose.pose.orientation.w = 1.0
    #     return pose

    # def _send_goal(self, goal_msg):
    #     self._action_client.wait_for_server()
    #     self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
    #     self._send_goal_future.add_done_callback(self.goal_response_callback)

    # def goal_response_callback(self, future):
    #     goal_handle = future.result()
    #     if not goal_handle.accepted:
    #         self.get_logger().info('Goal rejected')
    #         return

    #     self.get_logger().info('Goal accepted')
    #     self._get_result_future = goal_handle.get_result_async()
    #     self._get_result_future.add_done_callback(self.get_result_callback)

    # def feedback_callback(self, feedback_msg):
    #     feedback = feedback_msg.feedback
    #     self.get_logger().info(f'Current robot position: {feedback.current_pose}')

    # def get_result_callback(self, future):
    #     result = future.result().result
    #     self.get_logger().info(f'Navigation result: {result}')
    
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
