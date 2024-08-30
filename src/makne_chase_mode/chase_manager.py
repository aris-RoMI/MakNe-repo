import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import SetBool
from threading import Thread

from yolo_orb_tracker_main import YoloOrbTracker
from polar_calculator_main import PolarCalculator
from vfh_navigator_main import VFHNavigator


class ChaseManager(Node):
    def __init__(self, executor):
        super().__init__('chase_manager')
        self.srv = self.create_service(SetBool, '/chase_signal', self.manage_nodes_callback)

        self.yolo_orb_tracker_node = YoloOrbTracker()
        self.polar_calculator_node = PolarCalculator()
        self.VFH_navigator_node = VFHNavigator()

        self.nodes_active = False
        self.executor = executor
        
        self.executor.add_node(self)  # ChaseManager 노드를 추가

        self.get_logger().info('추종 매니저 준비 완료')

    def manage_nodes_callback(self, request, response):
        if request.data and not self.nodes_active:
            # Start nodes
            self.get_logger().info('노드 작동 시작...')
            self.executor.add_node(self.yolo_orb_tracker_node)
            self.executor.add_node(self.polar_calculator_node)
            self.executor.add_node(self.VFH_navigator_node)
            self.nodes_active = True
            response.success = True
            response.message = '모든 노드들이 작동중입니다'
        elif not request.data and self.nodes_active:
            # Stop nodes
            self.get_logger().info('Stopping nodes...')
            self.executor.remove_node(self.yolo_orb_tracker_node)
            self.executor.remove_node(self.polar_calculator_node)
            self.executor.remove_node(self.VFH_navigator_node)
            self.nodes_active = False
            response.success = True
            response.message = '모든 노드들이 중단되었습니다'
        else:
            response.success = False
            response.message = '노드들의'
        
        return response



def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    chase_manager = ChaseManager(executor)

    # executor 스핀을 별도의 스레드에서 실행
    executor_thread = Thread(target=executor.spin)
    executor_thread.start()

    try:
        # ChaseManager 노드는 이미 executor에 의해 관리되고 있으므로 따로 spin을 돌릴 필요가 없습니다.
        executor_thread.join()  # 메인 스레드가 종료되지 않도록 유지
    finally:
        executor.shutdown()
        executor_thread.join()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

