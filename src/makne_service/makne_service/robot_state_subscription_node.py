from library.Constants import Topic, Constants
import rclpy
from rclpy.qos import QoSProfile
from rclpy.node import Node
from message.msg import DispenserStatus
from PyQt5.QtCore import pyqtSignal, QObject

class ROSNodeSignals(QObject):
    robot_status_received = pyqtSignal(str, str, list)
    

class RobotStatusSubscription(Node):
    def __init__(self, signals):
        super().__init__("robot_status_sub")
        
        self.signals = signals
        qos_profile = QoSProfile(depth=Constants.QOS_DEFAULT)
        self.status_subscriber = self.create_subscription(
            DispenserStatus, 
            Topic.ROBOT_STATUS, 
            self.callback, 
            qos_profile=qos_profile
        )
        
    def callback(self, data):
        seq_no = data.seq_no
        node_status = data.node_status
        component = data.component
        

        self.signals.robot_status_received.emit(seq_no, node_status, component)

def main(args=None):
    rclpy.init(args=args)
    
    signals = ROSNodeSignals()
    node = RobotStatusSubscription(signals)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()