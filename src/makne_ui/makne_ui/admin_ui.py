from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5 import uic
from PyQt5.QtCore import *
import sys
import os
import yaml
import re
import glob
from io import BytesIO
from PIL import Image

from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory


admin_file = os.path.join(get_package_share_directory("makne_ui"), "ui", "admin_service.ui")
admin_ui = uic.loadUiType(admin_file)[0]



class AdminGUI(QMainWindow, admin_ui):
    def __init__(self):    
        super().__init__()
        self.setupUi(self)
        
        # self.nav = BasicNavigator()
        # self.goal_pose = PoseStamped()
        # self.goal_pose.header.frame_id = "map"
        # self.update_goal_pose()

        self.xLabel.setText("0")
        self.yLabel.setText("0")
        self.yawLabel.setText("0")
        
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_map)
        self.timer.start(100)


        with open(os.path.join(get_package_share_directory("makne_ui"), "data", "room_103.yaml")) as f:
            map_data = yaml.full_load(f)

        self.map_resolution = map_data["resolution"]
        self.map_origin = map_data["origin"][:2]

        self.pixmap = QPixmap(os.path.join(get_package_share_directory("makne_ui"), "data", map_data["image"]))
        self.height = self.pixmap.size().height()
        self.width = self.pixmap.size().width()
        self.image_scale = 2
        self.pixmap = self.pixmap.transformed(QTransform().scale(-1, -1))
        self.mapLabel.setPixmap(self.pixmap.scaled(self.width * self.image_scale, self.height * self.image_scale, Qt.KeepAspectRatio))
        self.mapLabel.setAlignment(Qt.AlignCenter)
        
        # header = self.requestTable.horizontalHeader()
        # header.setSectionResizeMode(QHeaderView.Stretch)

        self.x_location = 0.0
        self.y_location = 0.0

    def update_amcl_pose(self, x, y):
        self.x_location = x
        self.y_location = y

    def update_path_distance(self, distance):
        if distance < 0.4:
            distance = 0
        self.remainLabel.setText(str("{:.2f}".format(distance)))

    def update_task_request(self, x, y, yaw):
        self.xLabel.setText(str(x))
        self.yLabel.setText(str(y))
        self.yawLabel.setText(str(yaw))
        self.topic_test()
        
    # def update_goal_pose(self, x=None, y=None, yaw=None):
    #     if x is None:
    #         x = float(self.xLabel.text())
    #     if y is None:
    #         y = float(self.yLabel.text())
    #     if yaw is None:
    #         yaw = float(self.yawLabel.text())

    #     roll, pitch, yaw = 0.0, 0.0, yaw
    #     quaternion = quaternion_from_euler(roll, pitch, yaw)

    #     self.goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
    #     self.goal_pose.pose.position.x = x
    #     self.goal_pose.pose.position.y = y
    #     self.goal_pose.pose.position.z = 0.0
    #     self.goal_pose.pose.orientation.x = quaternion[0]
    #     self.goal_pose.pose.orientation.y = quaternion[1]
    #     self.goal_pose.pose.orientation.z = quaternion[2]
    #     self.goal_pose.pose.orientation.w = quaternion[3]
        
    def topic_test(self):
        self.update_goal_pose()
        self.nav.goToPose(self.goal_pose)
        
    def update_map(self):
        self.mapLabel.setPixmap(self.pixmap.scaled(self.width * self.image_scale, self.height * self.image_scale, Qt.KeepAspectRatio))
        self.mapLabel.setAlignment(Qt.AlignCenter)
        
        painter = QPainter(self.mapLabel.pixmap())
        
        painter.setPen(QPen(Qt.red, 20, Qt.SolidLine))
        
        x, y = self.calc_coord(self.x_location, self.y_location)
        
        painter.drawPoint(int((self.width - x) * self.image_scale), int(y * self.image_scale))
        painter.drawText(int((self.width - x) * self.image_scale + 13), int(y * self.image_scale + 5), '1')
        
    def calc_coord(self, x, y):
        pos_x = (x - self.map_origin[0]) / self.map_resolution
        pos_y = (y - self.map_origin[1]) / self.map_resolution
        return pos_x, pos_y
            

def main():
    app = QApplication(sys.argv)
    myWindow = AdminGUI()
    myWindow.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()