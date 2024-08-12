from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5 import uic
from PyQt5.QtCore import *

import sys, os

from library.Constants import SlackCommand
from ament_index_python.packages import get_package_share_directory

slack_file = os.path.join(get_package_share_directory("makne_ui"), "ui", "slack_test.ui")
slack_ui = uic.loadUiType(slack_file)[0]


class SlackWindow(QMainWindow, slack_ui):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.sendBtn.clicked.connect(self.check_message)
        self.commandLine.returnPressed.connect(self.check_message)
        self.command_list = [SlackCommand.ORDER, SlackCommand.CALL, SlackCommand.CANCEL]
        
    def check_message(self):
        message = self.commandLine.text()
        
        if message[0] == "/":
            service_type = message.split(" ")[0]
            service_type = service_type.replace("/", "")
            print(service_type)
            if service_type in self.command_list:
                self.command_manager(service_type, message)
            else:
                self.responseLine.setText("올바르지 않은 명령입니다(/ 뒤에 적절한 명령어를 입력해주세요)")
                self.commandLine.setText("")
            
    def command_manager(self, service_type, message):
        if service_type == SlackCommand.ORDER:
            self.responseLine.setText(SlackCommand.ORDER)
            
        elif service_type == SlackCommand.CALL:
            self.responseLine.setText(SlackCommand.CALL)
            
        elif service_type ==SlackCommand.CANCEL:
            self.responseLine.setText(SlackCommand.CANCEL)
            
        else:
            self.responseLine.setText(f"올바르지 않은 명령입니다(가능한 명령어 : {SlackCommand.ORDER}, {SlackCommand.CALL}, {SlackCommand.CANCEL})")
        self.commandLine.setText("")
        
        
def main():
    app = QApplication(sys.argv)
    slack_window = SlackWindow()
    slack_window.show()
    
    sys.exit(app.exec_())
    
if __name__ == "__main__":
    main()