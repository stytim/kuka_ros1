#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import String
from PySide6.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLineEdit, QLabel

class CommandPublisherNode:
    def __init__(self):
        rospy.init_node('command_publisher', anonymous=True)
        self.publisher = rospy.Publisher('/robot_command', String, queue_size=1)

    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.publisher.publish(msg)
        rospy.loginfo(f'Published command: {command}')

class CommandUI(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle('ROS Command Sender')
        self.setGeometry(100, 100, 300, 150)

        layout = QVBoxLayout()
        
        self.label = QLabel('Enter Command:', self)
        layout.addWidget(self.label)
        
        self.command_input = QLineEdit(self)
        layout.addWidget(self.command_input)
        
        self.send_button = QPushButton('Send Command', self)
        self.send_button.clicked.connect(self.send_command)
        layout.addWidget(self.send_button)
        
        self.setLayout(layout)
    
    def send_command(self):
        command = self.command_input.text()
        if command:
            self.ros_node.publish_command(command)
            self.command_input.clear()

if __name__ == '__main__':
    ros_node = CommandPublisherNode()
    
    app = QApplication(sys.argv)
    ui = CommandUI(ros_node)
    ui.show()
    
    try:
        sys.exit(app.exec())
    except SystemExit:
        rospy.signal_shutdown('Shutting down')
