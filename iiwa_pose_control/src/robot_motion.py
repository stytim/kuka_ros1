#!/usr/bin/env python
# -*-coding:utf-8 -*-
'''
@File    :   control_panel.py
@Time    :   2025/01/13 20:50:58
@Author  :   Feng Li  
@Contact :   feng.li@tum.de
@Desc    :   
'''

# qt
from PySide6.QtCore import QCoreApplication, QTimer, Qt, QThread, Signal
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import QApplication, QMessageBox, QWidget, QGraphicsScene
from PySide6.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLineEdit, QLabel

import sys
sys.path.append('/home/narvis/ros_ws/src/iiwa_pose_control')
from ui.Ui_robot_motion import Ui_Motion

# modules
import sys

# ros
import rospy
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool, Time

# robot
from iiwa_msgs.msg import CartesianPose
from iiwa_msgs.srv import ConfigureControlMode, ConfigureControlModeRequest, SetSpeedOverride
from iiwa_msgs.msg import ControlMode

# cv
import cv2
from cv_bridge import CvBridge

# plt
import matplotlib.pyplot as plt

# computation
import numpy as np
from collections import deque

from transforms3d import euler, quaternions
from scipy.spatial.transform import Rotation as R, Slerp
from scipy import ndimage
import math

# others
import time
from os import path
import yaml

class ControlPanel(object):
    def __init__(self,
                 project_path,
                 probe_path='/config/probe_linear.txt',
                 control_service='/iiwa/configuration/ConfigureControlMode',
                 robot_topic='/iiwa/state/CartesianPose',
                 iiwa_lin_pub_topic='/iiwa/command/CartesianPoseLin',
                 rob_command_topic='/robot_command',
                 max_rob_x=0.77,
                 lowest_rob_z=0.18
                 ):
        super(ControlPanel, self).__init__()
        rospy.init_node('robot_motion_node', anonymous=True)
        
        self.cv_bridge = CvBridge()
        self.tf = Transform()
        self.compute = Compute()

        self.max_rob_x = max_rob_x
        self.lowest_rob_z = lowest_rob_z

        self.control_service = control_service
        self.iiwa_lin_pose_pub = rospy.Publisher(iiwa_lin_pub_topic, PoseStamped, queue_size=10)
        self.probe_path = probe_path
        self.probe_to_ee = self.compute.get_coordinate(project_path + probe_path)

        file_path = path.join(project_path, 'config', 'motion.yaml')
        with open(file_path, 'r') as f:
            self.motion_para = yaml.load(f.read(), Loader=yaml.FullLoader)

        ## initialize parameters
        self.init_parameters()
        self.command_str = None
        self.robot_pose = None
        self.robot_pose_7d = []
        self.init_pose = np.array([0.5, 0.0, 0.42, 0, 1, 0, 0])

        # UI
        # UI and Button
        self.vis = QWidget()
        self.ui = Ui_Motion()
        self.ui.setupUi(self.vis)
        self.vis.setWindowFlags(self.vis.windowFlags() & ~Qt.WindowMaximizeButtonHint)

        self.ui.sample_start.clicked.connect(self.handle_sample_start)
        self.ui.sample_end.clicked.connect(self.handle_sample_end)
        self.ui.generate.clicked.connect(self.handle_generate)
        self.ui.start_rob.clicked.connect(self.handle_start)
        self.ui.pause.clicked.connect(self.handle_pause)
        self.ui.stop_rob.clicked.connect(self.handle_stop)
        self.ui.reset.clicked.connect(self.handle_reset)
        self.ui.quit.clicked.connect(QApplication.instance().quit)
        
        try:
            rospy.Subscriber(robot_topic, CartesianPose, self.rob_callback)
            rospy.wait_for_message(robot_topic, CartesianPose, timeout=1)
        except rospy.exceptions.ROSException as e:
            print("No robot topic. Please check it.")

        ## Robot command
        self.command_str = ''
        try:
            rospy.Subscriber(rob_command_topic, String, self.rob_command_callback)
        except rospy.exceptions.ROSException as e:
            print("There is no rob commandtopic. Please check it.") 

        ## Subscribe iiwa
        self.destination_reached = False
        try:
            rospy.Subscriber('/iiwa/state/DestinationReached', Time, self.destination_reached_callback)
        except rospy.exceptions.ROSException as e:
            print("There is no destination topic. Please check it.") 

        ## Timer
        rospy.Timer(rospy.Duration(0.05), self.robot_motion_detection)
        rospy.Timer(rospy.Duration(0.02), self.show_robot_states)

        self.command_timer = QTimer()
        self.command_timer.timeout.connect(self.auto_execuate_command)
        self.command_timer.start(10)

        self.trajectory_timer = QTimer()
        self.trajectory_timer.timeout.connect(self.motion)
        self.trajectory_timer.start(500)

        # self.generate_timer = QTimer()
        # self.generate_timer.timeout.connect(self.generate_path)
        # self.generate_timer.start(10)
    
    def handle_sample_start(self):
        if len(self.robot_pose_7d) == 0:
            return
        start_p = self.robot_pose_7d
        start_p_mat = self.tf.quat_to_matrix(start_p)
        start_probe_to_base = np.dot(start_p_mat, self.probe_to_ee)

        self.start_point = self.tf.matrix_to_quat(start_probe_to_base)

        pose_show = self.tf.matrix_to_angle(start_probe_to_base, unit_m_to_mm=True)
        pose_show = np.round(pose_show, decimals=2)
        self.ui.start_point.setText(self.tf.fstr(pose_show))

    def handle_sample_end(self):
        if len(self.robot_pose_7d) == 0:
            return
        end_p = self.robot_pose_7d
        end_p_mat = self.tf.quat_to_matrix(end_p)
        end_probe_to_base = np.dot(end_p_mat, self.probe_to_ee)

        self.end_point = self.tf.matrix_to_quat(end_probe_to_base)

        pose_show = self.tf.matrix_to_angle(end_probe_to_base, unit_m_to_mm=True)
        pose_show = np.round(pose_show, decimals=2)
        self.ui.end_point.setText(self.tf.fstr(pose_show))

    def handle_generate(self):
        if len(self.start_point) == 0 or len(self.end_point) == 0:
            QMessageBox.about(self.vis, 'Warning', 'Please get start and end points first! ')
            return

        try:
            number = int(self.ui.number.text())
        except Exception as e:
            QMessageBox.about(self.vis, 'Warning', 'Please input the number! ')
            return

        self.generated_points = []
        start_6d = self.tf.quat_to_angle(self.start_point)
        end_6d = self.tf.quat_to_angle(self.end_point)
        print(f'start: {start_6d} \nend: {end_6d}')
        tx_s, ty_s, tz_s, rx_s, ry_s, rz_s = start_6d
        tx_e, ty_e, tz_e, rx_e, ry_e, rz_e = end_6d
        test1 = []
        for num in range(number):
            delta_tx_s = self.motion_para["delta_tx_s"] * np.random.uniform(-1, 1)
            delta_tx_e = self.motion_para["delta_tx_e"] * np.random.uniform(-1, 1)
            delta_ty_s = self.motion_para["delta_ty_s"] * np.random.uniform(0, 1)
            delta_ty_e = self.motion_para["delta_ty_e"] * np.random.uniform(0, 1)
            delta_tz_s = self.motion_para["delta_tz_s"] * abs(delta_tx_s)
            delta_tz_e = self.motion_para["delta_tz_e"] * abs(delta_tx_e)

            delta_ry_s = self.motion_para["delta_ry_s"] * delta_tx_s
            delta_ry_e = self.motion_para["delta_ry_e"] * delta_tx_e
            delta_rz = self.motion_para["delta_rz"] * np.random.uniform(-1, 1)

            ry_s_new = ry_s + delta_ry_s
            ry_e_new = ry_e + delta_ry_e
            rz_s_new = rz_s + delta_rz
            rz_e_new = rz_e + delta_rz

            start_p = [tx_s + delta_tx_s, ty_s + delta_ty_s, tz_s - delta_tz_s, rx_s, ry_s_new, rz_s_new ]
            end_p = [tx_e - delta_tx_e, ty_e - delta_ty_e, tz_e + delta_tz_e, rx_e, ry_e_new, rz_e_new]

            # start_p = np.clip(start_p, -179.99, 179.99)
            # end_p = np.clip(end_p, -179.99, 179.99)

            test1.append(start_p)
            test1.append(end_p)

            start_p_quat = self.tf.angle_to_quat(start_p)
            if start_p_quat[0] > self.max_rob_x or start_p_quat[2] < self.lowest_rob_z:
                return
            end_p_quat = self.tf.angle_to_quat(end_p)
            if start_p_quat[0] > self.max_rob_x or start_p_quat[2] < self.lowest_rob_z:
                return

            self.generated_points.append(start_p_quat)
            self.generated_points.append(end_p_quat)

        print(np.array(test1), '\n')

    def handle_start(self):
        if len(self.generated_points) == 0:
            QMessageBox.about(self.vis, 'Warning', 'Please generate the path points first! ')
            return
        
        self.execute_command('start')
    
    def handle_pause(self):
        if self.is_paused:
            self.is_paused = False
            point_list = self.points_running_queue.copy()
            self.is_motion_start = True
            self.movement(point_list)

            self.ui.pause.setText("Pause")
            self.ui.pause.setStyleSheet("background-color: #ffffff")
        else:
            self.is_paused = True
            self.is_motion_start = False
            self.ui.pause.setText("Resume")
            self.ui.pause.setStyleSheet("background-color: #3CB371")
    
    def handle_stop(self):
        # self.set_impedance_control(enable=False)
        # current_probe_pose = self.robot_pose_7d.copy()
        # current_probe_pose[2] += 0.01
        # self.move_to_cartesian_pose(current_probe_pose)
        # self.is_robot_stopped = True

        self.execute_command('stop')
    
    def handle_reset(self):
        self.init_parameters()
        self.set_impedance_control(enable=True, is_impedance=False)
        self.desired_pose_mouse_quat = self.init_pose
        desired_pose_show = self.tf.quat_to_angle(self.desired_pose_mouse_quat, unit_m_to_mm=True)
        desired_pose_show = np.round(desired_pose_show, decimals=2)
        self.ui.desired_pose.setText(self.tf.fstr(desired_pose_show))
        self.move_to_cartesian_pose(self.init_pose)

    def rob_command_callback(self, ros_msg):
        self.command_str = ros_msg.data
        self.automated_command = True

    def destination_reached_callback(self, msg):
        self.destination_reached = True
    
    def motion(self):
        if not self.is_motion_start:
            return
        
        if len(self.points_running_queue) == 0:
            self.is_motion_start = False
            return
        
        point_to_world_quat = self.points_running_queue[0]
        left_number = self.tf.fstr(int(len(self.points_running_queue) / 2))
        self.ui.left.setText(left_number)

        if not self.is_probe_pose_closed(point_to_world_quat):
            self.desired_pose_mouse_quat = point_to_world_quat

            if self.deque_length == len(self.points_running_queue):
                return
            self.move_probe_to_point(point_to_world_quat)
            self.deque_length = len(self.points_running_queue)
                
        else:
            self.points_running_queue.popleft()
    
    #### robot control function
    def robot_motion_detection(self, event):
        if self.robot_pose_7d is None or self.desired_pose_mouse_quat is None:
            self.ui.warning.setText('')
            return
        pose_reached = bool(np.sum(np.abs(self.robot_pose_7d - self.desired_pose_mouse_quat)) < 0.0005)
        self.is_robot_stopped = pose_reached
        if not self.is_robot_stopped:
            self.ui.warning.setText('Robot is moving!')
        else:
            self.ui.warning.setText('Robot is stopped!')
    
    def show_robot_states(self, event):
        if self.robot_pose is not None:
            tvec = self.robot_pose.position
            qvec = self.robot_pose.orientation
            self.rob_t = [tvec.x, tvec.y, tvec.z]
            self.rob_q = [qvec.x, qvec.y, qvec.z, qvec.w]
            self.robot_pose_7d = np.array(self.rob_t + self.rob_q)
            
            ## Show robot end-effector pose
            rob_cur_pose_show = self.tf.quat_to_angle(self.robot_pose_7d, unit_m_to_mm=True)
            rob_cur_pose_show = np.round(rob_cur_pose_show, decimals=2)
            self.ui.robot_pose.setText(self.tf.fstr(rob_cur_pose_show))
        else:
            self.ui.robot_pose.setText('Do not get pose!')
    
    def auto_execuate_command(self):
        if not self.automated_command:
            return

        print(self.command_str)
        self.execute_command(self.command_str)

        self.automated_command = False
    
    def execute_command(self, value):
        if value == 'start':
            if len(self.generated_points) == 0:
                QMessageBox.about(self.vis, 'Warning', 'Please generate the path points first! ')
                return
            
            self.set_impedance_control(enable=True, is_impedance=True)
            self.movement(self.generated_points)

        elif value == 'stop':
            if len(self.generated_points) == 0:
                QMessageBox.about(self.vis, 'Warning', 'Please generate the path points first! ')
                return
            current_probe_pose = self.robot_pose_7d.copy()
            current_probe_pose[2] += 0.01
            self.move_to_cartesian_pose(current_probe_pose)
            self.desired_pose_mouse_quat = self.robot_pose_7d
            self.is_robot_stopped = True

        elif value == 'none':
            pass

        else:
            print('Please check the command!')
        
    def movement(self, point_list):
        if len(point_list) < 2:
            return
        self.points_running_queue = deque(point_list)
        self.deque_length = len(self.points_running_queue) + 1
        self.is_motion_start = True
                
    def is_probe_pose_closed(self, point_q):
        return np.sum(np.abs(self.get_probe_to_base_quat()[:3] - point_q[:3])) < 0.015
    
    def get_probe_to_base_quat(self):
        start_mat = self.tf.quat_to_matrix(self.robot_pose_7d)
        probe_to_base = np.dot(start_mat, self.probe_to_ee)
        probe_to_base_quat = self.tf.matrix_to_quat(probe_to_base)
        return probe_to_base_quat
    
    def move_probe_to_point(self, point_to_world_quat, startend = False):
        point_to_world = self.tf.quat_to_matrix(point_to_world_quat)
        desired_pose = point_to_world @ np.linalg.inv(self.probe_to_ee)
        
        desired_pose_show = self.tf.matrix_to_angle(desired_pose, unit_m_to_mm=True)
        desired_pose_show = np.round(desired_pose_show, decimals=2)
        self.ui.desired_pose.setText(self.tf.fstr(desired_pose_show))
        transform_quat = self.tf.matrix_to_quat(desired_pose)

        self.move_to_cartesian_pose(transform_quat, startend)
    
    #### ros function
    def rob_callback(self, ros_msg):
        self.robot_pose = ros_msg.poseStamped.pose
        self.robot_state_header = ros_msg.poseStamped.header

        if self.robot_pose is not None:
            tvec = self.robot_pose.position
            qvec = self.robot_pose.orientation
            self.rob_t = [tvec.x, tvec.y, tvec.z]
            self.rob_q = [qvec.x, qvec.y, qvec.z, qvec.w]
            self.robot_pose_7d = np.array(self.rob_t + self.rob_q)
        
    def set_impedance_control(self, enable=False, is_impedance=False, stiffness_z=800, weight=1.5):
        if not enable:
            return
        rospy.wait_for_service(self.control_service)
        try:
            configure_control_mode = rospy.ServiceProxy(self.control_service, ConfigureControlMode)
            request = ConfigureControlModeRequest()
            if is_impedance:
                request.control_mode = ControlMode.CARTESIAN_IMPEDANCE

                ## Impedance control
                request.cartesian_impedance.cartesian_stiffness.x = 1200 
                request.cartesian_impedance.cartesian_stiffness.y = 1200
                request.cartesian_impedance.cartesian_stiffness.z = stiffness_z
                request.cartesian_impedance.cartesian_stiffness.a = 200
                request.cartesian_impedance.cartesian_stiffness.b = 200
                request.cartesian_impedance.cartesian_stiffness.c = 200

                request.cartesian_impedance.cartesian_damping.x = 0.8
                request.cartesian_impedance.cartesian_damping.y = 0.8
                request.cartesian_impedance.cartesian_damping.z = 0.88
                request.cartesian_impedance.cartesian_damping.a = 0.8
                request.cartesian_impedance.cartesian_damping.b = 0.8
                request.cartesian_impedance.cartesian_damping.c = 0.8

            else:
                request.control_mode = ControlMode.POSITION_CONTROL
            resp = configure_control_mode(request)
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            return False
    
    def move_to_cartesian_pose(self, desired_pose, startend=False):
        posemsg = PoseStamped()
        posemsg.header.frame_id = "iiwa_link_0"

        posemsg.pose.position.x = desired_pose[0]
        posemsg.pose.position.y = desired_pose[1]
        posemsg.pose.position.z = desired_pose[2]
        posemsg.pose.orientation.x = desired_pose[3]
        posemsg.pose.orientation.y = desired_pose[4]
        posemsg.pose.orientation.z = desired_pose[5]
        posemsg.pose.orientation.w = desired_pose[6]

        self.destination_reached = False
        self.iiwa_lin_pose_pub.publish(posemsg)
        if startend:
            rospy.loginfo("Moving to pose.")
            while not self.destination_reached:
                rospy.sleep(0.5)
            rospy.loginfo("Reached start point, continue scan.")
            self.execute_command("trajectory")
     
    def init_parameters(self):
        self.start_point = []
        self.end_point = []
        self.generated_points = []
        self.points_running_queue = deque()
        self.deque_length = 0

        self.is_paused = False

        self.is_motion_start = False
        self.is_robot_stopped = True
        self.desired_pose_mouse_quat = None

        self.automated_command = False

class Compute():
    def __init__(self):
        self.tf = Transform()

    def get_calibration_res(self, filepath):
        calibration_res = np.identity(4)
        with open(filepath, 'r') as file:
            txt_list = file.readlines()
            if len(txt_list) != 6:
                return
            for index, txt_line in enumerate(txt_list):
                if index == 4:
                    break
                txt_line = txt_line[0:-1]
                line = [float(entry) for entry in txt_line.split('\t')]
                calibration_res[index] = line
        return calibration_res
    
    def get_coordinate(self, filepath):
        mat = np.identity(4)
        with open(filepath, 'r') as file:
            txt_list = file.readlines()
            if len(txt_list) != 6:
                return
            for index, txt_line in enumerate(txt_list):
                if index == 4:
                    break
                txt_line = txt_line[0:-1]
                line = [float(entry) for entry in txt_line.split(' ')]
                mat[index] = line
        return mat

class Transform():
    def __init__(self):
        pass

    def get_max_min(self, ee_to_base_angle):
        if len(ee_to_base_angle) == 0:
            return [], []
        max_tmp = np.max(ee_to_base_angle, axis=0)
        min_tmp = np.min(ee_to_base_angle, axis=0)
        max_value = np.full(6, -1000.0)
        min_value = np.full(6, 1000.0)
        for i in range(6):
            max_value[i] = max(max_value[i], max_tmp[i])
            min_value[i] = min(min_value[i], min_tmp[i])
        return np.round(max_value, decimals=3), np.round(min_value, decimals=3)

    def fstr(self, astr):
        format_str = str(astr)
        format_str = format_str.strip('[')
        format_str = format_str.strip(']')
        format_str = '\t'.join(format_str.split())
        return format_str

    def pose_7d_to_pose_6d(self, pose_7d, unit_m_to_mm=False):
        length = pose_7d.shape[0]
        pose_6d = np.empty((length, 6))
        for i in range(length):
            angle = self.quat_to_angle(pose_7d[i], unit_m_to_mm)
            pose_6d[i] = angle
        return np.round(pose_6d, decimals=3)

    def matrix_to_quat(self, matrix):
        trans = matrix[:3, 3]
        rot = matrix[:3, :3]
        qua = self.wxyz_to_xyzw(quaternions.mat2quat(rot)) 
        pose = np.empty(7)
        pose[:3] = trans
        pose[3:7] = qua
        return pose

    def matrix_to_angle(self, matrix, unit_m_to_mm=False):
        trans = matrix[:3, 3]
        rot = matrix[:3, :3]
        angle = euler.mat2euler(rot)
        pose = np.empty(6)
        pose[:3] = trans
        if unit_m_to_mm:
            pose[:3] = pose[:3] * 1000
        angle_new = []
        for value in angle:
            value = self.deg(value)
            angle_new.append(value)
        pose[3:6] = angle_new
        return pose

    def angle_to_matrix(self, pose):
        if len(pose) != 6:
            return
        trans = pose[:3]
        angle = pose[3:]
        rot = euler.euler2mat(self.rad(angle[0]), self.rad(angle[1]), self.rad(angle[2]))
        matrix = np.identity(4)
        matrix[:3, 3] = trans
        matrix[:3, :3] = rot
        return matrix

    def angle_to_quat(self, ang):
        if len(ang) != 6:
            return
        trans = ang[: 3]
        angle = ang[3:]
        qua = self.wxyz_to_xyzw(euler.euler2quat(self.rad(angle[0]), self.rad(angle[1]), self.rad(angle[2])))
        return np.append(trans, qua)

    def quat_to_angle(self, qua, unit_m_to_mm=False):
        if len(qua) != 7:
            return 
        trans = qua[: 3]
        qua = self.xyzw_to_wxyz(qua[3:])
        angle = euler.quat2euler(qua)
        pose = np.empty(6)
        pose[:3] = trans
        if unit_m_to_mm:
            pose[:3] = pose[:3] * 1000
        angle_new = []
        for value in angle:
            value = self.deg(value)
            angle_new.append(value)
        pose[3:] = angle_new
        return pose

    def quat_to_matrix(self, pose):
        if len(pose) != 7:
            return
        trans = pose[: 3]
        qua = pose[3:]
        # wxyz
        rot = euler.quat2mat(self.xyzw_to_wxyz(qua))
        matrix = np.identity(4)
        matrix[:3, 3] = trans
        matrix[:3, :3] = rot
        return matrix

    def xyzw_to_wxyz(self, qua):
        if len(qua) != 4:
            return
        return np.append([qua[3]], qua[: 3])

    def wxyz_to_xyzw(self, qua):
        if len(qua) != 4:
            return
        return np.append(qua[1:], [qua[0]])

    def rad(self, degree):
        return degree * math.pi / 180

    def deg(self, radian):
        return radian * 180 / math.pi

if __name__ == '__main__':
    QCoreApplication.setAttribute(Qt.AA_ShareOpenGLContexts)
    app = QApplication([])
    project_path = path.dirname(path.dirname(__file__))
    np.set_printoptions(precision=6, suppress=True)
    visualization = ControlPanel(project_path)
    visualization.vis.show()
    sys.exit(app.exec())