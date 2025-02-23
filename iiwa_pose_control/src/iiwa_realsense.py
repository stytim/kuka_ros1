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

# modules
import sys

# ros
import rospy
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool

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
        loop_x = np.identity(4)
        with open(filepath, 'r') as file:
            txt_list = file.readlines()
            if len(txt_list) != 6:
                return
            for index, txt_line in enumerate(txt_list):
                if index == 4:
                    break
                txt_line = txt_line[0:-1]
                line = [float(entry) for entry in txt_line.split(' ')]
                loop_x[index] = line
        return loop_x

class Algorithm():
    def __init__(self):
        # 640*480
        self.matrix = np.array([[604.88037361, 0.0, 310.10949692],
                             [0.0, 605.4810827, 244.07677517],
                             [0.0, 0.0, 1.0]])
        self.dist = np.array([-0.01712299, 1.37486846, -0.00030216, -0.00044062, -5.1056172])
        self.tf = Transform()
    
    def pixel_to_point(self, point, depth_queue):
        """
        Based on a queue depth images to optimize the depth value.
        
        Args:
            point (_type_): 2D point from RGB
            depth_queue (_type_): Depth queue

        Returns:
            _type_: _description_
        """
        
        u, v = int(point[0]), int(point[1])
        fx, fy = self.matrix[0, 0], self.matrix[1, 1]
        cx, cy = self.matrix[0, 2], self.matrix[1, 2]

        depth_values = [depth[v, u] for depth in depth_queue if depth[v, u] > 0]  

        if not depth_values:
            return np.array([0, 0, 0])

        Z = np.median(depth_values)
        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy
        return np.array([X, Y, Z])

    def extract_neighborhood_points(self, point, depth, window_size=15, step=4):
        u = point[0]
        v = point[1]
        neighbor_points = []
        h, w = depth[0].shape
        half_win = window_size // 2
        for i in range(max(0, v - half_win), min(h, v + half_win + 1), step):
            for j in range(max(0, u - half_win), min(w, u + half_win + 1), step):
                neighbor_points.append(self.pixel_to_point([j, i], depth))
        return np.array(neighbor_points)

    def fit_plane(self, points):
        ## PCA 
        centroid = np.mean(points, axis=0)
        centered_points = points - centroid

        cov_matrix = np.cov(centered_points, rowvar=False)
        eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)
        normal_vector = eigenvectors[:, np.argmin(eigenvalues)]
        return normal_vector / np.linalg.norm(normal_vector)

    def calculate_rot_by_normal(self, normal_vector, rob_r_mat):
        z_end = rob_r_mat[:, 2]
        normal_vector = normal_vector / np.linalg.norm(normal_vector)
        if np.dot(z_end, normal_vector) < 0:
            normal_vector = -normal_vector
        v = np.cross(z_end, normal_vector)
        s = np.linalg.norm(v)  
        c = np.dot(z_end, normal_vector)  
        if s == 0:
            return rob_r_mat

        k = v / s
        K = np.array([
            [0, -k[2], k[1]],
            [k[2], 0, -k[0]],
            [-k[1], k[0], 0]
        ])
        r_rot = np.eye(3) + np.sin(np.arccos(c)) * K + (1 - np.cos(np.arccos(c))) * K @ K
        return r_rot

    def calculate_rot_mat(self, points, rob_r_mat):
        if points.shape[0] < 3:
            print('Insufficient nerighborhood points!')
            return np.identity(3), False
        normal_vec = self.fit_plane(points)
        rot_mat = self.calculate_rot_by_normal(normal_vec, rob_r_mat)
        return rot_mat, True
    
    def transform_points(self, T, points_local):
        points_world = []
        if points_local.ndim == 0:  
            print("Insufficient points")
            return np.array([0, 0, 0])
        elif points_local.ndim == 1:
            points_local_homogeneous = np.append(points_local, 1)
        else:
            points_local_homogeneous = np.hstack((points_local, np.ones((points_local.shape[0], 1))))
        points_world_homogeneous = (T @ points_local_homogeneous.T).T
        if points_local.ndim == 1:
            points_world = points_world_homogeneous[:3]
        else:
            points_world = points_world_homogeneous[:, :3]
        return np.round(points_world, decimals=2) 
    
    def interpolate_pixel_points(self, points, max_distance=25):
        if len(points) < 2:
            return points  

        result = [points[0]]  
        for i in range(1, len(points)):
            p1 = np.array(points[i - 1])
            p2 = np.array(points[i])
            distance = np.linalg.norm(p2 - p1)
            if distance > max_distance:
                num_new_points = int(np.ceil(distance / max_distance))  
                for j in range(1, num_new_points):
                    new_point = tuple(p1 + (p2 - p1) * j / num_new_points)
                    result.append(new_point)
            result.append(tuple(p2))
        return result

    def interpolate_points(self, points, num):
        interpolated_points = []
        for i in range(len(points) - 1):
            p1 = np.array(points[i][:3])  
            p2 = np.array(points[i + 1][:3])  
            q1 = points[i][3:] 
            q2 = points[i + 1][3:]

            t_values = np.linspace(0, 1, num + 2)
            # Spherical Linear Interpolation
            slerp = Slerp([0, 1], R.from_quat([q1, q2]))
            interpolated_rotations = slerp(t_values)
            for t_idx, t in enumerate(t_values):
                interpolated_position = (1 - t) * p1 + t * p2
                interpolated_quaternion = interpolated_rotations[t_idx].as_quat()
                interpolated_points.append(np.array(list(interpolated_position) + list(interpolated_quaternion)))
        return interpolated_points
    
    def find_quadrant(self, x, y, W, H):
        if x < W / 2 and y < H / 2:
            return "2"
        elif x >= W / 2 and y < H / 2:
            return "1"
        elif x < W / 2 and y >= H / 2:
            return "3"
        else:
            return "4"
        
    def process_mask(self, mask, max_regions_to_keep=1, smallest_size=30):
        labeled_array, num_features = ndimage.label(mask)
        area_sizes = ndimage.sum(mask, labeled_array, range(1, num_features + 1))
        sorted_indices = np.argsort(area_sizes)[::-1]
        result_mask = np.zeros_like(mask, dtype=bool)
        regions_kept = 0
        for i in sorted_indices:
            if area_sizes[i] < smallest_size:
                continue
            result_mask[labeled_array == (i + 1)] = True
            regions_kept += 1
            if regions_kept >= max_regions_to_keep:
                break
        return result_mask

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

class CommandUI(QWidget):
    def __init__(self):
        super().__init__()
        self.init_ui()
        self.is_gen = False

    def init_ui(self):
        self.setWindowTitle('iiwa_realsense!')
        self.setGeometry(100, 100, 300, 150)

        layout = QVBoxLayout()
        
        self.label = QLabel('Welcome to use iiwa realsense!', self)
        layout.addWidget(self.label)

        self.send_button = QPushButton('Generate path', self)
        self.send_button.clicked.connect(self.generate)
        layout.addWidget(self.send_button)
        
        self.setLayout(layout)

    def generate(self):
        print("push")
        self.is_gen = True

class ControlPanel(object):
    def __init__(self,
                 project_path,
                 probe_path='/config/probe_linear.txt',
                 result_path='/config/cali_results.txt',
                 control_service='/iiwa/configuration/ConfigureControlMode',
                 img_topic='/camera/color/image_raw',
                 depth_img_topic='/camera/aligned_depth_to_color/image_raw',
                 robot_topic='/iiwa/state/CartesianPose',
                 iiwa_lin_pub_topic='/iiwa/command/CartesianPoseLin',
                 rob_command_topic='/robot_command'
                 ):
        super(ControlPanel, self).__init__()
        rospy.init_node('iiwa_realsense_node', anonymous=True)
        
        self.cv_bridge = CvBridge()
        self.tf = Transform()
        self.alg = Algorithm()
        self.compute = Compute()

        # UI
        self.ui = CommandUI()

        self.control_service = control_service

        self.iiwa_lin_pose_pub = rospy.Publisher(iiwa_lin_pub_topic, PoseStamped, queue_size=10)

        self.probe_path = probe_path
        self.probe_to_ee = self.compute.get_coordinate(project_path + probe_path)
        self.cali_res = self.compute.get_calibration_res(project_path + result_path)
        self.cali_res[:3, 3] = self.cali_res[:3, 3] * 1000
        self.cali_res = np.round(self.cali_res, decimals=3)

        ## initialize parameters
        self.init_parameters()
        self.command_str = None
        self.robot_pose = None
        self.robot_pose_7d = []
        self.init_pose = np.array([0.5, 0.0, 0.6, 0.7071068, 0.7071068, 0.0, 0.0])
        
        try:
            rospy.Subscriber(robot_topic, CartesianPose, self.rob_callback)
            rospy.wait_for_message(robot_topic, CartesianPose, timeout=1)
        except rospy.exceptions.ROSException as e:
            print("No robot topic. Please check it.")

        # Camera image detect
        self.img = None
        self.depth_img = None
        try:
            rospy.Subscriber(img_topic, Image, self.img_callback)
        except rospy.exceptions.ROSException as e:
            print("There is no rgb image topic. Please check it.")
        
        try:
            rospy.Subscriber(depth_img_topic, Image, self.depth_img_callback)
        except rospy.exceptions.ROSException as e:
            print("There is no depth image topic. Please check it.")

        ## Robot command
        self.command_str = ''
        try:
            rospy.Subscriber(rob_command_topic, String, self.rob_command_callback)
        except rospy.exceptions.ROSException as e:
            print("There is no ultrasound image topic. Please check it.") 

        self.command_timer = QTimer()
        self.command_timer.timeout.connect(self.auto_execuate_command)
        self.command_timer.start(10)

        self.trajectory_timer = QTimer()
        self.trajectory_timer.timeout.connect(self.motion)
        self.trajectory_timer.start(500)

        self.generate_timer = QTimer()
        self.generate_timer.timeout.connect(self.generate_path)
        self.generate_timer.start(10)
    
    def generate_path(self):
        if not self.ui.is_gen:
            return
        self.execute_command('generate')
        self.ui.is_gen = False

    def rob_command_callback(self, ros_msg):
        self.command_str = ros_msg.data
        self.automated_command = True

    def onclick(self, event):
        if event.xdata is None or event.ydata is None:
            return
        self.points_selection.append((event.xdata, event.ydata))
        print(f"Point selected: ({event.xdata:.5f}, {event.ydata:.5f})")
        plt.plot(event.xdata, event.ydata, color='red', marker='o', linewidth=1, markersize=2)
        plt.draw()
    
    def motion(self):
        if not self.is_motion_start:
            return
        
        if len(self.points_running_queue) == 0:
            self.is_motion_start = False
            self.end_scanning()
            return
        
        point_to_world_quat = self.points_running_queue[0]

        if not self.is_probe_pose_closed(point_to_world_quat):
            self.desired_pose_mouse_quat = point_to_world_quat
            desired_pose_angle = self.tf.quat_to_angle(self.desired_pose_mouse_quat, True)
            desired_pose_angle = np.round(desired_pose_angle, decimals=2)
            self.move_probe_to_point(point_to_world_quat)
        else:
            self.points_running_queue.popleft()
    
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
    
    def auto_execuate_command(self):
        if not self.automated_command:
            return

        print(self.command_str)
        self.execute_command(self.command_str)

        self.automated_command = False
    
    def execute_command(self, value):
        if value == 'start':
            if len(self.points_quat) == 0:
                return
            self.move_probe_to_point(self.points_quat[0])

        elif value == 'trajectory':
            point_to_world = self.tf.quat_to_matrix(self.points_quat[0])
            desired_pose = point_to_world @ np.linalg.inv(self.probe_to_ee)
            desired_pose_quat = self.tf.matrix_to_quat(desired_pose)
            if np.sum(np.abs(self.robot_pose_7d[:3] - desired_pose_quat[:3])) > 0.1:
                return
            
            self.set_impedance_control(enable=True, is_impedance=True)
            self.is_segmented = True
            self.movement(self.points_quat)
        
        elif value == 'press':
            if len(self.points_running_queue) == 0 or self.is_robot_stopped:
                current_probe_pose = self.robot_pose_7d.copy()
                current_probe_pose[2] -= 0.001
                self.move_to_cartesian_pose(current_probe_pose)
            else:
                points_array = np.array(self.points_running_queue)
                points_array[:, 2] -= 0.001
                self.points_running_queue = deque(points_array)
        
        elif value == 'release':
            if len(self.points_running_queue) == 0 or self.is_robot_stopped:
                current_probe_pose = self.robot_pose_7d.copy()
                current_probe_pose[2] += 0.015
                self.move_to_cartesian_pose(current_probe_pose)
            else:
                points_array = np.array(self.points_running_queue)
                points_array[:, 2] += 0.001
                self.points_running_queue = deque(points_array)

        elif value == 'pause':
            current_probe_pose = self.robot_pose_7d.copy()
            current_probe_pose[2] += 0.01
            self.is_motion_start = False
            self.move_to_cartesian_pose(current_probe_pose)
            self.desired_pose_mouse_quat = self.robot_pose_7d
        
        elif value == 'resume':
            point_list = self.points_running_queue.copy()
            self.is_motion_start = True
            self.movement(point_list, is_trajectory=False)

        elif value == 'stop':
            current_probe_pose = self.robot_pose_7d.copy()
            current_probe_pose[2] += 0.02
            self.move_to_cartesian_pose(current_probe_pose)
            self.desired_pose_mouse_quat = self.robot_pose_7d

        elif value == 'select':
            img = self.img.copy()
            self.init_parameters()
            fig, ax = plt.subplots()
            ax.imshow(img)
            self.points_selection = []
            cid = fig.canvas.mpl_connect('button_press_event', self.onclick)
            plt.show()
        
        elif value == 'reset':
            self.end_scanning()
        
        elif value == 'generate':
            self.gen_trajectory()
            print("Points: \n", self.points_quat)
        
        elif value == 'none':
            pass

        else:
            print('Please check the command!')
        
    def gen_trajectory(self):
        if len(self.points_selection) < 2:
            print("Check 2d points!")
            return
        idx = 1
        for point2d in self.points_selection:
            point_to_world = self.get_point_mat(point2d, idx)

            # Publish has the different coordinate system
            if len(point_to_world) == 0:
                continue
            point_to_world_quat = self.tf.matrix_to_quat(point_to_world)
            self.points_quat.append(point_to_world_quat)
            idx += 1
    
    def movement(self, point_list, is_trajectory=True):
        if len(self.points_quat) < 2:
            return
        if is_trajectory:
            point_array = np.array(point_list)
            point_array[:, 2] += 0.003
            self.points_running_queue = deque(point_list)
            self.points_running_queue.popleft()

        self.is_motion_start = True
                
    def end_scanning(self):
        self.set_impedance_control(enable=True, is_impedance=False)
        self.desired_pose_mouse_quat = self.init_pose
        self.move_to_cartesian_pose(self.init_pose)

    def is_probe_pose_closed(self, point_q):
        return np.sum(np.abs(self.get_probe_to_base_quat() - point_q)) < 0.018
    
    def get_probe_to_base_quat(self):
        start_mat = self.tf.quat_to_matrix(self.robot_pose_7d)
        probe_to_base = np.dot(start_mat, self.probe_to_ee)
        probe_to_base_quat = self.tf.matrix_to_quat(probe_to_base)
        return probe_to_base_quat
    
    def get_point_mat(self, point2d, idx=0, is_published=False):
        point3d, point_rot = self.single_point_calculation(point2d, is_published=is_published)
        point_to_world = []
        if len(point3d) == 0:
            text = f'Point {idx}\'s depth detection failed!!!'
            print('3D point get failed!')
            return point_to_world
        self.points_trans.append(point3d)
        self.points_rot_mat.append(point_rot)

        point_to_world = np.identity(4)
        point_to_world[:3, 3] = point3d / 1000
        point_to_world[:3, :3] = point_rot
        return point_to_world
        
    def single_point_calculation(self, point2d, is_published=False, max_rot_angle=20):
        point3d = []
        point_rot = []
        point3d_cam = self.alg.pixel_to_point(point2d, self.depth_queue)
        if point3d_cam[2] == 0.0:
            return point3d, point_rot
        
        # Get the 3D neighbor points in a 2D point and a depth_img, then transform them to the world coordinate to get rotation.
        neighbor_points3d = self.alg.extract_neighborhood_points(np.round(np.array(point2d)).astype(int), self.depth_queue)
        ee_to_base = self.tf.quat_to_matrix(self.robot_pose_7d)
        ee_to_base[:3, 3] = ee_to_base[:3, 3] * 1000
        if is_published:
            cam_to_base = np.identity(4)
        else:
            cam_to_base = ee_to_base @ self.cali_res
        point3d = self.alg.transform_points(cam_to_base, point3d_cam)
        
        neighbor_points_ee = self.alg.transform_points(self.cali_res, neighbor_points3d)
        point_rot_t, is_succeed = self.alg.calculate_rot_mat(neighbor_points_ee, ee_to_base[:3, :3])
        if not is_succeed:
            return point3d, point_rot
        angle = euler.mat2euler(point_rot_t)
        point_rot_t = euler.euler2mat(angle[1], angle[0], angle[2])
        point_rot = point_rot_t @ ee_to_base[:3, :3] 

        point_rot_angle = np.rad2deg(euler.mat2euler(point_rot_t))
        point_rot_angle[point_rot_angle > max_rot_angle] = max_rot_angle

        return point3d, point_rot
        
    def move_probe_to_point(self, point_to_world_quat):
        point_to_world = self.tf.quat_to_matrix(point_to_world_quat)
        desired_pose = point_to_world @ np.linalg.inv(self.probe_to_ee)
        
        desired_pose_show = self.tf.matrix_to_angle(desired_pose, unit_m_to_mm=True)
        desired_pose_show = np.round(desired_pose_show, decimals=2)
        transform_quat = self.tf.matrix_to_quat(desired_pose)

        self.move_to_cartesian_pose(transform_quat)
    
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

    def img_callback(self, ros_img):
        self.img = self.cv_bridge.imgmsg_to_cv2(ros_img, "passthrough")
    
    def depth_img_callback(self, ros_img):
        depth_img = self.cv_bridge.imgmsg_to_cv2(ros_img, "passthrough")
        self.depth_img = depth_img
        self.depth_queue.append(depth_img)
    
    def move_to_cartesian_pose(self, desired_pose):
        posemsg = PoseStamped()
        posemsg.header.frame_id = "iiwa_link_0"

        posemsg.pose.position.x = desired_pose[0]
        posemsg.pose.position.y = desired_pose[1]
        posemsg.pose.position.z = desired_pose[2]
        posemsg.pose.orientation.x = desired_pose[3]
        posemsg.pose.orientation.y = desired_pose[4]
        posemsg.pose.orientation.z = desired_pose[5]
        posemsg.pose.orientation.w = desired_pose[6]

        self.iiwa_lin_pose_pub.publish(posemsg)
     
    def init_parameters(self):
        self.points_selection = []
        self.points_trans = []
        self.points_rot_mat = []
        self.points_quat = []
        self.depth_queue = deque(maxlen=10)

        self.is_motion_start = False

        self.is_robot_stopped = True
        self.desired_pose_mouse_quat = None

        self.automated_command = False
        self.points_running_queue = deque()

if __name__ == '__main__':
    QCoreApplication.setAttribute(Qt.AA_ShareOpenGLContexts)
    app = QApplication([])
    project_path = path.dirname(path.dirname(__file__))
    np.set_printoptions(precision=6, suppress=True)
    visualization = ControlPanel(project_path)
    visualization.ui.show()
    sys.exit(app.exec())