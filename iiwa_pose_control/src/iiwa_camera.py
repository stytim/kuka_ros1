#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import pyrealsense2 as rs
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Bool
from tf.transformations import quaternion_matrix

# Camera Intrinsics (to be updated from RealSense)
camera_intrinsics = None

# Transformation Matrix from Camera to End Effector (to be provided)
T_cam2ee = np.eye(4)  # Update with actual transformation

# Transformation Matrix from End Effector to Robot Base (to be provided)
T_ee2base = np.eye(4)  # Update with actual transformation

# Global variables
clicked_points = []
depth_frame = None
color_frame = None
depth_intrinsics = None

# Mouse callback function to capture 2D points
def click_event(event, x, y, flags, param):
    global clicked_points
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked_points.append((x, y))
        if len(clicked_points) == 2:
            cv2.destroyAllWindows()

def capture_image():
    global depth_frame, color_frame, depth_intrinsics
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
    profile = pipeline.start(config)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    # Get camera intrinsics
    depth_stream = profile.get_stream(rs.stream.depth).as_video_stream_profile()
    depth_intrinsics = depth_stream.get_intrinsics()
    
    rospy.sleep(2)  # Allow camera to stabilize
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    if not depth_frame or not color_frame:
        rospy.logerr("Failed to capture frames")
        return None, None

    color_image = np.asanyarray(color_frame.get_data())
    pipeline.stop()
    return color_image, depth_frame

# Function to get 3D point from 2D pixel
def get_3d_point(x, y, depth_frame, depth_intrinsics):
    depth = depth_frame.get_distance(x, y)
    if depth == 0:
        rospy.logwarn(f"Depth at ({x}, {y}) is zero!")
        return None
    point_3d = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [x, y], depth)
    return np.array([point_3d[0], point_3d[1], point_3d[2], 1])  # Homogeneous coordinates

# Transform point from camera frame to robot base frame
def transform_to_base(point_cam, T_cam2ee, T_ee2base):
    point_ee = np.dot(T_cam2ee, point_cam)
    point_base = np.dot(T_ee2base, point_ee)
    return point_base[:3]

# Main function
def main():
    global clicked_points
    rospy.init_node('realsense_arm_tracker', anonymous=True)
    pose_pub = rospy.Publisher('/iiwa/command/CartesianPoseLin', PoseStamped, queue_size=10)
    reached_pub = rospy.Publisher('/final_pose_reached', Bool, queue_size=10)
    
    rospy.loginfo("Capturing image from RealSense camera...")
    color_image, depth_frame = capture_image()
    if color_image is None:
        return
    
    cv2.imshow('Select Start and End Points', color_image)
    cv2.setMouseCallback('Select Start and End Points', click_event)
    cv2.waitKey(0)
    
    if len(clicked_points) != 2:
        rospy.logerr("Two points must be selected.")
        return
    
    start_2d, end_2d = clicked_points
    rospy.loginfo(f"Selected Points: Start: {start_2d}, End: {end_2d}")
    
    start_3d = get_3d_point(*start_2d, depth_frame, depth_intrinsics)
    end_3d = get_3d_point(*end_2d, depth_frame, depth_intrinsics)
    
    if start_3d is None or end_3d is None:
        rospy.logerr("Failed to obtain 3D points.")
        return
    
    rospy.loginfo(f"3D Camera Frame: Start: {start_3d}, End: {end_3d}")
    
    start_base = transform_to_base(start_3d, T_cam2ee, T_ee2base)
    end_base = transform_to_base(end_3d, T_cam2ee, T_ee2base)
    
    rospy.loginfo(f"3D Robot Base Frame: Start: {start_base}, End: {end_base}")
    
    start_pose = PoseStamped()
    start_pose.pose.position.x = start_base[0]
    start_pose.pose.position.y = start_base[1]
    start_pose.pose.position.z = start_base[2]
    
    end_pose = PoseStamped()
    end_pose.pose.position.x = end_base[0]
    end_pose.pose.position.y = end_base[1]
    end_pose.pose.position.z = end_base[2]
    
    rospy.loginfo("Moving to start pose...")
    pose_pub.publish(start_pose)
    rospy.sleep(2)
    
    rospy.loginfo("Moving to end pose...")
    pose_pub.publish(end_pose)
    rospy.sleep(2)
    
    reached_pub.publish(Bool(data=True))
    rospy.loginfo("Movement complete.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
