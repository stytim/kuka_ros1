#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@File    :   us_screen_pub.py
@Time    :   2024-07-24 16:33:19
@Author  :   feng li
@Contact :   feng.li@tum.de
@Description    :   
'''

import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

import cv2
import yaml
from os import path

if __name__ == '__main__':
    # ros initialization
    rospy.init_node('us_screen_pub_node',anonymous=True)
    bridgeC = CvBridge()
    pub_us_img = rospy.Publisher("/us_image/compressed", CompressedImage, queue_size=1)

    # loading yaml files for reading vedio straming cropping screen
    project_path = path.dirname(path.dirname(__file__))
    file_path = path.join(project_path, 'config', 'screen_cap_config_linear.yaml')
    with open(file_path, 'r') as f:
        capture_config = yaml.load(f.read(), Loader=yaml.FullLoader)
    
    # capture screen
    screen_cap = cv2.VideoCapture(capture_config["video_index"]) # Add cv2.CAP_V4L2 to solve the problem that can't capture the video.
    screen_cap.set(cv2.CAP_PROP_FRAME_WIDTH,5000)
    screen_cap.set(cv2.CAP_PROP_FRAME_HEIGHT,5000)
    if not screen_cap.isOpened():
        print("Screen haven't been captured!!!")

    # cropped parameters
    x0 = capture_config["frame_cropped_coordinates"]["x0"]
    x1 = capture_config["frame_cropped_coordinates"]["x1"]
    y0 = capture_config["frame_cropped_coordinates"]["y0"]
    y1 = capture_config["frame_cropped_coordinates"]["y1"]

    # start publishing cropped images
    last_img = None

    frequence = capture_config["hz"]
    loop_rate = rospy.Rate(frequence)
    while not rospy.is_shutdown():
        if not screen_cap.isOpened(): 
            continue
        ret, frame = screen_cap.read()
        if ret is True:
            cropped = frame[y0:y1, x0:x1]

            # check for the first time
            if last_img is None:
                last_img = cropped
                continue

            # msg_img = bridgeC.cv2_to_imgmsg(cropped)
            # msg_img.header.stamp = rospy.Time.now()
            # msg_img.header.frame_id = "ultrasound_frame"
            msg_img = bridgeC.cv2_to_compressed_imgmsg(cropped, dst_format='jpg')
            pub_us_img.publish(msg_img)
            last_img = cropped
        else:
            rospy.loginfo("not valid!")
        loop_rate.sleep()
    screen_cap.release()