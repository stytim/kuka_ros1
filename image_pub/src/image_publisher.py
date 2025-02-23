#!/usr/bin/env python
# scrcpy --v4l2-sink=/dev/video0 -d --no-video-playback -m2000 --crop 835:800:150:600

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
import sys
import select
import termios
import tty

def get_key():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    global settings
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('image_publisher', anonymous=True)
    image_pub = rospy.Publisher('/ultrasound_image/compressed', CompressedImage, queue_size=1)
    bridge = CvBridge()

    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        rospy.logerr("Cannot open image stream")
        return

    rate = rospy.Rate(20)  # 20 Hz

    print("Press 'q' to quit.")
    while not rospy.is_shutdown():
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.read(1)
            if key == 'q':
                rospy.loginfo("Quitting program.")
                break

        ret, frame = cap.read()
        if not ret:
            rospy.logerr("Cannot read from image stream")
            break

        # Convert the frame to a ROS CompressedImage message
        msg = bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpg')
        image_pub.publish(msg)

        rate.sleep()

    cap.release()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
