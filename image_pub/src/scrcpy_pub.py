#!/usr/bin/env python

# scrcpy -d --video-codec=h264 --video-bit-rate=2M --tcpip --record-format=mkv --record - | ffmpeg -i - -f rawvideo -pix_fmt bgr24 - 2>/dev/null

import rospy
import cv2
import numpy as np
import subprocess
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

def main():
    rospy.init_node('image_publisher', anonymous=True)
    image_pub = rospy.Publisher('/ultrasound_image/compressed', CompressedImage, queue_size=1)
    bridge = CvBridge()

    # FFmpeg Command to Capture from scrcpy
    ffmpeg_cmd = [
        "ffmpeg", "-i", "-", "-f", "rawvideo", "-pix_fmt", "bgr24", "-"
    ]
    
    # Start FFmpeg Process
    ffmpeg_process = subprocess.Popen(ffmpeg_cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, bufsize=10**8)

    rospy.loginfo("Streaming image from scrcpy...")

    rate = rospy.Rate(20)  # 20 Hz
    while not rospy.is_shutdown():
        raw_frame = ffmpeg_process.stdout.read(835 * 800 * 3)  # (Width * Height * 3 for RGB)
        if not raw_frame:
            rospy.logerr("Failed to read frame")
            break
        
        frame = np.frombuffer(raw_frame, np.uint8).reshape((800, 835, 3))
        msg = bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpg')
        image_pub.publish(msg)

        rate.sleep()

    ffmpeg_process.terminate()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass