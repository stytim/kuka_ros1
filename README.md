roscore
roslaunch ros_tcp_endpoint endpoint.launch
sudo modprobe v4l2loopback
scrcpy --v4l2-sink=/dev/video0 -d --no-video-playback -m2000 --crop 835:800:150:600
rosrun image_pub image_publisher.py
roslaunch iiwa_pose_control iiwa_realsense.launch


rosrun iiwa_pose_control iiwa_pose_control.py
