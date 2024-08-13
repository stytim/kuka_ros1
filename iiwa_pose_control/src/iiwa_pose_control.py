#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from iiwa_msgs.msg import CartesianPose, DesiredForceControlMode
from iiwa_msgs.srv import ConfigureControlMode,ConfigureControlModeRequest, ConfigureControlModeResponse
import sys
import select
import termios
import tty

stored_pose = None
current_pose = None

def get_key():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def pose_callback(msg):
    global current_pose
    current_pose = msg.poseStamped


def set_desired_force():
    rospy.wait_for_service('/iiwa/configuration/ConfigureControlMode')
    rospy.loginfo("Setting up desired force")
    try:
        configure_control_mode = rospy.ServiceProxy('/iiwa/configuration/ConfigureControlMode', ConfigureControlMode)
        request = ConfigureControlModeRequest()

        request.control_mode = 3  
        request.desired_force = DesiredForceControlMode()
        request.desired_force.cartesian_dof = 3  # Set the Cartesian degree of freedom
        request.desired_force.desired_force = 5.0  # Desired force in N
        request.desired_force.desired_stiffness = 500.0  # Desired stiffness in N/m

        response = configure_control_mode(request)

        if response.success:
            rospy.loginfo("Desired force control mode configured successfully.")
        else:
            rospy.logerr(f"Failed to configure desired force control mode: {response.error}")

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def set_position_control():
    rospy.wait_for_service('/iiwa/configuration/ConfigureControlMode')
    rospy.loginfo("Setting up position control")
    try:
        configure_control_mode = rospy.ServiceProxy('/iiwa/configuration/ConfigureControlMode', ConfigureControlMode)
        request = ConfigureControlModeRequest()

        request.control_mode = 0

        response = configure_control_mode(request)

        if response.success:
            rospy.loginfo("Position control mode configured successfully.")
        else:
            rospy.logerr(f"Failed to configure position control mode: {response.error}")

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def main():
    global settings, stored_pose, current_pose
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('iiwa_pose_control', anonymous=True)
    rospy.Subscriber('/iiwa/state/CartesianPose', CartesianPose, pose_callback)
    pose_pub = rospy.Publisher('/iiwa/command/CartesianPoseLin', PoseStamped, queue_size=10)

    set_desired_force()

    rospy.loginfo("Press 's' to store the current pose. Press 'm' to publish the stored pose. 'q' to quit.")

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        key = get_key()
        if key == 's':
            if current_pose:
                stored_pose = current_pose
                rospy.loginfo("Pose stored.")
            else:
                rospy.logwarn("No pose available to store.")
        elif key == 'm':
            if stored_pose:
                set_desired_force()
                pose_pub.publish(stored_pose)
                rospy.loginfo("Stored pose published.")
            else:
                rospy.logwarn("No pose stored to publish.")
        elif key == 'q':
            rospy.loginfo("Quitting program.")
            break
        rate.sleep()

    set_position_control()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
