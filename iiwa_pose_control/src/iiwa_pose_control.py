#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Time, Bool
from iiwa_msgs.msg import CartesianPose, DesiredForceControlMode
from iiwa_msgs.srv import ConfigureControlMode,ConfigureControlModeRequest, ConfigureControlModeResponse
import sys
import select
import termios
import tty
import copy

stored_pose1 = None
stored_pose2 = None
current_pose = None
destination_reached = False

def get_key():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def pose_callback(msg):
    global current_pose
    current_pose = msg.poseStamped

def destination_reached_callback(msg):
    global destination_reached
    destination_reached = True

def start_signal_callback(msg):
    if msg.data:
        execute_movement_sequence(pose_pub)

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

def move_to_pose(pose_pub, pose):
    global destination_reached
    destination_reached = False
    pose_pub.publish(pose)
    rospy.loginfo("Moving to pose.")
    while not destination_reached:
        rospy.sleep(0.5)
    rospy.loginfo("Reached pose.")

def execute_movement_sequence(pose_pub):
    set_desired_force()
    rospy.sleep(0.5)
    if stored_pose1 and stored_pose2:
        move_to_pose(pose_pub, stored_pose1)
        move_to_pose(pose_pub, stored_pose2)
        # Move 0.1m up in the z direction

        rospy.loginfo("Moving to Final Pose.")

        # Create a new PoseStamped message with the final pose, copy data from pose2
        final_pose = copy.deepcopy(stored_pose2)
        final_pose.pose.position.z += 0.15
        rospy.loginfo("Final pose: x={}, y={}, z={}".format(final_pose.pose.position.x, final_pose.pose.position.y, final_pose.pose.position.z))
        #final_pose = 3
        move_to_pose(pose_pub, final_pose)
        reached_pub.publish(Bool(data=True))
    else:
        rospy.logwarn("Both poses must be stored before moving.")

def main():
    global settings, stored_pose1, stored_pose2, current_pose, destination_reached, pose_pub, stored_pose_pub, reached_pub
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('iiwa_pose_control', anonymous=True)
    rospy.Subscriber('/iiwa/state/CartesianPose', CartesianPose, pose_callback)
    rospy.Subscriber('/iiwa/state/DestinationReached', Time, destination_reached_callback)
    rospy.Subscriber('/start_signal', Bool, start_signal_callback)
    pose_pub = rospy.Publisher('/iiwa/command/CartesianPoseLin', PoseStamped, queue_size=10)
    stored_pose_pub = rospy.Publisher('/pose_storage_signal', String, queue_size=10)  # Publisher for pose storage signal
    reached_pub = rospy.Publisher('/final_pose_reached', Bool, queue_size=10)  # Publisher for final pose reached signal

    rospy.loginfo("Press 's' to store the current pose. Press 'm' to publish the stored pose. 'q' to quit.")

    rate = rospy.Rate(30)  # 30 Hz
    while not rospy.is_shutdown():
        key = get_key()
        if key == 's':
            if not stored_pose1 and current_pose:
                stored_pose1 = current_pose
                rospy.loginfo("First pose stored.")
                #rospy.loginfo("Pose: x={}, y={}, z={}".format(stored_pose1.pose.position.x, stored_pose1.pose.position.y, stored_pose1.pose.position.z))
                stored_pose_pub.publish(String(data="First"))  # Publish signal for first pose
            elif not stored_pose2 and current_pose:
                stored_pose2 = current_pose
                rospy.loginfo("Second pose stored.")
                #rospy.loginfo("Pose: x={}, y={}, z={}".format(stored_pose2.pose.position.x, stored_pose2.pose.position.y, stored_pose2.pose.position.z))
                stored_pose_pub.publish(String(data="Second"))  # Publish signal for second pose
            else:
                rospy.logwarn("Both poses are already stored.")
        elif key == 'm':
            execute_movement_sequence(pose_pub)
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
