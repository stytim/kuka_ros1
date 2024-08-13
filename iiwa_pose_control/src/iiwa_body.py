#!/usr/bin/env python

import rospy
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped
from iiwa_msgs.srv import ConfigureControlMode, ConfigureControlModeRequest
from iiwa_msgs.msg import DesiredForceControlMode
import sys
import select
import tty
import termios

class BodyTrackingNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('body_tracking_node', anonymous=True)


        # Subscribe to the /transformed_body_tracking_data topic
        self.sub = rospy.Subscriber('/transformed_body_tracking_data', MarkerArray, self.marker_callback)

        # Publisher for the /iiwa/command/CartesianPoseLin topic
        self.pub = rospy.Publisher('/iiwa/command/CartesianPoseLin', PoseStamped, queue_size=10)


        # Set the desired force control mode
        self.set_desired_force()


        # Store the current pose to publish when 'm' is pressed
        self.current_pose = None

        # Set up for keyboard input detection
        self.settings = termios.tcgetattr(sys.stdin)

    def set_desired_force(self):
        rospy.wait_for_service('/iiwa/configuration/ConfigureControlMode')
        rospy.loginfo("Setting up desired force")
        try:
            configure_control_mode = rospy.ServiceProxy('/iiwa/configuration/ConfigureControlMode', ConfigureControlMode)
            request = ConfigureControlModeRequest()

            request.control_mode = 3  # Assuming 3 corresponds to force control mode
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

    def marker_callback(self, msg):
        # Iterate through the markers to find the one with joint_index = 8 and body_id = 1
        for marker in msg.markers:
            joint_index = marker.id % 100
            body_id = marker.id // 100
            if joint_index == 8 and body_id == 1:
                # Store the pose of the joint
                self.current_pose = PoseStamped()
                self.current_pose.header.stamp = rospy.Time.now()
                self.current_pose.header.frame_id = "iiwa_link_0"  # Adjust the frame ID as needed
                self.current_pose.pose.position = marker.pose.position
                self.current_pose.pose.orientation.x = -0.7071068
                self.current_pose.pose.orientation.y = 0.7071068
                self.current_pose.pose.orientation.z = 0
                self.current_pose.pose.orientation.w = 0

                # rospy.loginfo(f"Pose stored for body_id: {body_id}, joint_index: {joint_index}")
                # rospy.loginfo(f"Pose: x={marker.pose.position.x}, y={marker.pose.position.y}, z={marker.pose.position.z}")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        rospy.loginfo("Waiting for 'm' key press to publish pose...")
        while not rospy.is_shutdown():
            key = self.get_key()
            if key == 'm' and self.current_pose:
                # Publish the current stored pose
                self.pub.publish(self.current_pose)

                # print pose information
                rospy.loginfo(f"Pose: x={self.current_pose.pose.position.x}, y={self.current_pose.pose.position.y}, z={self.current_pose.pose.position.z}")
            elif key == '\x03':  # Ctrl+C
                break

if __name__ == '__main__':
    try:
        node = BodyTrackingNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
