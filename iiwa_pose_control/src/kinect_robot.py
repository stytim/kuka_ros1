#!/usr/bin/env python

import rospy
import tf
import tf2_ros
from apriltag_ros.msg import AprilTagDetectionArray
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseStamped, TransformStamped

class BodyTrackingTransformer:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('body_tracking_transformer', anonymous=True)

        # Subscribe to the tag detections and body tracking data topics
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)
        rospy.Subscriber('/body_tracking_data', MarkerArray, self.body_tracking_callback)

        # Publisher for transformed body tracking data
        self.transformed_pub = rospy.Publisher('/transformed_body_tracking_data', MarkerArray, queue_size=10)

        # # TF listener and broadcaster
        self.tf_listener = tf.TransformListener()
        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster()

        # Store the latest tag detection
        self.latest_tag_pose = None
        self.tag_frame_id = "apriltag_frame"
        self.robot_base_frame_id = "world"


    def tag_callback(self, data):
        # Process the tag detection
        if data.detections:
            tag_pose = data.detections[0].pose.pose.pose
            self.latest_tag_pose = tag_pose

            # Broadcast the new AprilTag frame
            self.broadcast_static_tag_frame()

            self.broadcast_tag_robot_base_transform()

            rospy.loginfo("Tag detected with ID: {}".format(data.detections[0].id[0]))

            # log pose of the tag
            rospy.loginfo("Tag pose: x={}, y={}, z={}".format(tag_pose.position.x, tag_pose.position.y, tag_pose.position.z))
            rospy.loginfo("Tag orientation: x={}, y={}, z={}, w={}".format(tag_pose.orientation.x, tag_pose.orientation.y, tag_pose.orientation.z, tag_pose.orientation.w))

    def broadcast_static_tag_frame(self):
        if self.latest_tag_pose is None:
            return

        # Create a TransformStamped object
        static_transform_stamped = TransformStamped()

        static_transform_stamped.header.stamp = rospy.Time.now()
        static_transform_stamped.header.frame_id = "rgb_camera_link"
        static_transform_stamped.child_frame_id = self.tag_frame_id

        static_transform_stamped.transform.translation.x = self.latest_tag_pose.position.x
        static_transform_stamped.transform.translation.y = self.latest_tag_pose.position.y
        static_transform_stamped.transform.translation.z = self.latest_tag_pose.position.z
        static_transform_stamped.transform.rotation.x = self.latest_tag_pose.orientation.x
        static_transform_stamped.transform.rotation.y = self.latest_tag_pose.orientation.y
        static_transform_stamped.transform.rotation.z = self.latest_tag_pose.orientation.z
        static_transform_stamped.transform.rotation.w = self.latest_tag_pose.orientation.w


        # Broadcast the transform
        self.tf_static_broadcaster.sendTransform(static_transform_stamped) 

    def broadcast_tag_robot_base_transform(self):
            if self.latest_tag_pose is None:
                return

            # Create the static transform message with random offset
            static_transform_stamped = TransformStamped()

            static_transform_stamped.header.stamp = rospy.Time.now()
            static_transform_stamped.header.frame_id = self.tag_frame_id
            static_transform_stamped.child_frame_id = self.robot_base_frame_id

            static_transform_stamped.transform.translation.x = 0.25
            static_transform_stamped.transform.translation.y = 0.10
            static_transform_stamped.transform.translation.z = 0.002
            static_transform_stamped.transform.rotation.x = 0
            static_transform_stamped.transform.rotation.y = 0
            static_transform_stamped.transform.rotation.z = 1
            static_transform_stamped.transform.rotation.w = 0

            # Broadcast the static transform with the offset
            self.tf_static_broadcaster.sendTransform(static_transform_stamped)


    def body_tracking_callback(self, data):
        if self.latest_tag_pose is None:
            rospy.logwarn("No tag detected yet, skipping transformation")
            return

        transformed_markers = MarkerArray()

        for marker in data.markers:
            transformed_marker = Marker()
            transformed_marker = marker  # Copy all data from the original marker


            # Transform the pose of each marker to the tag frame
            try:
                # Create a PoseStamped object for the body tracking pose
                pose_stamped = PoseStamped()
                pose_stamped.header = marker.header
                pose_stamped.pose = marker.pose

                # Transform the pose to the tag frame
                transformed_pose = self.tf_listener.transformPose(self.tag_frame_id, pose_stamped)

                # Now, transform the pose to the robot base frame
                transformed_pose_robot = self.tf_listener.transformPose(self.robot_base_frame_id, transformed_pose)

                # Update the marker's pose with the transformed pose
                transformed_marker.pose = transformed_pose_robot.pose
                transformed_marker.header.frame_id = self.robot_base_frame_id

                # Append the transformed marker to the new MarkerArray
                transformed_markers.markers.append(transformed_marker)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logerr("TF transformation failed: {}".format(e))
                continue

        # Publish the transformed body tracking data
        self.transformed_pub.publish(transformed_markers)

if __name__ == '__main__':
    try:
        BodyTrackingTransformer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
