#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from cartographer_ros_msgs.msg import LandmarkList, LandmarkEntry


class LandmarkPublisher:
    """Simple dummy landmark publisher"""

    def __init__(self):
        rospy.init_node("landmark_publisher")

        self.landmark_frame = "base_link"

        self.landmark_list = LandmarkList()


        self.landmark_poses = PoseArray()
        self.landmark_poses.header.frame_id = "odom"

        pose_1 = Pose()
        pose_1.position.x = 2.0
        pose_1.position.y = 1.0
        pose_1.position.z = 1.0
        pose_1.orientation.w = 1.0
        pose_1.orientation.x = 0.0
        pose_1.orientation.y = 0.0
        pose_1.orientation.z = 0.0
        self.landmark_poses.poses.append(pose_1)

        pose_2 = Pose()
        pose_2.position.x = 2.0
        pose_2.position.y = -1.0
        pose_2.position.z = 1.0
        pose_2.orientation.w = 1.0
        pose_2.orientation.x = 0.0
        pose_2.orientation.y = 0.0
        pose_2.orientation.z = 0.0
        self.landmark_poses.poses.append(pose_2)

        pose_3 = Pose()
        pose_3.position.x = -2.0
        pose_3.position.y = 0.0
        pose_3.position.z = 1.0
        pose_3.orientation.w = 1.0
        pose_3.orientation.x = 0.0
        pose_3.orientation.y = 0.0
        pose_3.orientation.z = 0.0
        self.landmark_poses.poses.append(pose_3)


        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        publish_rate = rospy.get_param("~frequency", 10)
        self.landmark_pub = rospy.Publisher("landmark", LandmarkList, queue_size=1)
        self.poses_pub = rospy.Publisher("poses", PoseArray, queue_size=1)
        self.publish_timer = rospy.Timer(rospy.Duration(1.0 / publish_rate), self.publish_list)


    def get_relative_pose(self, pose_stamped):
        trans = self.tf_buffer.lookup_transform(
            self.landmark_frame, pose_stamped.header.frame_id, rospy.Time()
        )
        pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, trans)
        return pose_transformed

    def publish_list(self, event):
        del event

        self.landmark_list.header.stamp = rospy.Time.now()
        self.landmark_list.header.frame_id = self.landmark_frame
        self.landmark_list.landmarks = list()

        for index, pose in enumerate(self.landmark_poses.poses):
            pose_trans = self.get_relative_pose(PoseStamped(header=self.landmark_poses.header, pose=pose))
            entry = LandmarkEntry()
            entry.id = str(index)
            entry.tracking_from_landmark_transform.position.x = pose_trans.pose.position.x
            entry.tracking_from_landmark_transform.position.y = pose_trans.pose.position.y
            entry.tracking_from_landmark_transform.position.z = pose_trans.pose.position.z
            entry.tracking_from_landmark_transform.orientation.x = pose_trans.pose.orientation.x
            entry.tracking_from_landmark_transform.orientation.y = pose_trans.pose.orientation.y
            entry.tracking_from_landmark_transform.orientation.z = pose_trans.pose.orientation.z
            entry.tracking_from_landmark_transform.orientation.w = pose_trans.pose.orientation.w
            entry.translation_weight = 1e8
            entry.rotation_weight = 0.0
            self.landmark_list.landmarks.append(entry)
        self.landmark_pub.publish(self.landmark_list)
        self.poses_pub.publish(self.landmark_poses)


if __name__ == "__main__":
    pub = LandmarkPublisher()

    rospy.spin()
