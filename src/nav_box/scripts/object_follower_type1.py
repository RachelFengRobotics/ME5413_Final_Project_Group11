#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
import tf
from math import cos, sin

class ObjectFollower:
    def __init__(self):
        rospy.init_node('object_follower_type1')

        self.odom_sub = rospy.Subscriber('/gazebo/ground_truth/state', Odometry, self.odom_callback)
        self.obj_sub = rospy.Subscriber('/objects', Float32MultiArray, self.obj_callback)

        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

        self.current_pose = None
        self.last_published_time = rospy.Time.now()
        self.last_detected_time = rospy.Time.now()  # 上次检测到物体的时间
        self.last_detected_pose = None  # 上次检测到物体时的位置和姿态

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def obj_callback(self, msg):
        current_time = rospy.Time.now()
        if len(msg.data) > 0:
            self.last_detected_time = current_time
            self.last_detected_pose = self.current_pose
            if (current_time - self.last_published_time).to_sec() >= 2.0:
                self.publish_goal()
                self.last_published_time = current_time
        elif (current_time - self.last_detected_time).to_sec() >= 10.0 and self.last_detected_pose:
            self.publish_goal(returning=True)
            self.last_detected_time = current_time  # 重置，以便再次开始检测物体

    def publish_goal(self, returning=False):
        goal_pose = PoseStamped()
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.header.frame_id = "map"

        if returning:
            # 返回上一次检测到物体时的位置
            goal_pose.pose = self.last_detected_pose
        else:
            # 向前2米计算新目标
            current_position = self.current_pose.position
            current_orientation = self.current_pose.orientation
            quaternion = (
                current_orientation.x,
                current_orientation.y,
                current_orientation.z,
                current_orientation.w
            )
            euler = tf.transformations.euler_from_quaternion(quaternion)
            yaw = euler[2]
            goal_pose.pose.position.x = current_position.x + 2 * cos(yaw)
            goal_pose.pose.position.y = current_position.y + 2 * sin(yaw)
            goal_pose.pose.position.z = current_position.z
            goal_pose.pose.orientation = current_orientation

        self.goal_pub.publish(goal_pose)

if __name__ == '__main__':
    try:
        ObjectFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

