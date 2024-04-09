#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
import tf
from math import cos, sin

class ObjectFollower:
    def __init__(self):
        rospy.init_node('object_follower')

        # 订阅 /gazebo/ground_truth/state 和 /objects 话题
        self.odom_sub = rospy.Subscriber('/gazebo/ground_truth/state', Odometry, self.odom_callback)
        self.obj_sub = rospy.Subscriber('/objects', Float32MultiArray, self.obj_callback)

        # 发布到 /move_base_simple/goal
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

        self.current_pose = None
        self.last_published_time = rospy.Time.now()  # 初始化上次发布时间

    def odom_callback(self, msg):
        # 更新当前位置
        self.current_pose = msg.pose.pose

    def obj_callback(self, msg):
        # 检测到物体时（即，data数组非空）执行
        current_time = rospy.Time.now()
        if len(msg.data) > 0 and self.current_pose is not None and \
           (current_time - self.last_published_time).to_sec() >= 3.0:
            self.publish_goal()
            self.last_published_time = current_time  # 更新上次发布时间

    def publish_goal(self):
        goal_pose = PoseStamped()
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.header.frame_id = "map"

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

