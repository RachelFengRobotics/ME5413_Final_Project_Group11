#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
import tf
from math import cos, sin, radians, pi

class ObjectFollower:
    def __init__(self):
        rospy.init_node('object_follower_type2')

        self.odom_sub = rospy.Subscriber('/gazebo/ground_truth/state', Odometry, self.odom_callback)
        self.obj_sub = rospy.Subscriber('/objects', Float32MultiArray, self.obj_callback)

        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

        self.current_pose = None
        self.last_published_time = rospy.Time.now()
        self.object_detected_once = False  # Indicates if an object has been detected at least once.

    def odom_callback(self, msg):
        # Update the current robot position based on odometry information.
        self.current_pose = msg.pose.pose

    def obj_callback(self, msg):
        current_time = rospy.Time.now()
        if len(msg.data) > 0:
            # Object detected.
            self.object_detected_once = True  # Mark that an object has been detected.
            self.publish_goal()
            self.last_published_time = current_time
        elif self.object_detected_once and (current_time - self.last_published_time).to_sec() >= 10.0:
            # If an object was detected at least once and no object has been detected for 10 seconds, rotate.
            self.rotate_right_degrees()

    def publish_goal(self):
        # Calculate and publish a new target position 2 meters in front of the robot.
        if self.current_pose:
            quaternion = (
                self.current_pose.orientation.x,
                self.current_pose.orientation.y,
                self.current_pose.orientation.z,
                self.current_pose.orientation.w
            )
            euler = tf.transformations.euler_from_quaternion(quaternion)
            yaw = euler[2]

            goal_pose = PoseStamped()
            goal_pose.header.stamp = rospy.Time.now()
            goal_pose.header.frame_id = "map"
            goal_pose.pose.position.x = self.current_pose.position.x + 1.5 * cos(yaw)
            goal_pose.pose.position.y = self.current_pose.position.y + 1.5 * sin(yaw)
            goal_pose.pose.orientation = self.current_pose.orientation

            self.goal_pub.publish(goal_pose)

    def rotate_right_degrees(self):
        # Rotate the robot x degrees to the right.
        if self.current_pose:
            quaternion = (
                self.current_pose.orientation.x,
                self.current_pose.orientation.y,
                self.current_pose.orientation.z,
                self.current_pose.orientation.w
            )
            euler = tf.transformations.euler_from_quaternion(quaternion)
            new_yaw = (euler[2] - radians(75)) % (2 * pi)  # Ensure yaw stays within valid range

            new_quaternion = tf.transformations.quaternion_from_euler(euler[0], euler[1], new_yaw)
            
            goal_pose = PoseStamped()
            goal_pose.header.stamp = rospy.Time.now()
            goal_pose.header.frame_id = "map"
            goal_pose.pose.position = self.current_pose.position  # Keep position unchanged
            goal_pose.pose.orientation.x = new_quaternion[0]
            goal_pose.pose.orientation.y = new_quaternion[1]
            goal_pose.pose.orientation.z = new_quaternion[2]
            goal_pose.pose.orientation.w = new_quaternion[3]

            self.goal_pub.publish(goal_pose)
            self.last_published_time = rospy.Time.now()  # Update the time a goal was last published

if __name__ == '__main__':
    try:
        ObjectFollower()
        rospy.spin()  # Keep the program running.
    except rospy.ROSInterruptException:
        pass
