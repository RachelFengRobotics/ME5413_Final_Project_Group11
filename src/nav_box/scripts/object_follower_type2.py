#!/usr/bin/env python3

# Import necessary libraries
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
import tf
from math import cos, sin, radians, pi

# Define the ObjectFollower class
class ObjectFollower:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('object_follower_type2')

        # Subscribe to odometry and object detection topics
        self.odom_sub = rospy.Subscriber('/gazebo/ground_truth/state', Odometry, self.odom_callback)
        self.obj_sub = rospy.Subscriber('/objects', Float32MultiArray, self.obj_callback)

        # Publisher to publish goal positions for the robot
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

        # Initialize variables to keep track of the robot's state and the detection of objects
        self.current_pose = None
        self.last_published_time = rospy.Time.now()
        self.object_detected_once = False
        self.is_rotating = False
        self.rotate_timer = None  # Timer used to control the rotation state

    # Callback function for odometry messages
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    # Callback function for object detection messages
    def obj_callback(self, msg):
        current_time = rospy.Time.now()
        if len(msg.data) > 0:
            self.object_detected_once = True
            if self.is_rotating:
                # If an object is detected while rotating, immediately stop rotation and publish a goal position
                if self.rotate_timer:
                    self.rotate_timer.shutdown()  # Stop the current rotation timer
                self.is_rotating = False  # Reset rotation state
            # Publish a new goal position upon detecting an object, regardless of rotation
            if (current_time - self.last_published_time).to_sec() >= 2.0:
                self.publish_goal(current_time)
                self.last_published_time = current_time
        elif self.object_detected_once and (current_time - self.last_published_time).to_sec() >= 7.0 and not self.is_rotating:
            # If no object is detected for a while, start rotating
            self.rotate_right_degrees(current_time)

    # Function to publish a goal position based on the current pose and detection
    def publish_goal(self, current_time):
        if self.current_pose:
            quaternion = (
                self.current_pose.orientation.x,
                self.current_pose.orientation.y,
                self.current_pose.orientation.z,
                self.current_pose.orientation.w
            )
            euler = tf.transformations.euler_from_quaternion(quaternion)
            yaw = euler[2]

            # Calculate and publish the new goal position in front of the current position
            goal_pose = PoseStamped()
            goal_pose.header.stamp = current_time
            goal_pose.header.frame_id = "map"
            goal_pose.pose.position.x = self.current_pose.position.x + 1.5 * cos(yaw)
            goal_pose.pose.position.y = self.current_pose.position.y + 1.5 * sin(yaw)
            goal_pose.pose.orientation = self.current_pose.orientation

            self.goal_pub.publish(goal_pose)

    # Function to rotate the robot to the right by a certain degree
    def rotate_right_degrees(self, current_time):
        if self.current_pose and (current_time - self.last_published_time).to_sec() >= 5.0:
            self.is_rotating = True
            quaternion = (
                self.current_pose.orientation.x,
                self.current_pose.orientation.y,
                self.current_pose.orientation.z,
                self.current_pose.orientation.w
            )
            euler = tf.transformations.euler_from_quaternion(quaternion)
            new_yaw = (euler[2] - radians(75)) % (2 * pi)  # Calculate the new yaw after rotation

            new_quaternion = tf.transformations.quaternion_from_euler(euler[0], euler[1], new_yaw)
            
            # Publish the goal position to rotate the robot
            goal_pose = PoseStamped()
            goal_pose.header.stamp = current_time
            goal_pose.header.frame_id = "map"
            goal_pose.pose.position = self.current_pose.position
            goal_pose.pose.orientation.x = new_quaternion[0]
            goal_pose.pose.orientation.y = new_quaternion[1]
            goal_pose.pose.orientation.z = new_quaternion[2]
            goal_pose.pose.orientation.w = new_quaternion[3]

            self.goal_pub.publish(goal_pose)
            self.last_published_time = current_time

            # Set a timer to reset the rotation state after completion
            self.rotate_timer = rospy.Timer(rospy.Duration(2), self.end_rotation, oneshot=True)

    # Function to reset the rotation state after the timer ends
    def end_rotation(self, event):
        self.is_rotating = False
        self.rotate_timer = None  # Clear the timer reference

# Main function to run the ROS node
if __name__ == '__main__':
    try:
        ObjectFollower()
        rospy.spin()  # Keep the node running
    except rospy.ROSInterruptException:
        pass
