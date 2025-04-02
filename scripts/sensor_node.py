#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

class DistanceNode:
    def __init__(self):
        self.prev_distance = 2  # Initial distance is 2 meters
        self.target_velocity = 0.0  # Velocity of the target robot (leader)
        self.follower_velocity = 0.0  # Velocity of the follower robot
        self.time_step = 0.1  # Since the loop runs at 10Hz, each step is 0.1s
        # Publisher to publish the calculated distance
        self.distance_pub = rospy.Publisher('/distance', Int32, queue_size=10)

    def target_velocity_callback(self, data):
        """Callback to get the target robot's velocity."""
        self.target_velocity = data.linear.x

    def follower_velocity_callback(self, data):
        """Callback to get the follower robot's velocity."""
        self.follower_velocity = data.linear.x

    def compute_distance(self):
        """
        Update the distance based on both target and follower velocities.
        The distance is updated using the relative velocity over `time_step`.
        """
        relative_velocity = self.target_velocity - self.follower_velocity
        self.prev_distance += relative_velocity * self.time_step

        # Create the message and publish the updated distance
        distance = Int32()
        distance.data = int(self.prev_distance)  # Convert to integer before publishing
        self.distance_pub.publish(distance)  # Publish the updated distance

def sensor_node():
    rospy.init_node('sensor_node', anonymous=True)
    
    # Create an instance of the DistanceNode class
    node = DistanceNode()
    
    # Subscribe to both the target's and follower's velocity topics
    rospy.Subscriber('/cmd_vel_target', Twist, node.target_velocity_callback)  # For the target velocity
    rospy.Subscriber('/cmd_vel_follower', Twist, node.follower_velocity_callback)  # For the follower velocity
    
    rate = rospy.Rate(10)  # Set the loop frequency to 10Hz
    while not rospy.is_shutdown():
        node.compute_distance()  # Compute the distance at each loop iteration
        rate.sleep()  # Sleep to maintain the desired frequency

if __name__ == '__main__':
    sensor_node()
