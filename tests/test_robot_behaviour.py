#!/usr/bin/env python

import unittest
import rospy
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist
import numpy as np


class TestRobotBehavior(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node('test_robot_behavior', anonymous=True)

        cls.max_speed = 1.0  # Max speed (m/s)
        cls.min_speed = 0  # Min speed (m/s)

    def setUp(self):
        self.test_type = rospy.get_param("~test_type", "constant")  # Default to test_costant if not set

        self.received_action = []
        self.received_distance = []
        self.initial_distance = 2
        self.target_distance = 5
        self.allowed_error = 5*0.1
        self.initial_follower_velocity = 0

        rospy.Subscriber('/cmd_vel_follower', Twist, self.action_callback)
        rospy.Subscriber('/distance', Int32, self.distance_callback)
        self.vel_pub = rospy.Publisher('/cmd_vel_target', Twist, queue_size=10)
        rospy.sleep(1)  # Allow time for nodes to initialize

    def distance_callback(self, data):
        self.received_distance.append(data.data)

    def action_callback(self, data):
        self.received_action.append(data.linear.x)

    def test(self):
        if(self.test_type == "constant"):
            self.constant_acc()
        elif(self.test_type == "sinusoid"):
            self.sinusoid()

    def constant_acc(self):
        speed_increment = 0.1
        time_steps = 200
        speed_list = [min(self.min_speed + i * speed_increment, self.max_speed) for i in range(time_steps)]

        for s in speed_list:
            msg = Twist()
            msg.linear.x = s
            self.vel_pub.publish(msg)
            rospy.sleep(0.1)

        rospy.sleep(2)  # Allow time for system to stabilize

        # Print all recorded distances
        rospy.loginfo(f"Recorded distances: {self.received_distance}")
        rospy.loginfo(f"Recorded follower velocities: {self.received_action}")

        # Check if the final distance is within 20% tolerance of target distance
        final_distance = self.received_distance[-1] if self.received_distance else None
        self.assertIsNotNone(final_distance, "No distance messages received.")

        lower_bound = self.target_distance - self.allowed_error
        upper_bound = self.target_distance + self.allowed_error

        self.assertTrue(lower_bound <= final_distance <= upper_bound, f"Final distance {final_distance}m is out of allowed range [{lower_bound}, {upper_bound}].")

    def sinusoid(self):
        speed_increment = 0.1
        time_steps = 200
        frequency = 0.1  # Adjust frequency to control the oscillation speed
        amplitude = 0.5   # Adjust amplitude to scale speed variation
        offset = 0.5      # Offset to ensure speed remains positive
        speed_list = [min(max(self.min_speed, offset + amplitude * np.sin(2 * np.pi * frequency * i)), self.max_speed) for i in range(time_steps)]
        speed_list = [min(self.min_speed + i * speed_increment, self.max_speed) for i in range(time_steps)]

        for s in speed_list:
            msg = Twist()
            msg.linear.x = s
            self.vel_pub.publish(msg)
            rospy.sleep(0.1)

        rospy.sleep(2)  # Allow time for system to stabilize

        # Print all recorded distances
        rospy.loginfo(f"Recorded target velocity: {speed_list}")
        rospy.loginfo(f"Recorded distances: {self.received_distance}")
        rospy.loginfo(f"Recorded follower velocities: {self.received_action}")

        # Check if the final distance is within 20% tolerance of target distance
        final_distance = self.received_distance[-1] if self.received_distance else None
        self.assertIsNotNone(final_distance, "No distance messages received.")

        lower_bound = self.target_distance - self.allowed_error
        upper_bound = self.target_distance + self.allowed_error

        self.assertTrue(lower_bound <= final_distance <= upper_bound, f"Final distance {final_distance}m is out of allowed range [{lower_bound}, {upper_bound}].")


if __name__ == '__main__':
    import rostest
    rostest.rosrun('example_pkg', 'test_robot_behavior', TestRobotBehavior)

         