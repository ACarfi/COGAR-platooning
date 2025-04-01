#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist 

class RobotController:
    def __init__(self):
        self.desired_distance = 5  # Desired distance in meters
        self.current_speed = 0.0  # Current speed of the robot
        self.max_speed = 1.2  # Max speed (m/s)
        self.min_speed = 0  # Min speed (m/s)
        # Publisher to send the speed (cmd_vel_follower)
        self.action_pub = rospy.Publisher('/cmd_vel_follower', Twist, queue_size=10)

    def adjust_speed(self, current_distance):
        # Adjust speed based on how far the robot is from the leader
        if current_distance < self.desired_distance:
            self.current_speed = max(self.min_speed, self.current_speed - 0.1)  # Slow down if too close
        elif current_distance > self.desired_distance:
            self.current_speed = min(self.max_speed, self.current_speed + 0.1)  # Go faster if too far
        # No action if the robot is at the desired distance (already optimal)
        rospy.loginfo(f"Current distance: {current_distance} m, Adjusting speed to {self.current_speed} m/s.")
        return self.current_speed

    def publish_speed(self, speed):
        # Publish the new speed as a Twist message
        twist = Twist()
        twist.linear.x = speed
        self.action_pub.publish(twist)

def distance_callback(data):
    # Get the current distance from the message
    current_distance = data.data
    
    # Adjust the speed based on the current distance
    speed = controller.adjust_speed(current_distance)
    
    # Publish the adjusted speed
    controller.publish_speed(speed)

def action_node():
    rospy.init_node('action_node', anonymous=True)
    global controller
    controller = RobotController()  # Initialize the RobotController class
    rospy.Subscriber('/distance', Int32, distance_callback)  # Subscribe to the distance topic
    rospy.spin()

if __name__ == '__main__':
    try:
        action_node()
    except rospy.ROSInterruptException:
        pass
