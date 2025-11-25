#!/usr/bin/env python3

#This script initializes a ROS node that subscribes to the '/my_point' topic to receive PointStamped messages and prints the received messages to the console.

import rospy
from geometry_msgs.msg import PointStamped

class PointLocationReceiver:
    def __init__(self):
        rospy.init_node('receive_point_location', anonymous=True)  # Initialize the ROS node
        self.subscriber = rospy.Subscriber('/my_point', PointStamped, self.callback)  # Subscribe to the '/my_point' topic

    def callback(self, msg):
        print("Received PointStamped message:")  # Print a message indicating receipt
        rospy.loginfo(f"Received point: [{msg.point.x:.2f}, {msg.point.y:.2f}, {msg.point.z:.2f}]")

if __name__ == '__main__':
    receiver = PointLocationReceiver()  # Create an instance of the PointLocationReceiver class
    rospy.spin()  # Keep the node running and listening for messages
