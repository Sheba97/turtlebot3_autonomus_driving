#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
from geometry_msgs.msg import Point


class PointLocationSender:
    def __init__(self):
        rospy.init_node('send_point_loaction', anonymous=True) # Initialize the ROS node
        self.publisher = rospy.Publisher('/my_point', PointStamped, queue_size=10)  # Publisher for the '/my_point' topic
        self.rate = rospy.Rate(1)  # Set the rate to 1 Hz
        self.send_point_location()  # Call the method to send point location
    
    def send_point_location(self):
        self.my_header = Header(stamp=rospy.Time.now(), frame_id='odom') # Create a header with the current time and frame_id 'odom'
        self.my_point = Point(1.0, 2.0, 0.0) # Define a point at coordinates (1.0, 2.0, 0.0)
        self.my_point_stamped = PointStamped(header=self.my_header, point=self.my_point) # Create a PointStamped message
        print(self.my_point_stamped) # Print the PointStamped message to the console
        
        while not rospy.is_shutdown():
            self.my_point_stamped.header.stamp = rospy.Time.now() # Update the timestamp in the header
            self.publisher.publish(self.my_point_stamped) # Publish the PointStamped message
            self.rate.sleep() # Sleep to maintain the loop rate



if __name__ == '__main__':
    sender = PointLocationSender()  # Create an instance of the PointLocationSender class



    