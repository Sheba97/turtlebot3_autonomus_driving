#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import sys, select, termios, tty, math
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import time

class ControlSquareOdometry:
    def __init__(self):
        rospy.init_node('control_square_odometry', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.rate = rospy.Rate(10)
        self.settings = termios.tcgetattr(sys.stdin)
        self.yaw = 0.0
        self.x_pos = 0.0
        self.y_pos = 0.0

    def odom_callback(self, msg):
        self.x_pos = msg.pose.pose.position.x
        self.y_pos = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.yaw) = euler_from_quaternion(orientation_list)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def move_square(self,side_length =1.0, speed=0.2, turn_speed=0.8):
        move_cmd = Twist()
        move_cmd.linear.x = speed
        move_cmd.angular.z = 0.0

        turn_cmd = Twist()
        turn_cmd.linear.x = 0.0
        turn_cmd.angular.z = turn_speed 


        for i in range(4):
            # Move forward
            print("Moving forward")
            self.pub.publish(move_cmd)
            rospy.sleep(0.2) 
            start_x = self.x_pos
            start_y = self.y_pos
            while True: 
                distance = math.sqrt((self.x_pos - start_x)**2 + (self.y_pos - start_y)**2)
                
                if distance >= side_length:
                    break
                self.pub.publish(move_cmd)
                self.rate.sleep()

            # Stop before turning
            self.pub.publish(Twist())
            rospy.sleep(0.2)

            # Turn
            print("Turning 90 degrees")
            start_yaw = self.yaw
            while True:
                dyaw = self.normalize_angle(self.yaw - start_yaw)

                if abs(dyaw) >= math.pi / 2:
                    break

                self.pub.publish(turn_cmd)
                self.rate.sleep()

            # Apply braking to stop precise angle
            brake_turn = Twist()
            brake_turn.angular.z = -0.2
            self.pub.publish(brake_turn)
            rospy.sleep(0.2)

            self.pub.publish(Twist())
            rospy.sleep(0.2)


    def run(self):
        try:
            print("Press 's' to start moving in a square, 'q' to quit.")
            while not rospy.is_shutdown():
                key = self.getKey()
                if key == 's':
                    print("Starting square movement...")
                    self.move_square()
                    print("Square movement completed.")
                elif key == 'q':
                    print("Exiting...")
                    break
        except Exception as e:
            print(e)
        finally:
            self.pub.publish(Twist())
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

if __name__ == '__main__':
    controller = ControlSquareOdometry()
    controller.run()
