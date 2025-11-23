#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import sys, select, termios, tty, math
import time

class ControlSquare:
    def __init__(self):
        rospy.init_node('control_square', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.settings = termios.tcgetattr(sys.stdin)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def move_square(self, side_length=1.0, speed=0.2, turn_speed=0.5):
        move_cmd = Twist()
        move_cmd.linear.x = speed
        move_cmd.angular.z = 0.0
        move_duration = side_length / speed

        turn_cmd = Twist()
        turn_cmd.linear.x = 0.0
        turn_cmd.angular.z = turn_speed 
        turn_angle = math.pi/2              # 90 degrees in radians
        turn_duration = turn_angle/turn_speed 

        for _ in range(4):
            # Move forward
            print("Moving forward")
            start_time = time.time()
            while time.time() - start_time < move_duration:
                self.pub.publish(move_cmd)
                self.rate.sleep()

            # Stop before turning
            self.pub.publish(Twist())
            time.sleep(0.5)

            # Turn
            print("Turning 90 degrees")
            start_time = time.time()
            while time.time() - start_time < turn_duration:  
                self.pub.publish(turn_cmd)
                self.rate.sleep()

            # Stop after turning
            self.pub.publish(Twist())
            time.sleep(0.5)

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
    controller = ControlSquare()
    controller.run()
