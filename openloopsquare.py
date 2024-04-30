#!/usr/bin/env python3
# Satyam Raina
# 222196682
# Task: Task 5.1P Open Loop Square 

import rospy
from duckietown_msgs.msg import Twist2DStamped
from duckietown_msgs.msg import FSMState
import time
import math

class DuckiebotSquareDriver:
    def __init__(self):
        self.cmd_msg = Twist2DStamped()
        self.side_length = 1.0  
        self.straight_line_speed = 0.5  
        self.angular_speed = math.pi / 2  
        self.publish_rate = 0.1  
        self.state = "STOPPED"  

        rospy.init_node('duckiebot_square_driver', anonymous=True)

        self.pub = rospy.Publisher('/sherr/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=2)
        rospy.Subscriber('/sherr/fsm_node/mode', FSMState, self.fsm_callback, queue_size=1)

    def fsm_callback(self, msg):
        rospy.loginfo("State: %s", msg.state)
        if msg.state == "NORMAL_JOYSTICK_CONTROL":
            self.stop_robot()
            self.state = "STOPPED"
        elif msg.state == "LANE_FOLLOWING":
            if self.state == "STOPPED":
                rospy.sleep(1)  
                self.drive_square()
                self.state = "DRIVING"
            else:
                rospy.logwarn("Duckiebot is already driving a square!")
        else:
            rospy.logerr("Unexpected FSM state: %s", msg.state)
            self.stop_robot()
            self.state = "STOPPED"

    def stop_robot(self):
        self.cmd_msg.header.stamp = rospy.Time.now()
        self.cmd_msg.v = 0.0
        self.cmd_msg.omega = 0.0
        self.pub.publish(self.cmd_msg)

    def go_forward(self, duration):
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.v = self.straight_line_speed
            self.cmd_msg.omega = 0
            self.pub.publish(self.cmd_msg)
            time.sleep(self.publish_rate)

    def turn(self, angle):
        duration = abs(angle / self.angular_speed)
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_msg.header.stamp = rospy.Time.now()
            self.cmd_msg.v = 0
            self.cmd_msg.omega = self.angular_speed if angle > 0 else -self.angular_speed
            self.pub.publish(self.cmd_msg)
            time.sleep(self.publish_rate)

    def drive_square(self):
        rospy.loginfo("Driving a square with side length of %f meters", self.side_length)
        for _ in range(4):
            self.go_forward(self.side_length / self.straight_line_speed)
            self.turn(math.pi / 2)
        self.stop_robot()
        rospy.loginfo("Square driving complete!")

    def run(self):
        rospy.spin()  

if __name__ == '__main__':
    try:
        duckiebot_driver = DuckiebotSquareDriver()
        duckiebot_driver.run()
    except rospy.ROSInterruptException:
        pass
