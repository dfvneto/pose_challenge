#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import numpy as np
from math import atan2
class Turtle:
    def __init__(self, node_name, publisher_topic_name, subscriber_topic_name):
        rospy.init_node(node_name, anonymous=True)
        self.publisher = rospy.Publisher(publisher_topic_name, Twist, queue_size=10)
        self.subscriber = rospy.Subscriber(subscriber_topic_name, Pose, self.callback)
        self.pose = Pose()
        self.rate = rospy.Rate(10)

    def callback(self, data):
        self.pose.x = round(data.x,1)
        self.pose.y = round(data.y,1)
        self.pose.theta = data.theta

    def move(self,vel):
        msg = Twist()
        msg.linear.x = vel
        self.publisher.publish(msg)
    def steer(self,x,y):
        msg = Twist()
        while round(atan2(y - self.pose.y, x - self.pose.x)-self.pose.theta, 2) !=0:
            msg.angular.z = atan2(y - self.pose.y, x - self.pose.x)-self.pose.theta
            self.publisher.publish(msg)
            self.rate.sleep()
        msg.angular.z = 0
        self.publisher.publish(msg)
    def set_goal(self, goal):
        goal_array = np.array([goal.x,goal.y])
        self.steer(goal.x,goal.y)

        while np.linalg.norm(goal_array-np.array([self.pose.x,self.pose.y])) > 0.1:
            self.move(1)
            self.rate.sleep()
        self.move(0)

if __name__ == '__main__':
    tartaral_generuga = Turtle("tartaral_generuga","turtle1/cmd_vel", "/turtle1/pose")
    try:
        while not rospy.is_shutdown():
            print("Choose your destiny")
            x = float(input())
            y = float(input())
            goal = Pose()
            goal.x = round(x,1)
            goal.y = round(y,1)
            tartaral_generuga.set_goal(goal)
        rospy.spin()

    except rospy.ROSInterruptException: pass