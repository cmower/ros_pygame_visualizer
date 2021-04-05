#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Point

class Node:

    dt = 1.0/100.0

    def __init__(self):
        rospy.init_node('test_ros_pygame_visualizer_node')
        self.p = [0.1, 0.1]
        self.maxu = 0.05
        self.u = [0, 0]
        self.pub = rospy.Publisher('robot/position', Point, queue_size=10)
        rospy.Subscriber('joy', Joy, self.callback)
        rospy.Timer(rospy.Duration(self.dt), self.mainLoop)

    def callback(self, msg):
        self.u = [-self.maxu * msg.axes[0], -self.maxu * msg.axes[1]]

    def mainLoop(self, event):
        self.p = [p+self.dt*u for p, u in zip(self.p, self.u)]
        self.pub.publish(Point(x=self.p[0], y=self.p[1]))

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    Node().spin()
