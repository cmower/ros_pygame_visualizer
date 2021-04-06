#!/usr/bin/env python3
import rospy
import random
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray

class Node:

    dt = 1.0/50.0
    n_pred = 100

    def __init__(self):
        rospy.init_node('test_ros_pygame_visualizer_node')
        self.p = [0.1, 0.1]
        self.maxu = 0.05
        self.u = [0, 0]
        self.pub_robot = rospy.Publisher('robot/position', Point, queue_size=10)
        self.pub_pred = rospy.Publisher('prediction/position', Float64MultiArray, queue_size=10)
        self.pub_point_array = rospy.Publisher('point_array/position', Float64MultiArray, queue_size=10)
        self.points = []
        rospy.Subscriber('joy', Joy, self.callback)
        rospy.Timer(rospy.Duration(self.dt), self.mainLoop)
        rospy.Timer(rospy.Duration(1.0), self.publishPointArray)

    def publishPointArray(self, event):
        self.points.append([0.6*random.random(), 0.45*random.random()])
        self.pub_point_array.publish(Float64MultiArray(data=[p[0] for p in self.points] + [p[1] for p in self.points]))

    def callback(self, msg):
        self.u = [-self.maxu * msg.axes[0], -self.maxu * msg.axes[1]]

    def mainLoop(self, event):
        self.p = [p+self.dt*u for p, u in zip(self.p, self.u)]
        self.pub_robot.publish(Point(x=self.p[0], y=self.p[1]))
        prediction = [self.p]*self.n_pred
        for i in range(self.n_pred-1):
            p_old = prediction[i]
            p_new = [p+self.dt*u for p, u in zip(p_old, self.u)]
            prediction[i+1] = p_new
        self.pub_pred.publish(Float64MultiArray(data=[p[0] for p in prediction]+[p[1] for p in prediction]))

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    Node().spin()
