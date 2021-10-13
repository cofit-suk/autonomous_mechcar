#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16MultiArray, Float32MultiArray
from math import sin, cos, atan2, pi

PICTURE_WIDTH = 640
PICTURE_HEIGHT = 480

class Yaw(object):
    def __init__(self):
        self.xlos = 0
        self.ylos = 0
        self.xd = 0
        self.yd = 0
        self.psi = 0

    def final(self, x_1, y_1, x_2, y_2, x_r, y_r, k_p, l_p):
        K = k_p# coefficient
        L = l_p  # robot length

        x1, y1 = corcov(x_1, y_1)
        x2, y2 = corcov(x_2, y_2)
        xr, yr = corcov(x_r, y_r)

        x = x2 - x1
        y = y2 - y1

        a = y
        b = -x
        c = x2*y1 - x1*y2

        self.xd = (b*(b*xr-a*yr)-a*c)/(a**2+b**2)
        self.yd = (a*(-b*xr+a*yr)-b*c)/(a**2+b**2)

        ang = atan2(y, x) # Even if the denominator is zero, it can be executed
        self.xlos = self.xd + (cos(ang) * K*L)
        self.ylos = self.yd + (sin(ang) * K*L)

        self.psi = -atan2(self.ylos-yr, self.xlos-xr)
        print("theta = {}".format(self.psi*180/pi))
        return self.psi*180/pi
        #self.sendAction()

    def sendAction(self):
        los_data = Float32MultiArray()
        xl, yl = corcov(self.xlos, self.ylos)
        los_data.data.extend([xl, yl])
        self.pub.publish(los_data)

        d_data = Float32MultiArray()
        x_d, y_d = corcov(self.xd, self.yd)
        d_data.data.extend([x_d, y_d])
        self.pub_d.publish(d_data)

    def rosrun(self):
        rospy.loginfo('yaw_run')
        sub = rospy.Subscriber('/points', Int16MultiArray, self.callback)
        self.pub = rospy.Publisher('/yawtest', Float32MultiArray, queue_size = 1)
        self.pub_d = rospy.Publisher('/xdyd', Float32MultiArray, queue_size = 1)
        self.sendAction()

    def callback(self, points):
        self.final(points.data[3], points.data[4], points.data[5], points.data[6], points.data[1], points.data[2])
        self.sendAction()

def corcov(x, y):
    return x, 480-y

if __name__ == '__main__':
    rospy.init_node('yaw')
    #rospy.loginfo('yaw_run')
    yaw = Yaw()
    yaw.rosrun()
    rospy.spin()
