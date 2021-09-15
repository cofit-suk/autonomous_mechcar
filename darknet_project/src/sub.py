#!/usr/bin/env python

import rospy
import math
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import Imu 
from std_msgs.msg import Float64 
import pdb
from yaw import Yaw
GOAL_X = 200
GOAL_Y = 290
TH = 10

# yaw gain parameter
yaw_pgain = 0.7
yaw_dgain = 0.05
yaw_igain = 0.01

# distance gain parameter
dis_pgain = 0.1
dis_dgain = 0.0
dis_igain = 0.0

# length parameter
k_gain = 1.5

# way point list
global way_point
way_point = [0,0,1,1]

# start flag
# if 
global start_flag
start_flag = False

class Control(Yaw):
    def __init__(self):
        super(Control, self).__init__()
        self.pub = rospy.Publisher('/motor_PWM', Int16MultiArray, queue_size = 10)
        self.pub_test = rospy.Publisher('/test_target_yaw', Float64, queue_size = 10)
        self.x = 400
        self.y = 200
        self.length = 100.0
        self.old_yaw_angle = 0.0
        self.yaw_angle = 0.0
        self.l_vel = 100.0
        self.r_vel = 100.0
        self.turn_flag = True
        self.dis_flag = False
        self.go_cnt = 0
        self.distance = 0.0
        self.old_distance = 0.0
        self.base_speed = 80
        # for set param 'target_yaw'
        rospy.set_param('target_yaw', 0)
    
        
    def drive(self):                                       
        msg = Int16MultiArray()
        msg_test = Float64()
        msg.data = [0,0]
        
        # calculate yaw from Yaw()
        cal_yaw= Yaw()
        target_angle = cal_yaw.final(way_point[0],way_point[1],way_point[2],way_point[3],self.x,self.y,k_gain,50)
        msg_test.data = target_angle
        cal_yaw.rosrun()
        self.pub_test.publish(msg_test)

        # for yaw control
        if self.turn_flag:            
            #target_angle = rospy.get_param('target_yaw')
            #print(target_angle)
            vel_amount = self.pid_angle(target_angle)        
            self.l_vel = self.l_vel+vel_amount
            self.r_vel = self.r_vel-vel_amount

            # contorl speed limit
            speed_lim = 100
            self.l_vel = speed_lim if self.l_vel > speed_lim else self.l_vel
            self.r_vel = speed_lim if self.r_vel > speed_lim else self.r_vel
            self.l_vel = -speed_lim if self.l_vel < -speed_lim else self.l_vel
            self.r_vel = -speed_lim if self.r_vel < -speed_lim else self.r_vel
        
        #update old_yaw_angle
        self.old_yaw_angle = self.yaw_angle
        #update old_distance
        self.old_distance = self.distance

        # move next way point
        if (abs(way_point[2]-self.x) < 20) and (abs(way_point[3]-self.y)<20):
            #pdb.set_trace()
            del way_point[0:2]
        
        msg.data[0] = self.l_vel + self.base_speed
        msg.data[1] = self.r_vel + self.base_speed
        
        # final way_point stop robot
        if(len(way_point)==2):
            msg.data[0] = 0
            msg.data[1] = 0
        rospy.loginfo('l = {}, r = {}'.format(msg.data[0], msg.data[1]))
        rospy.loginfo('yaw_data = %f' % self.yaw_angle)
        rospy.loginfo('x = {}, y = {}'.format(self.x, self.y))

        self.pub.publish(msg)
    
    
    def imu_callback(self, data):
        # update new yaw_angle
        self.yaw_angle = data.data
        #rospy.loginfo('yaw_data = %f' % self.yaw_angle)
    
    def locoation_callback(self, box):
        for data in box.bounding_boxes:
            if data.Class == "robot":
                self.x = (data.xmin + data.xmax) / 2
                self.y = (data.ymin + data.ymax) / 2
                self.length = ((data.xmax - data.xmin) + (data.ymax - data.ymin))/2
                print("length : %f" % self.length)
            #rospy.loginfo('class = {}, x = {}, y = {}, size = {}, prob. = {:.3f}'.format(data.Class, self.x, self.y, self.size, data.probability))
    
    def pid_angle(self, target_yaw):

        # yaw error    
        yaw_error = target_yaw - self.yaw_angle
        # p contorl
        p_amount_yaw = yaw_pgain * yaw_error

        # about loop rate dt = 0.01s
        dt = 0.01
        # i control
        i_amount_yaw = yaw_igain * yaw_error*dt
        # d control
        dif_yaw = (self.yaw_angle - self.old_yaw_angle)/dt
        d_amount_yaw = yaw_dgain * dif_yaw
        print("d_amount: %f" % d_amount_yaw)
        
        amount_yaw = p_amount_yaw + d_amount_yaw + i_amount_yaw
        print("i_amount: %f" % i_amount_yaw)


        # Reset l_vel, r_vel -> 0.0 stop overshoot
        if yaw_error <1.5 and yaw_error > -1.5:
            self.l_vel = 0.0
            self.r_vel = 0.0
            #amount_yaw = 0.0
        if yaw_error <0.5 and yaw_error > -0.5:
            yaw_error = 0.0
            self.l_vel = 0.0
            self.r_vel = 0.0
            amount_yaw = 0.0
            # if yaw_error is stable
            # self.turn_flag = False
            
        return amount_yaw

    def pid_distance(self, target_distance):
        # distance error
        dis_error = target_distance - self.distance
        # p control
        p_amount_dis = dis_pgain * dis_error
        # about loop rate dt = 0.01s
        dt = 0.01
        # i control
        i_amount_dis = dis_igain * dis_error*dt
        # d control
        dif_dis = (self.distance - self.old_distance)/dt
        d_amount_dis = dis_dgain * dif_dis
        #print("distnace_d: %f" % d_amount_dis)
        amount_dis = p_amount_dis + d_amount_dis + i_amount_dis
        # Reset l_vel, r_vel -> 0.0 stop overshoot
        if dis_error <0.3 and dis_error > -0.3:
            self.l_vel = 0.0
            self.r_vel = 0.0
            #amount_yaw = 0.0
        if dis_error <0.3 and dis_error > -0.3:
            dis_error = 0.0
            self.l_vel = 0.0
            self.r_vel = 0.0
            amount_dis = 0.0
        print("amount_dis: %f" % amount_dis)
        return amount_dis

def corcov(x, y):
    xd = x 
    yd = -y + 480.0
    return xd, yd
    # return x, y

def div_(x, y):
    if not y:
        return 0
    else:
        return x/y

    # point_data = Int16MultiArray()
    # point_data.data = self.points
    # self.pub_sendpoints.publish(point_data)

def point_callback(data):
    # update new yaw_angle
    point_data = list(data.data)
    way_cnt = point_data[0]
    del(point_data[0])
    #pdb.set_trace()
    # update global variable
    global way_point, start_flag
    start_flag = True
    way_point = point_data
    print(point_data)
    #rospy.loginfo('yaw_data = %f' % self.yaw_angle)

if __name__ == '__main__':
    rospy.init_node('contorl')
    control = Control()
    rospy.loginfo('run')
    rospy.Subscriber('/points',Int16MultiArray,point_callback)
    rospy.Subscriber('yaw_data', Float64, control.imu_callback)
    rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, control.locoation_callback)

    # about loop rate -> 10ms, 0.01s
    r = rospy.Rate(10)
    #global start_flag
    while not rospy.is_shutdown():
        # start_flag -> if gui send 
        if start_flag:
            control.drive()
            print(way_point)
        #rospy.spin()
        r.sleep()