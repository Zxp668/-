#! /usr/bin/env python
# -*- coding: utf-8 -*-
import os
import time
from geometry_msgs.msg import Twist, Vector3
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_msgs.msg import Int8
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
#import geometry_msgs/PoseWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
#from xf_mic_asr_offline.msg import lty_arrive
#from xf_mic_asr_offline.msg import lty_action
from nav_msgs.msg import Odometry
import serial
#import re
import rospy
#import sys
import math
import string
from sensor_msgs.msg import LaserScan

import actionlib

from std_msgs.msg import  Float64
#from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
# 定义ANSI颜色码
COLOR_RED = '\033[91m'
COLOR_GREEN = '\033[92m'
COLOR_YELLOW = '\033[93m'
COLOR_BLUE = '\033[94m'
COLOR_PURPLE = '\033[95m'
COLOR_CYAN = '\033[96m'
COLOR_WHITE = '\033[97m'

# 定义ANSI样式码
STYLE_BOLD = '\033[1m'
STYLE_UNDERLINE = '\033[4m'
STYLE_RESET = '\033[0m'

PI = 3.14159



class MOVE_ARRIVE:

    def time_delay(self,s):
        start_time = rospy.get_time()
        while (rospy.get_time() - start_time<s):
            xyz=0
    

    def task_A(self):

        print(COLOR_YELLOW+ ">>>>>Start Task A." + STYLE_RESET)
        self.time_delay(3.6)   #到A点的延时
        self.stop_msg.data = 0
        self.stop_flag_pub.publish(self.stop_msg)
        self.time_delay(3)
        self.pose_array_msg.data = 1
        self.scratch_pub.publish(self.pose_array_msg)



    
    
    
    

    def get_laser_min_dis(self, scan_data):
        # 获取角度的索引，例如角度为0度和180度
        left_desired_degrees = self.left_dis_angle
        right_desired_degrees = self.right_dis_angle
        left_desired_angle_rad = left_desired_degrees / 180.0 * PI
        right_desired_angle_rad = right_desired_degrees / 180.0 * PI
        left_angle_index = int((left_desired_angle_rad - scan_data.angle_min) / scan_data.angle_increment)
        right_angle_index = int((right_desired_angle_rad - scan_data.angle_min) / scan_data.angle_increment)
        
        left_dis = 0.0
        right_dis = 0.0

        rospy.loginfo("angle_min = %f, angle_max = %f, angle_increment = %f", scan_data.angle_min, scan_data.angle_max, scan_data.angle_increment)
        rospy.loginfo("left_angle_index = %d, right_angle_index = %d",left_angle_index, right_angle_index)

        if 0 <= left_angle_index < len(scan_data.ranges):
            left_dis = min(scan_data.ranges[left_angle_index:left_angle_index+2])
            rospy.loginfo("Left Dis at %f degrees: %f meters", left_desired_degrees, left_dis)
        else:
            rospy.logwarn("Left Desired angle is out of range")

        if 0 <= right_angle_index < len(scan_data.ranges):
            right_dis = min(scan_data.ranges[right_angle_index:right_angle_index+2])
            rospy.loginfo("Right Dis at %f degrees: %f meters", right_desired_degrees, right_dis)
        else:
            rospy.logwarn("Right Desired angle is out of range")


        self.min_dis = [left_dis, right_dis]


    def __init__(self):

        self.cmd_vel_topic = rospy.get_param('cmd_vel_topic','/cmd_vel')
        self.cmd_arm_topic = rospy.get_param('cmd_arm_topic','/arm_cmd')###teb
       
        self.go_along_time = rospy.get_param("go_along_time",[[5.0,30.0],[5.0,30.0],[5.0,30.0]])
        self.task_A_go_along = rospy.get_param("task_A_go_along",[1,2,2,2,2,2,3,4,5])
        self.task_B_go_along = rospy.get_param("task_B_go_along",[1,2,2,2,2,2,3,6])
        self.task_C_go_along = rospy.get_param("task_C_go_along",[1,2,2,2,2,2,3,4,5])
        
        
        self.cmd_arm_pub = rospy.Publisher(self.cmd_arm_topic,Int32,queue_size=10)
        self.stop_flag_pub = rospy.Publisher("/stop_flag",Int8,queue_size=10)
        self.scratch_pub =  rospy.Publisher("/read_array",Int32,queue_size=10)

        # 订阅雷达数据，获取左右两侧障碍物的最小距离
        self.left_dis_angle = 0
        self.right_dis_angle = 180
        self.min_dis = [10,10]
        rospy.Subscriber('/scan', LaserScan, self.get_laser_min_dis)

        
        self.linear_x = rospy.get_param('linear_x',0.2)
        self.anglular_z = rospy.get_param('anglular_z',2.5)
        self.pose_array_msg = Int32()
        self.stop_msg = Int8()

        #self.timer_vel = rospy.Timer(rospy.Duration(0.05),self.ljy_vel)###定时器发布所有速度
        


    def RobotStart(self):   
        #while not rospy.is_shutdown():        
        self.time_delay(1.0) 
        self.task_A()
      

#main function
if __name__=="__main__":
    try:
        rospy.init_node('compitation_scratch',anonymous=True)
        rospy.loginfo('compitation_irrigate_node instruction start...')
        MA = MOVE_ARRIVE()
        MA.RobotStart()
        rospy.spin()

    except KeyboardInterrupt:
        print("Shutting down")

