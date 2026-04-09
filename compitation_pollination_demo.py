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
        for i in range(7):
            print(COLOR_YELLOW+ ">>>>>>>>>>>Task A No"+str(i+1) +" ..."+ STYLE_RESET)
            if i<4:
                self.cmd_vel_msg.linear.x = self.linear_x
                self.cmd_vel_msg.angular.z = 0
            elif i == 4:
                self.cmd_vel_msg.linear.x = 0
                self.cmd_vel_msg.angular.z = -self.anglular_z
            elif i == 5:
                self.cmd_vel_msg.linear.x = self.linear_x
                self.cmd_vel_msg.angular.z = 0
            elif i == 6:
                self.cmd_vel_msg.linear.x = 0
                self.cmd_vel_msg.angular.z = -self.anglular_z
            self.cmd_vel_pub.publish(self.cmd_vel_msg)
            print(COLOR_YELLOW+ ">>>>>>>>>>>Move. Speed is "+str(self.linear_x) +"m/s. Move time is "\
                                        +str(self.go_along_time[self.task_A_go_along[i]][0])+"s. Stop time is "\
                                            +str(self.go_along_time[self.task_A_go_along[i]][1])+"s."+ STYLE_RESET)

            # 可以尝试把延时换为激光雷达左右两侧距离数据的判断
            self.time_delay(self.go_along_time[self.task_A_go_along[i]][0])
            # 打印激光雷达两侧的距离数据，单位：m
            rospy.loginfo("Left Dis = %fm, Right Dis = %fm", self.min_dis[0], self.min_dis[1])

            self.cmd_vel_msg.linear.x = 0
            self.cmd_vel_msg.angular.z = 0
            self.cmd_vel_pub.publish(self.cmd_vel_msg)
            print(COLOR_YELLOW+ ">>>>>>>>>>>Stop."+ STYLE_RESET)
            if i<3:
                print(COLOR_YELLOW+ ">>>>>>>>>>>Arm active to detect left."+ STYLE_RESET)
                # 机械臂识别左侧作物
                self.cmd_arm_msg.data = 2
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(self.go_along_time[self.task_A_go_along[i]][1])
                print(COLOR_YELLOW+ ">>>>>>>>>>>Start to detect. Times="+str(self.go_along_time[self.task_A_go_along[i]][1])+ STYLE_RESET)
                
                # 此处根据yolo识别结果，判断机械臂去哪里授粉
                print(COLOR_YELLOW+ ">>>>>>>>>>>Arm active to pollinate up."+ STYLE_RESET)
                # 对上方的花朵进行授粉
                # 如果动作不够，可以在这里加动作
                self.cmd_arm_msg.data = 4
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(self.go_along_time[self.task_A_go_along[i]][1])
                self.cmd_arm_msg.data = 2
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(self.go_along_time[self.task_A_go_along[i]][1])
                
                print(COLOR_YELLOW+ ">>>>>>>>>>>Arm active to pollinate mid."+ STYLE_RESET)
                # 对中间的花朵进行授粉
                # 如果动作不够，可以在这里加动作
                self.cmd_arm_msg.data = 5
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(self.go_along_time[self.task_A_go_along[i]][1])
                self.cmd_arm_msg.data = 2
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(self.go_along_time[self.task_A_go_along[i]][1])
                
                print(COLOR_YELLOW+ ">>>>>>>>>>>Arm active to pollinate bot."+ STYLE_RESET)
                # 对下方的花朵进行授粉
                # 如果动作不够，可以在这里加动作
                self.cmd_arm_msg.data = 6
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(self.go_along_time[self.task_A_go_along[i]][1])
                self.cmd_arm_msg.data = 2
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(self.go_along_time[self.task_A_go_along[i]][1])
                
                print(COLOR_YELLOW+ ">>>>>>>>>>>Arm active to detect right."+ STYLE_RESET)
                # 机械臂识别右侧作物
                self.cmd_arm_msg.data = 3
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(self.go_along_time[self.task_A_go_along[i]][1])
                print(COLOR_YELLOW+ ">>>>>>>>>>>Start to detect. Times="+str(self.go_along_time[self.task_A_go_along[i]][1])+ STYLE_RESET)
                
                # 此处根据yolo识别结果，判断机械臂去哪里授粉
                print(COLOR_YELLOW+ ">>>>>>>>>>>Arm active to pollinate up."+ STYLE_RESET)
                # 对上方的花朵进行授粉
                # 如果动作不够，可以在这里加动作
                self.cmd_arm_msg.data = 7
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(self.go_along_time[self.task_A_go_along[i]][1])
                self.cmd_arm_msg.data = 3
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(self.go_along_time[self.task_A_go_along[i]][1])
                
                print(COLOR_YELLOW+ ">>>>>>>>>>>Arm active to pollinate mid."+ STYLE_RESET)
                # 对上方的花朵进行授粉
                # 如果动作不够，可以在这里加动作
                self.cmd_arm_msg.data = 8
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(self.go_along_time[self.task_A_go_along[i]][1])
                self.cmd_arm_msg.data = 3
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(self.go_along_time[self.task_A_go_along[i]][1])
                
                print(COLOR_YELLOW+ ">>>>>>>>>>>Arm active to pollinate bot."+ STYLE_RESET)
                # 对上方的花朵进行授粉
                # 如果动作不够，可以在这里加动作
                self.cmd_arm_msg.data = 9
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(self.go_along_time[self.task_A_go_along[i]][1])
                self.cmd_arm_msg.data = 3
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(self.go_along_time[self.task_A_go_along[i]][1])
                
                print(COLOR_YELLOW+ ">>>>>>>>>>>Arm reset."+ STYLE_RESET)
                # 机械臂恢复到初始位姿
                self.cmd_arm_msg.data = 1
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(self.go_along_time[self.task_A_go_along[i]][1])
            else:
                self.time_delay(self.go_along_time[self.task_A_go_along[i]][1])
        print(COLOR_YELLOW+ ">>>>>Task A finished." + STYLE_RESET)

    def task_B(self):

        print(COLOR_YELLOW+ ">>>>>Start Task B." + STYLE_RESET)
        for i in range(8):
            print(COLOR_YELLOW+ ">>>>>>>>>>>Task B No"+str(i+1) +" ..."+ STYLE_RESET)
            if i<5:
                self.cmd_vel_msg.linear.x = self.linear_x
                self.cmd_vel_msg.angular.z = 0
            elif i == 5:
                self.cmd_vel_msg.linear.x = 0
                self.cmd_vel_msg.angular.z = -self.anglular_z
            elif i == 6:
                self.cmd_vel_msg.linear.x = self.linear_x
                self.cmd_vel_msg.angular.z = 0
            elif i == 7:
                self.cmd_vel_msg.linear.x = 0
                self.cmd_vel_msg.angular.z = -self.anglular_z
            self.cmd_vel_pub.publish(self.cmd_vel_msg)
            print(COLOR_YELLOW+ ">>>>>>>>>>>Move. Speed is "+str(self.linear_x) +"m/s. Move time is "\
                                        +str(self.go_along_time[self.task_B_go_along[i]][0])+"s. Stop time is "\
                                            +str(self.go_along_time[self.task_B_go_along[i]][1])+"s."+ STYLE_RESET)
            
            # 可以尝试把延时换为激光雷达左右两侧距离数据的判断
            self.time_delay(self.go_along_time[self.task_B_go_along[i]][0])
            # 打印激光雷达两侧的距离数据，单位：m
            rospy.loginfo("Left Dis = %fm, Right Dis = %fm", self.min_dis[0], self.min_dis[1])

            self.cmd_vel_msg.linear.x = 0
            self.cmd_vel_msg.angular.z = 0
            self.cmd_vel_pub.publish(self.cmd_vel_msg)
            print(COLOR_YELLOW+ ">>>>>>>>>>>Stop."+ STYLE_RESET)
            
            if i<3:
                print(COLOR_YELLOW+ ">>>>>>>>>>>Arm active to detect No"+str(i+1)+" ahead."+ STYLE_RESET)
                # 机械臂识别第一根杆的正前方作物
                self.cmd_arm_msg.data = 15
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(self.go_along_time[self.task_B_go_along[i]][1])
                print(COLOR_YELLOW+ ">>>>>>>>>>>Start to detect. Times="+str(self.go_along_time[self.task_B_go_along[i]][1])+ STYLE_RESET)
                
                # 此处根据yolo识别结果，判断机械臂去哪里授粉
                print(COLOR_YELLOW+ ">>>>>>>>>>>Arm active to pollinate left."+ STYLE_RESET)
                # 对左侧的花朵进行授粉
                # 如果动作不够，可以在这里加动作
                self.cmd_arm_msg.data = 17
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(self.go_along_time[self.task_B_go_along[i]][1])
                self.cmd_arm_msg.data = 15
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(self.go_along_time[self.task_B_go_along[i]][1])
                
                print(COLOR_YELLOW+ ">>>>>>>>>>>Arm active to pollinate mid."+ STYLE_RESET)
                # 对中间的花朵进行授粉
                # 如果动作不够，可以在这里加动作
                self.cmd_arm_msg.data = 18
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(self.go_along_time[self.task_B_go_along[i]][1])
                self.cmd_arm_msg.data = 15
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(self.go_along_time[self.task_B_go_along[i]][1])
                
                print(COLOR_YELLOW+ ">>>>>>>>>>>Arm active to pollinate right."+ STYLE_RESET)
                # 对右侧的花朵进行授粉
                # 如果动作不够，可以在这里加动作
                self.cmd_arm_msg.data = 19
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(self.go_along_time[self.task_B_go_along[i]][1])
                self.cmd_arm_msg.data = 15
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(self.go_along_time[self.task_B_go_along[i]][1])
            if i>0 and i<4:
                print(COLOR_YELLOW+ ">>>>>>>>>>>Arm active to detect No"+str(i+1)+" back."+ STYLE_RESET)
                # 机械臂识别第一根杆的正后作物
                self.cmd_arm_msg.data = 16
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(self.go_along_time[self.task_B_go_along[i]][1])
                print(COLOR_YELLOW+ ">>>>>>>>>>>Start to detect. Times="+str(self.go_along_time[self.task_B_go_along[i]][1])+ STYLE_RESET)
                
                # 此处根据yolo识别结果，判断机械臂去哪里授粉
                print(COLOR_YELLOW+ ">>>>>>>>>>>Arm active to pollinate left."+ STYLE_RESET)
                # 对左侧的花朵进行授粉
                # 如果动作不够，可以在这里加动作
                self.cmd_arm_msg.data = 20
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(self.go_along_time[self.task_B_go_along[i]][1])
                self.cmd_arm_msg.data = 16
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(self.go_along_time[self.task_B_go_along[i]][1])
                
                print(COLOR_YELLOW+ ">>>>>>>>>>>Arm active to pollinate mid."+ STYLE_RESET)
                # 对中间的花朵进行授粉
                # 如果动作不够，可以在这里加动作
                self.cmd_arm_msg.data = 21
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(self.go_along_time[self.task_B_go_along[i]][1])
                self.cmd_arm_msg.data = 16
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(self.go_along_time[self.task_B_go_along[i]][1])
                
                print(COLOR_YELLOW+ ">>>>>>>>>>>Arm active to pollinate right."+ STYLE_RESET)
                # 对右侧的花朵进行授粉
                # 如果动作不够，可以在这里加动作
                self.cmd_arm_msg.data = 22
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(self.go_along_time[self.task_B_go_along[i]][1])
                self.cmd_arm_msg.data = 16
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(self.go_along_time[self.task_B_go_along[i]][1])
                
                print(COLOR_YELLOW+ ">>>>>>>>>>>Arm reset."+ STYLE_RESET)
                self.cmd_arm_msg.data = 1
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(self.go_along_time[self.task_B_go_along[i]][1])
            else:
                self.time_delay(self.go_along_time[self.task_B_go_along[i]][1])
        print(COLOR_YELLOW+ ">>>>>Task B finished." + STYLE_RESET)
    
    
    def task_C(self):

        print(COLOR_YELLOW+ ">>>>>Start Task C." + STYLE_RESET)
        for i in range(7):
            print(COLOR_YELLOW+ ">>>>>>>>>>>Task C No"+str(i+1) +" ..."+ STYLE_RESET)
            if i<4:
                self.cmd_vel_msg.linear.x = self.linear_x
                self.cmd_vel_msg.angular.z = 0
            elif i == 4:
                self.cmd_vel_msg.linear.x = 0
                self.cmd_vel_msg.angular.z = -self.anglular_z
            elif i == 5:
                self.cmd_vel_msg.linear.x = self.linear_x
                self.cmd_vel_msg.angular.z = 0
            elif i == 6:
                self.cmd_vel_msg.linear.x = 0
                self.cmd_vel_msg.angular.z = -self.anglular_z
            self.cmd_vel_pub.publish(self.cmd_vel_msg)
            print(COLOR_YELLOW+ ">>>>>>>>>>>Move. Speed is "+str(self.linear_x) +"m/s. Move time is "\
                                        +str(self.go_along_time[self.task_C_go_along[i]][0])+"s. Stop time is "\
                                            +str(self.go_along_time[self.task_C_go_along[i]][1])+"s."+ STYLE_RESET)

            # 可以尝试把延时换为激光雷达左右两侧距离数据的判断
            self.time_delay(self.go_along_time[self.task_C_go_along[i]][0])
            # 打印激光雷达两侧的距离数据，单位：m
            rospy.loginfo("Left Dis = %fm, Right Dis = %fm", self.min_dis[0], self.min_dis[1])

            self.cmd_vel_msg.linear.x = 0
            self.cmd_vel_msg.angular.z = 0
            self.cmd_vel_pub.publish(self.cmd_vel_msg)
            print(COLOR_YELLOW+ ">>>>>>>>>>>Stop."+ STYLE_RESET)
            if i<3:
                print(COLOR_YELLOW+ ">>>>>>>>>>>Arm active to detect left."+ STYLE_RESET)
                # 机械臂识别左侧作物
                self.cmd_arm_msg.data = 25
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(self.go_along_time[self.task_C_go_along[i]][1])
                print(COLOR_YELLOW+ ">>>>>>>>>>>Start to detect. Times="+str(self.go_along_time[self.task_C_go_along[i]][1])+ STYLE_RESET)
                
                # 此处根据yolo识别结果，判断机械臂去哪里授粉
                print(COLOR_YELLOW+ ">>>>>>>>>>>Arm active to pollinate left."+ STYLE_RESET)
                # 对左侧的花朵进行授粉
                # 如果动作不够，可以在这里加动作
                self.cmd_arm_msg.data = 27
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(self.go_along_time[self.task_C_go_along[i]][1])
                self.cmd_arm_msg.data = 25
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(self.go_along_time[self.task_C_go_along[i]][1])
                
                print(COLOR_YELLOW+ ">>>>>>>>>>>Arm active to pollinate mid."+ STYLE_RESET)
                # 对中间的花朵进行授粉
                # 如果动作不够，可以在这里加动作
                self.cmd_arm_msg.data = 28
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(self.go_along_time[self.task_C_go_along[i]][1])
                self.cmd_arm_msg.data = 25
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(self.go_along_time[self.task_C_go_along[i]][1])
                
                print(COLOR_YELLOW+ ">>>>>>>>>>>Arm active to pollinate right."+ STYLE_RESET)
                # 对右侧的花朵进行授粉
                # 如果动作不够，可以在这里加动作
                self.cmd_arm_msg.data = 29
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(self.go_along_time[self.task_C_go_along[i]][1])
                self.cmd_arm_msg.data = 25
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(self.go_along_time[self.task_C_go_along[i]][1])
                
                print(COLOR_YELLOW+ ">>>>>>>>>>>Arm active to detect right."+ STYLE_RESET)
                # 机械臂识别右侧作物
                self.cmd_arm_msg.data = 26
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(self.go_along_time[self.task_C_go_along[i]][1])
                print(COLOR_YELLOW+ ">>>>>>>>>>>Start to detect. Times="+str(self.go_along_time[self.task_C_go_along[i]][1])+ STYLE_RESET)
                
                # 此处根据yolo识别结果，判断机械臂去哪里授粉
                print(COLOR_YELLOW+ ">>>>>>>>>>>Arm active to pollinate left."+ STYLE_RESET)
                # 对左侧的花朵进行授粉
                # 如果动作不够，可以在这里加动作
                self.cmd_arm_msg.data = 30
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(self.go_along_time[self.task_C_go_along[i]][1])
                self.cmd_arm_msg.data = 26
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(self.go_along_time[self.task_C_go_along[i]][1])
                
                print(COLOR_YELLOW+ ">>>>>>>>>>>Arm active to pollinate mid."+ STYLE_RESET)
                # 对中间的花朵进行授粉
                # 如果动作不够，可以在这里加动作
                self.cmd_arm_msg.data = 31
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(self.go_along_time[self.task_C_go_along[i]][1])
                self.cmd_arm_msg.data = 26
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(self.go_along_time[self.task_C_go_along[i]][1])
                
                print(COLOR_YELLOW+ ">>>>>>>>>>>Arm active to pollinate right."+ STYLE_RESET)
                # 对右侧的花朵进行授粉
                # 如果动作不够，可以在这里加动作
                self.cmd_arm_msg.data = 32
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(self.go_along_time[self.task_C_go_along[i]][1])
                self.cmd_arm_msg.data = 26
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(self.go_along_time[self.task_C_go_along[i]][1])
                
                print(COLOR_YELLOW+ ">>>>>>>>>>>Arm reset."+ STYLE_RESET)
                # 机械臂恢复到初始位姿
                self.cmd_arm_msg.data = 1
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(self.go_along_time[self.task_C_go_along[i]][1])
            else:
                self.time_delay(self.go_along_time[self.task_C_go_along[i]][1])
        print(COLOR_YELLOW+ ">>>>>Task C finished." + STYLE_RESET)
    

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
       
        self.go_along_time = [[5.0,10.0],[5.0,10.0],[5.0,10.0]]  # 直接覆盖参数服务器中的值
        self.task_A_go_along = rospy.get_param("task_A_go_along",[1,2,2,2,2,2,3,4,5])
        self.task_B_go_along = rospy.get_param("task_B_go_along",[1,2,2,2,2,2,3,6])
        self.task_C_go_along = rospy.get_param("task_C_go_along",[1,2,2,2,2,2,3,4,5])
        
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic,Twist,queue_size=10)
        self.cmd_arm_pub = rospy.Publisher(self.cmd_arm_topic,Int32,queue_size=10)


        # 订阅雷达数据，获取左右两侧障碍物的最小距离
        self.left_dis_angle = 0
        self.right_dis_angle = 180
        self.min_dis = [10,10]
        rospy.Subscriber('/scan', LaserScan, self.get_laser_min_dis)

        self.cmd_vel_msg = Twist()
        self.linear_x = rospy.get_param('linear_x',0.2)
        self.anglular_z = rospy.get_param('anglular_z',2.5)
        self.cmd_arm_msg = Int32()

        #self.timer_vel = rospy.Timer(rospy.Duration(0.05),self.ljy_vel)###定时器发布所有速度
        


    def RobotStart(self):   
        #while not rospy.is_shutdown():        
        self.time_delay(5.0) 
        self.task_A()
        self.task_B()
        self.task_C()

#main function
if __name__=="__main__":
    try:
        rospy.init_node('compitation_node',anonymous=True)
        rospy.loginfo('compitation_irrigate_node instruction start...')
        MA = MOVE_ARRIVE()
        MA.RobotStart()
        rospy.spin()

    except KeyboardInterrupt:
        print("Shutting down")

