#! /usr/bin/env python
# -*- coding: utf-8 -*-
import os
import binascii
import time

from Cython.Plex.Actions import Return
from geometry_msgs.msg import Twist, Vector3
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_msgs.msg import Int8
from std_msgs.msg import Int32
from sensor_msgs.msg import LaserScan
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
from playsound import playsound
import actionlib
import cv2
import numpy as np
from matplotlib import pyplot as plt
from std_msgs.msg import  Float64
import socket
import ast
import pickle  # 用于反序列化数据
import matplotlib.patches as patches


buf_length = 11

RxBuff = [0]*buf_length

ACCData = [0.0]*8
GYROData = [0.0]*8
AngleData = [0.0]*8
FrameState = 0  # What is the state of the judgment
CheckSum = 0  # Sum check bit

start = 0 #帧头开始的标志
data_length = 0 #根据协议的文档长度为11 eg:55 51 31 FF 53 02 CD 07 12 0A 1B

acc = [0.0]*3
gyro = [0.0]*3
Angle = [0.0]*3

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

PI=3.14159
A=['qinweiganhan.wav','yibanganhan.wav','yanzhongganhan.wav']

#cv2.namedWindow('video', cv2.WINDOW_AUTOSIZE)
#cv2.resizeWindow('video',640,480)

# 请根据你的实际情况修改串口名称
SERIAL_PORT4 = '/dev/ttyUSB4'  # 这里是默认的串口名称，你可以用 dmesg | grep tty 查看实际的端口号
BAUD_RATE4 = 9600  # 波特率，和继电器的说明相符合

# 继电器的打开和关闭命令 (根据你的文档中的十六进制指令)
TURN_ON_RELAY = bytearray.fromhex('A0 01 01 A2')  # 打开继电器的HEX指令
TURN_OFF_RELAY = bytearray.fromhex('A0 01 00 A1')  # 关闭继电器的HEX指令

A_SPEED=0.20
A_huapen_thresh = 0.45#A赛道正常检测花盆的阈值
diyicizhuanwanqiandezhixing_time=4.9 #A区后区
diyicizhuanwanhoudezhixing_time=3.9
diyicizhuanwan_law=1.3#A区和B区第一个白色十字转弯角速度

#A区避障专用
#避障时的花盆检测阈值（小于正常花盆检测阈值，一般 = 正常阈值 - 横移阈值）
A_huapen_check_thresh = 0.3

#A_action_1为第一次转向
#障碍物在左边用的参数
A_avoid_obstacle_action1_left_vel = 0.0 #原地转向，线速度为0
A_avoid_obstacle_action1_left_law = 1.5 #用于避障的角速度，略快，弥补曲线多余的时间耗费
#障碍物在右边用的参数
A_avoid_obstacle_action1_right_vel = 0.0 #原地转向，线速度为0
A_avoid_obstacle_action1_right_law = 1.5  #用于避障的角速度，略快，弥补曲线多余的时间耗费

#A_action_2为第一次转向后直行
#障碍物在左边用的参数
A_avoid_obstacle_action2_left_vel = 0.15 #用于避障的线速度，略快，弥补曲线多余的时间耗费
A_avoid_obstacle_action2_left_law = 0.0 #直行，角速度为零
A_avoid_obstacle_action2_left_time = 1.1#用于决定直行距离的执行时间
#障碍物在右边用的参数
A_avoid_obstacle_action2_right_vel = 0.15 #用于避障的线速度，略快，弥补曲线多余的时间耗费
A_avoid_obstacle_action2_right_law = 0.0 #直行，角速度为零
A_avoid_obstacle_action2_right_time = 1.1#用于决定直行距离的执行时间

#A_action_3为第二次转向
#障碍物在左边用的参数
A_avoid_obstacle_action3_left_vel = 0.0 #原地转向，线速度为0
A_avoid_obstacle_action3_left_law = 1.5 #用于避障的角速度，略快，弥补曲线多余的时间耗费
#障碍物在右边用的参数
A_avoid_obstacle_action3_right_vel = 0.0 #原地转向，线速度为0
A_avoid_obstacle_action3_right_law = 1.5 #用于避障的角速度，略快，弥补曲线多余的时间耗费

#A_action_4为第二次转向后直行
#障碍物在左边用的参数
A_avoid_obstacle_action4_left_vel = 0.15 #用于避障的线速度，略快，弥补曲线多余的时间耗费
A_avoid_obstacle_action4_left_law = 0.0 #直行，角速度为零
A_avoid_obstacle_action4_left_time = 3.8#用于决定直行距离的执行时间
#障碍物在右边用的参数
A_avoid_obstacle_action4_right_vel = 0.15 #用于避障的线速度，略快，弥补曲线多余的时间耗费
A_avoid_obstacle_action4_right_law = 0.0 #直行，角速度为零
A_avoid_obstacle_action4_right_time = 3.8#用于决定直行距离的执行时间

#A_action_5为第三次转向
#障碍物在左边用的参数
A_avoid_obstacle_action5_left_vel = 0.0 #原地转向，线速度为0
A_avoid_obstacle_action5_left_law = 1.5 #用于避障的角速度，略快，弥补曲线多余的时间耗费
#A_障碍物在右边用的参数
A_avoid_obstacle_action5_right_vel = 0.0 #原地转向，线速度为0
A_avoid_obstacle_action5_right_law = 1.5 #用于避障的角速度，略快，弥补曲线多余的时间耗费

#A_action_6为第三次转向后直行
#障碍物在左边用的参数
A_avoid_obstacle_action6_left_vel = 0.1 #用于避障的线速度，略快，弥补曲线多余的时间耗费
A_avoid_obstacle_action6_left_law = 0.0 #直行，角速度为零
A_avoid_obstacle_action6_left_time = 1.5#用于决定直行距离的执行时间
#障碍物在右边用的参数
A_avoid_obstacle_action6_right_vel = 0.1 #用于避障的线速度，略快，弥补曲线多余的时间耗费
A_avoid_obstacle_action6_right_law = 0.0 #直行，角速度为零
A_avoid_obstacle_action6_right_time = 1.5#用于决定直行距离的执行时间

#A_action_7为最后一次转向
#障碍物在左边用的参数
A_avoid_obstacle_action7_left_vel = 0.0 #原地转向，线速度为0
A_avoid_obstacle_action7_left_law = 1.5 #用于避障的角速度，略快，弥补曲线多余的时间耗费
#障碍物在右边用的参数
A_avoid_obstacle_action7_right_vel = 0.0 #原地转向，线速度为0
A_avoid_obstacle_action7_right_law = 1.5 #用于避障的角速度，略快，弥补曲线多余的时间耗费


B_SPEED=0.20
B_huapen_thresh = 0.4#B赛道正常检测花盆的阈值
diercizhuanwanqiandezhixing_time=2.35#B区后区
diercizhuanwanhoudezhixing_time=4.0
diercizhuanwan_law=1.3#B区第二个白色十字和C区第一个白色十字转弯角速度

#B区避障专用
#避障时的花盆检测阈值（小于正常花盆检测阈值，一般 = 正常阈值 - 横移阈值）
B_huapen_check_thresh = 0.25

#B_action_1为第一次转向
#障碍物在左边用的参数
B_avoid_obstacle_action1_left_vel = 0.0 #原地转向，线速度为0
B_avoid_obstacle_action1_left_law = 1.3 #用于避障的角速度，略快，弥补曲线多余的时间耗费
#障碍物在右边用的参数
B_avoid_obstacle_action1_right_vel = 0.0 #原地转向，线速度为0
B_avoid_obstacle_action1_right_law = 1.3 #用于避障的角速度，略快，弥补曲线多余的时间耗费

#B_action_2为第一次转向后直行
#障碍物在左边用的参数
B_avoid_obstacle_action2_left_vel = 0.15 #用于避障的线速度，略快，弥补曲线多余的时间耗费
B_avoid_obstacle_action2_left_law = 0.0 #直行，角速度为零
B_avoid_obstacle_action2_left_time = 1.1#用于决定直行距离的执行时间
#障碍物在右边用的参数
B_avoid_obstacle_action2_right_vel = 0.15 #用于避障的线速度，略快，弥补曲线多余的时间耗费
B_avoid_obstacle_action2_right_law = 0.0 #直行，角速度为零
B_avoid_obstacle_action2_right_time = 1.1#用于决定直行距离的执行时间

#B_action_3为第二次转向
#障碍物在左边用的参数
B_avoid_obstacle_action3_left_vel = 0.0 #原地转向，线速度为0
B_avoid_obstacle_action3_left_law = 1.3 #用于避障的角速度，略快，弥补曲线多余的时间耗费
#障碍物在右边用的参数
B_avoid_obstacle_action3_right_vel = 0.0 #原地转向，线速度为0
B_avoid_obstacle_action3_right_law = 1.3 #用于避障的角速度，略快，弥补曲线多余的时间耗费

#B_action_4为第二次转向后直行
#障碍物在左边用的参数
B_avoid_obstacle_action4_left_vel = 0.15 #用于避障的线速度，略快，弥补曲线多余的时间耗费
B_avoid_obstacle_action4_left_law = 0.0 #直行，角速度为零
B_avoid_obstacle_action4_left_time = 3.8#用于决定直行距离的执行时间
#障碍物在右边用的参数
B_avoid_obstacle_action4_right_vel = 0.15 #用于避障的线速度，略快，弥补曲线多余的时间耗费
B_avoid_obstacle_action4_right_law = 0.0 #直行，角速度为零
B_avoid_obstacle_action4_right_time = 3.8#用于决定直行距离的执行时间

#B_action_5为第三次转向
#障碍物在左边用的参数
B_avoid_obstacle_action5_left_vel = 0.0 #原地转向，线速度为0
B_avoid_obstacle_action5_left_law = 1.3 #用于避障的角速度，略快，弥补曲线多余的时间耗费
#B_障碍物在右边用的参数
B_avoid_obstacle_action5_right_vel = 0.0 #原地转向，线速度为0
B_avoid_obstacle_action5_right_law = 1.3 #用于避障的角速度，略快，弥补曲线多余的时间耗费

#B_action_6为第三次转向后直行
#障碍物在左边用的参数
B_avoid_obstacle_action6_left_vel = 0.1 #用于避障的线速度，略快，弥补曲线多余的时间耗费
B_avoid_obstacle_action6_left_law = 0.0 #直行，角速度为零
B_avoid_obstacle_action6_left_time = 1.5#用于决定直行距离的执行时间
#障碍物在右边用的参数
B_avoid_obstacle_action6_right_vel = 0.1 #用于避障的线速度，略快，弥补曲线多余的时间耗费
B_avoid_obstacle_action6_right_law = 0.0 #直行，角速度为零
B_avoid_obstacle_action6_right_time = 1.5#用于决定直行距离的执行时间

#B_action_7为最后一次转向
#障碍物在左边用的参数
B_avoid_obstacle_action7_left_vel = 0.0 #原地转向，线速度为0
B_avoid_obstacle_action7_left_law = 1.3 #用于避障的角速度，略快，弥补曲线多余的时间耗费
#障碍物在右边用的参数
B_avoid_obstacle_action7_right_vel = 0.0 #原地转向，线速度为0
B_avoid_obstacle_action7_right_law = 1.3 #用于避障的角速度，略快，弥补曲线多余的时间耗费

class PID:#只是对角度的PID控制（只负责直行保持，不负责原地转向）
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.last_error1 = 0
        self.last_error2 = 0
        self.last_error3 = 0
        self.integral = 0

    def update1(self,want_yaw,current_value):#走A和C区的直线以及上方的转弯区域适用该函数,A区用0,C区用-2.0
        error = math.radians(want_yaw - current_value)
        self.integral = 0
        self.integral += error
        derivative = error - self.last_error1
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.last_error1 = error
        return output

    def update2(self,want_yaw,current_value):#走B区直线适用该函数B角度用-180
        if current_value>0 and current_value<180:     
            error = math.radians(abs(want_yaw) - current_value)
            self.integral = 0
            self.integral += error
            derivative = error - self.last_error2
            output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
            self.last_error2 = error
        else:
            error = math.radians(want_yaw - current_value)
            self.integral = 0
            self.integral += error
            derivative = error - self.last_error2
            output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
            self.last_error2 = error
        return output
    
    def update3(self,want_yaw,current_value):#走下方的转弯适用该函数,want_yaw取-90.0度
        if current_value>0 and current_value<180:     
            error = math.radians(current_value-abs(want_yaw))
            self.integral = 0
            self.integral += error
            derivative = error - self.last_error3
            output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
            self.last_error3 = error
        else:
            error = math.radians(want_yaw - current_value)
            self.integral = 0
            self.integral += error
            derivative = error - self.last_error3
            output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
            self.last_error3 = error
        return output
    
    def update4(self,want_yaw,current_value):#走D区的特殊纠偏适用该函数,want_yaw范围是0--180,取大概-178.0
        if current_value>0 and current_value>abs(want_yaw):     
            error = math.radians(current_value-abs(want_yaw))
            self.integral = 0
            self.integral += error
            derivative = error - self.last_error2
            output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
            self.last_error2 = error
        elif current_value>0 and current_value<abs(want_yaw):
            error = math.radians(abs(want_yaw)-current_yaw)
            self.integral = 0
            self.integral += error
            derivative = error - self.last_error2
            output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
            self.last_error2 = error
        else:
            error = math.radians(want_yaw - current_value)
            self.integral = 0
            self.integral += error
            derivative = error - self.last_error2
            output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
            self.last_error2 = error
        return output    

class MOVE_ARRIVE:

    def time_delay(self,s):
        start_time = rospy.get_time()
        while (rospy.get_time() - start_time<s):
            xyz=0

    def get_acc(self,datahex):
        axl = datahex[0]
        axh = datahex[1]
        ayl = datahex[2]
        ayh = datahex[3]
        azl = datahex[4]
        azh = datahex[5]
        k_acc = 16.0
        acc_x = (axh << 8 | axl) / 32768.0 * k_acc
        acc_y = (ayh << 8 | ayl) / 32768.0 * k_acc
        acc_z = (azh << 8 | azl) / 32768.0 * k_acc
        if acc_x >= k_acc:
            acc_x -= 2 * k_acc
        if acc_y >= k_acc:
            acc_y -= 2 * k_acc
        if acc_z >= k_acc:
            acc_z -= 2 * k_acc
        return acc_x, acc_y, acc_z

    def get_gyro(self,datahex):
        wxl = datahex[0]
        wxh = datahex[1]
        wyl = datahex[2]
        wyh = datahex[3]
        wzl = datahex[4]
        wzh = datahex[5]
        k_gyro = 2000.0
        gyro_x = (wxh << 8 | wxl) / 32768.0 * k_gyro
        gyro_y = (wyh << 8 | wyl) / 32768.0 * k_gyro
        gyro_z = (wzh << 8 | wzl) / 32768.0 * k_gyro
        if gyro_x >= k_gyro:
            gyro_x -= 2 * k_gyro
        if gyro_y >= k_gyro:
            gyro_y -= 2 * k_gyro
        if gyro_z >= k_gyro:
            gyro_z -= 2 * k_gyro
        return gyro_x, gyro_y, gyro_z

    def get_angle(self,datahex):
        rxl = datahex[0]
        rxh = datahex[1]
        ryl = datahex[2]
        ryh = datahex[3]
        rzl = datahex[4]
        rzh = datahex[5]
        k_angle = 180.0
        angle_x = (rxh << 8 | rxl) / 32768.0 * k_angle
        angle_y = (ryh << 8 | ryl) / 32768.0 * k_angle
        angle_z = (rzh << 8 | rzl) / 32768.0 * k_angle
        if angle_x >= k_angle:
            angle_x -= 2 * k_angle
        if angle_y >= k_angle:
            angle_y -= 2 * k_angle
        if angle_z >= k_angle:
            angle_z -= 2 * k_angle
        return angle_x, angle_y, angle_z

    def GetDataDeal(self,list_buf):
        global acc,gyro,Angle
        if(list_buf[buf_length - 1] != CheckSum): #校验码不正确
            return
        if(list_buf[1] == 0x51): #加速度输出
            for i in range(6): 
                ACCData[i] = list_buf[2+i] #有效数据赋值
            acc = self.get_acc(ACCData)
        elif(list_buf[1] == 0x52): #角速度输出
            for i in range(6): 
                GYROData[i] = list_buf[2+i] #有效数据赋值
            gyro = self.get_gyro(GYROData)
        elif(list_buf[1] == 0x53): #姿态角度输出
            for i in range(6): 
                AngleData[i] = list_buf[2+i] #有效数据赋值
            Angle = self.get_angle(AngleData)
        #print("acc:%10.3f %10.3f %10.3f \n" % (acc[0],acc[1],acc[2]))
        #print("gyro:%10.3f %10.3f %10.3f \n" % (gyro[0],gyro[1],gyro[2]))
        #print("angle:%10.3f %10.3f %10.3f \n" % (Angle[0],Angle[1],Angle[2]))

    def DueData(self,inputdata):  # New core procedures, read the data partition, each read to the corresponding array 
        global start
        global CheckSum
        global data_length
        # print(type(inputdata))
        if inputdata == 0x55 and start == 0:
            start = 1
            data_length = 11
            CheckSum = 0
            #清0
            for i in range(11):
                RxBuff[i] = 0
        if start == 1:
            CheckSum += inputdata #校验码计算 会把校验位加上
            RxBuff[buf_length-data_length] = inputdata #保存数据
            data_length = data_length - 1 #长度减一
            if data_length == 0: #接收到完整的数据
                CheckSum = (CheckSum-inputdata) & 0xff 
                start = 0 #清0
                self.GetDataDeal(RxBuff)  #处理数据

    def read_imu_data(self):
        RXdata = self.imu_serial.read(1)
        # 如果 RXdata 是字节对象，使用 binascii 转换为十六进制字符串
        if isinstance(RXdata, str):
            RXdata = int(binascii.hexlify(RXdata), 16)  # 使用 binascii 模块进行转换
        else:
            raise TypeError("RXdata 不是字节对象")
        self.DueData(RXdata)
        return Angle[2],Angle[1]     #返回当前角度

    def receive_data(self):
        host = '0.0.0.0'  # 监听所有接口
        port = 8080        # 和树莓派的端口号保持一致
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.bind((host, port))
        server_socket.settimeout(None)  # 设置为无限等待
        server_socket.listen(1)

        print("等待树莓派发送数据...")
        client_socket, addr = server_socket.accept()
        print("接收到来自 {} 的连接".format(addr))

        # 接收数据并反序列化
        data = client_socket.recv(1024)  # 不进行解码，直接接收原始二进制数据
        data_tuple = pickle.loads(data)  # 使用pickle进行反序列化

        client_socket.close()
        server_socket.close()
        return  data_tuple

    def dotime_stop(self):#小车停下来的指令
        self.cmd_vel_msg.angular.z = 0.0
        self.cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_pub.publish(self.cmd_vel_msg)
        self.time_delay(0.2)
        self.cmd_vel_msg.angular.z = 0.0
        self.cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_pub.publish(self.cmd_vel_msg)# 避障过程中特殊的花盆检测函数
    def A_huapen_dis_check(self):#避障过程中特殊的花盆检测函数
        #只要本函数被运行，则self.avoidance_direction必然是1 or -1
        if self.avoidance_direction == 1:#左侧有障碍物，检测右侧花盆距离
            if self.min_dis[1]<A_huapen_check_thresh:#A区避障时检测阈值，小于正常花盆检测阈值，一般等于"正常阈值 - 横移阈值"
                #返回信号“检测到花盆！！！”
                return True
            else:
                return False
        elif self.avoidance_direction == -1:#右侧有障碍物，左绕避障
            #检测左侧花盆距离
            if self.min_dis[0]<A_huapen_check_thresh:
                #返回信号“检测到花盆！！！”
                return True
            else:
                return False
        return None

    def A_arm_execute(self):
        print("...............arm_execute..............")
        print("Before：self.current_arm_index = ",self.current_arm_index)
        print("检查是否正在避障：")
        if self.avoidance_c_time ==0 :
            print("未进行避障")
            print(COLOR_YELLOW + ">>>>>>>>>>>Arm active first." + STYLE_RESET)
            self.cmd_arm_msg.data = 3#标准情况下A赛道的向左摆动机械臂动作组
            self.cmd_arm_pub.publish(self.cmd_arm_msg)
            self.time_delay(1.0)
            print(COLOR_YELLOW + ">>>>>>>>>>>Pump work. Times=" + str(self.area_A_dry[self.current_arm_index][0]) + STYLE_RESET)
            playsound(A[self.area_A_dry[self.current_arm_index][0] - 1])
            self.control_irrigation(self.pump_on_time, self.pump_off_time, self.area_A_dry[self.current_arm_index][0])
            self.time_delay(1.0)
            self.cmd_arm_msg.data = 2#机械臂复位
            self.cmd_arm_pub.publish(self.cmd_arm_msg)
            self.time_delay(5.0)
            print(COLOR_YELLOW + ">>>>>>>>>>>Arm active second." + STYLE_RESET)
            self.cmd_arm_msg.data = 4#标准情况下A赛道的向右摆动机械臂动作组
            self.cmd_arm_pub.publish(self.cmd_arm_msg)
            self.time_delay(2.0)
            print(COLOR_YELLOW + ">>>>>>>>>>>Pump work. Times=" + str(self.area_A_dry[self.current_arm_index][1]) + STYLE_RESET)
            self.control_irrigation(self.pump_on_time, self.pump_off_time, self.area_A_dry[self.current_arm_index][1])
            self.time_delay(1.0)
            print(COLOR_YELLOW + ">>>>>>>>>>>Arm reset." + STYLE_RESET)
            self.cmd_arm_msg.data = 2#机械臂复位
            self.cmd_arm_pub.publish(self.cmd_arm_msg)
            self.time_delay(2.0)

        elif self.avoidance_c_time ==  1:#标准避障情况下，需额外检验左避障还是右避障
            print("正在避障")
            print(COLOR_YELLOW + ">>>>>>>>>>>Arm active first." + STYLE_RESET)
            #先向左摆动
            if self.avoidance_direction == 1 :#左侧有障碍物，向右避障时
                print("子状态:左侧有障碍物，向右避障")
                print("机械臂:A_左摆动伸长")
                self.cmd_arm_msg.data = 5 #A_左摆动伸长
            elif self.avoidance_direction == -1 :#右侧有障碍物，向左避障时
                print("子状态:右侧有障碍物，向左避障")
                print("机械臂:A_左摆动缩短")
                self.cmd_arm_msg.data = 6 #A_左摆动缩短
            self.cmd_arm_pub.publish(self.cmd_arm_msg)
            self.time_delay(1.0)
            print(COLOR_YELLOW + ">>>>>>>>>>>Pump work. Times=" + str(self.area_A_dry[self.current_arm_index][0]) + STYLE_RESET)
            playsound(A[self.area_A_dry[self.current_arm_index][0] - 1])
            self.control_irrigation(self.pump_on_time, self.pump_off_time, self.area_A_dry[self.current_arm_index][0])
            self.time_delay(1.0)
            self.cmd_arm_msg.data = 2#机械臂复位
            self.cmd_arm_pub.publish(self.cmd_arm_msg)
            self.time_delay(5.0)
            print(COLOR_YELLOW + ">>>>>>>>>>>Arm active second." + STYLE_RESET)
            if self.avoidance_direction == 1:  # 左侧有障碍物，向右避障时
                print("子状态:左侧有障碍物，向右避障")
                print("机械臂:A_右摆动缩短")
                self.cmd_arm_msg.data = 8 #A_右摆动缩短_动作组
            elif self.avoidance_direction == -1:  # 右侧有障碍物，向左避障时
                print("子状态:右侧有障碍物，向左避障")
                print("机械臂:A_右摆动伸长")
                self.cmd_arm_msg.data = 7 #A_右摆动伸长_浇水动作组
            self.cmd_arm_pub.publish(self.cmd_arm_msg)
            self.time_delay(2.0)
            print(COLOR_YELLOW + ">>>>>>>>>>>Pump work. Times=" + str(self.area_A_dry[self.current_arm_index][1]) + STYLE_RESET)
            self.control_irrigation(self.pump_on_time, self.pump_off_time, self.area_A_dry[self.current_arm_index][1])
            self.time_delay(1.0)
            print(COLOR_YELLOW + ">>>>>>>>>>>Arm reset." + STYLE_RESET)
            self.cmd_arm_msg.data = 2 #机械臂复位
            self.cmd_arm_pub.publish(self.cmd_arm_msg)
            self.time_delay(2.0)

        self.current_arm_index += 1#直行之后，标志位+1
        print("After：self.current_arm_index = ",self.current_arm_index)
        print("..................arm_execute Finished!!!!......................")




    def A_avoid_action_1(self):#原地右转
        global want_yaw, current_yaw, scope_law
        want_yaw = -90.0*self.avoidance_direction#左侧有障碍物（self.avoidance_direction=1），右画弧
        print("避障ing......Action1")  # 任务进程
        if self.avoidance_direction >0 :#左侧障碍，原地右转
            while True:  # 永循环，直到误差abs(current_yaw-want_yaw)<2.5
                current_yaw, scope_law = self.read_imu_data()
                if current_yaw > 0.0:#0度朝向偏左，顺时针
                    self.cmd_vel_msg.angular.z = -A_avoid_obstacle_action1_left_law
                    self.cmd_vel_msg.linear.x = A_avoid_obstacle_action1_left_vel
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                elif current_yaw < 0.0 and current_yaw >= want_yaw + 5.0:#0度朝向偏右，或转向过程中偏到目标角度范围正方向以外，顺时针转
                    self.cmd_vel_msg.angular.z = -A_avoid_obstacle_action1_left_law
                    self.cmd_vel_msg.linear.x = A_avoid_obstacle_action1_left_vel
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                elif current_yaw <= want_yaw - 5.0:#转向过程中偏到目标角度范围负方向以外，逆时针转
                    self.cmd_vel_msg.angular.z = A_avoid_obstacle_action1_left_law
                    self.cmd_vel_msg.linear.x = A_avoid_obstacle_action1_left_vel
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                else:#在want_yaw上下5度以内，开始原地微调，微调是顺时针还是逆时针？？？
                    #print("我在微调...")  # 验证是否会困于微调函数
                    self.cmd_vel_msg.angular.z = A_avoid_obstacle_action1_left_law#此时原地微调
                    self.cmd_vel_msg.linear.x = 0.0#应该继续直行还是直接停住？？？
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                if abs(current_yaw - want_yaw) < 2.5:
                    self.cmd_vel_msg.angular.z = 0.0
                    self.cmd_vel_msg.linear.x = 0.0
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                    break
                else:
                    continue
        elif self.avoidance_direction <0 :#右侧障碍，原地左转到90度
            while True:  # 永循环，直到误差abs(current_yaw-want_yaw)<2.5
                current_yaw, scope_law = self.read_imu_data()
                if current_yaw < 0.0:#0度朝向偏右，逆时针
                    self.cmd_vel_msg.angular.z = A_avoid_obstacle_action1_right_law
                    self.cmd_vel_msg.linear.x = A_avoid_obstacle_action1_right_vel
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                elif current_yaw > 0.0 and current_yaw <= want_yaw - 5.0:#0度朝向偏左，或转向过程中偏到目标角度范围负方向以外，逆时针转
                    self.cmd_vel_msg.angular.z = A_avoid_obstacle_action1_right_law
                    self.cmd_vel_msg.linear.x = A_avoid_obstacle_action1_right_vel
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                elif current_yaw >= want_yaw + 5.0:#转向过程中偏到目标角度范围正方向以外，顺时针转
                    self.cmd_vel_msg.angular.z = -A_avoid_obstacle_action1_right_law
                    self.cmd_vel_msg.linear.x = A_avoid_obstacle_action1_right_vel
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                else:#在want_yaw上下5度以内，开始原地微调，微调是顺时针还是逆时针？？？
                    #print("我在微调...")#验证是否会困于微调函数
                    self.cmd_vel_msg.angular.z = A_avoid_obstacle_action1_right_law#此时原地微调
                    self.cmd_vel_msg.linear.x = 0.0#应该继续直行还是直接停住？？？
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                if abs(current_yaw - want_yaw) < 2.5:
                    self.cmd_vel_msg.angular.z = 0.0
                    self.cmd_vel_msg.linear.x = 0.0
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                    break
                else:
                    continue
    def A_avoid_action_2(self):#第一次转弯后直行
        print("避障ing......Action2")  # 任务进程
        global want_yaw, current_yaw, start_time1, scope_law
        want_yaw = -90.0*self.avoidance_direction
        start_time1 = rospy.get_time()
        if self.avoidance_direction > 0:  # 左侧障碍
            while True:
                current_yaw, scope_law = self.read_imu_data()
                pid_output = self.PID1.update1(want_yaw, current_yaw)#PID1专用于A区浇水或避障的PID
                self.cmd_vel_msg.angular.z = pid_output
                self.cmd_vel_msg.linear.x = A_avoid_obstacle_action2_left_vel
                self.cmd_vel_pub.publish(self.cmd_vel_msg)
                if rospy.get_time() - start_time1 > (A_avoid_obstacle_action2_left_time*self.avoidance_c_time):#与放大后的时间比较
                    break
                else:
                    continue
        elif self.avoidance_direction < 0:  # 右侧障碍
            while True:
                current_yaw, scope_law = self.read_imu_data()
                pid_output = self.PID1.update1(want_yaw, current_yaw)#PID1专用于A区浇水或避障的PID
                self.cmd_vel_msg.angular.z = pid_output
                self.cmd_vel_msg.linear.x = A_avoid_obstacle_action2_right_vel
                self.cmd_vel_pub.publish(self.cmd_vel_msg)
                if rospy.get_time() - start_time1 > (A_avoid_obstacle_action2_right_time*self.avoidance_c_time):#与放大后的时间比较
                    break
                else:
                    continue

    def A_avoid_action_3(self):#转回0度
        global want_yaw, current_yaw, scope_law
        want_yaw = 0
        print("避障ing......Action3")  # 任务进程
        if self.avoidance_direction > 0:  # 左侧障碍
            while True:
                current_yaw, scope_law = self.read_imu_data()
                if current_yaw < want_yaw:
                    self.cmd_vel_msg.angular.z = A_avoid_obstacle_action3_left_law
                    self.cmd_vel_msg.linear.x = A_avoid_obstacle_action3_left_vel
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                elif current_yaw > want_yaw:
                    self.cmd_vel_msg.angular.z = -A_avoid_obstacle_action3_left_law
                    self.cmd_vel_msg.linear.x = A_avoid_obstacle_action3_left_vel
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                else:
                    self.cmd_vel_msg.angular.z = 0.0
                    self.cmd_vel_msg.linear.x = 0.0
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                if abs(current_yaw - want_yaw) < 2.5:
                    self.cmd_vel_msg.angular.z = 0.0
                    self.cmd_vel_msg.linear.x = 0.0
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                    break
                else:
                    continue
        elif self.avoidance_direction < 0:  # 右侧障碍
            while True:
                current_yaw, scope_law = self.read_imu_data()
                if current_yaw < want_yaw:
                    self.cmd_vel_msg.angular.z = A_avoid_obstacle_action3_right_law
                    self.cmd_vel_msg.linear.x = A_avoid_obstacle_action3_right_vel
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                elif current_yaw > want_yaw:
                    self.cmd_vel_msg.angular.z = -A_avoid_obstacle_action3_right_law
                    self.cmd_vel_msg.linear.x = A_avoid_obstacle_action3_right_vel
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                else:
                    self.cmd_vel_msg.angular.z = 0.0
                    self.cmd_vel_msg.linear.x = 0.0
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                if abs(current_yaw - want_yaw) < 2.5:
                    self.cmd_vel_msg.angular.z = 0.0
                    self.cmd_vel_msg.linear.x = 0.0
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                    break
                else:
                    continue

    def A_avoid_action_4(self):#第二次转弯后直行
        print("避障ing......Action4")  # 任务进程
        global want_yaw, current_yaw, start_time1, scope_law
        want_yaw = 0.0
        t=0.0 #存储：避障过程浇水占用了多少时间

        # 设置标志位：避障过程检测到花盆,停车并执行浇水
        # 供execute_avoidance()使用
        tag = False

        # action4中限制最多执行一次浇水
        once_tag = 0#标志action4中有没有执行过浇水

        start_time1 = rospy.get_time()
        if self.avoidance_direction > 0:  # 左侧障碍
            while True:
                current_yaw, scope_law = self.read_imu_data()
                pid_output = self.PID1.update1(want_yaw, current_yaw)  # PID1专用于A区浇水或避障的PID
                self.cmd_vel_msg.angular.z = pid_output
                self.cmd_vel_msg.linear.x = A_avoid_obstacle_action4_left_vel
                self.cmd_vel_pub.publish(self.cmd_vel_msg)

                # 如果避障过程中浇水了，必然占用直行时间，耽误直行距离
                # 解决方法：记录执行避障中处理花盆浇水的函数运行所需要的时间（运行了则在函数内部用两次rospy.get_time()标记时间起始点，没运行则时间为0.0s）

                if self.A_huapen_dis_check() and once_tag == 0:#如果避障过程检测到花盆,并且action4中尚未浇水#：停车并执行浇水
                    #停车
                    print("避障中检测到花盆！！！！")
                    print("停车")
                    self.dotime_stop()
                    #执行避障中处理花盆浇水的函数
                    #获取起始时间
                    t1=rospy.get_time()
                    # 执行避障中处理花盆浇水的函数
                    print("执行机械臂动作组ing......")
                    print("Start：", t1)
                    self.A_arm_execute()
                    # 获取结束时间
                    t2= rospy.get_time()
                    print("Finished", t2)
                    #赋值，占用了多少时间
                    t=t2-t1
                    print("Total Time", t)
                    #设置标志位：避障过程检测到花盆,停车并执行浇水
                    #供execute_avoidance()使用
                    tag = True
                    once_tag = 1 #标志位，表示action4中仅限一次的浇水机会用完了
                    print("once_tag = 1,表示action4中仅限一次的浇水机会用完了")
                    #!!!最后的判断相应变成if rospy.get_time() -Spend_Time - start_time1 > A_avoid_obstacle_action4_time
                if rospy.get_time() -t- start_time1 > A_avoid_obstacle_action4_left_time :  # 此段路程不用系数放大A_avoid_obstacle_action4_time
                    print("rospy.get_time() -t- start_time1 =",rospy.get_time() -t- start_time1)
                    print("A_avoid_obstacle_action4_time:",A_avoid_obstacle_action4_left_time)
                    print("rospy.get_time() -t- start_time1 > A_avoid_obstacle_action4_time")
                    print("跳出while循环")
                    break
                else:
                    continue
        elif self.avoidance_direction < 0:  # 右侧障碍
            while True:
                current_yaw, scope_law = self.read_imu_data()
                pid_output = self.PID1.update1(want_yaw, current_yaw)  # PID1专用于A区浇水或避障的PID
                self.cmd_vel_msg.angular.z = pid_output
                self.cmd_vel_msg.linear.x = A_avoid_obstacle_action4_right_vel
                self.cmd_vel_pub.publish(self.cmd_vel_msg)

                # 如果避障过程中浇水了，必然占用直行时间，耽误直行距离
                # 解决方法：记录执行避障中处理花盆浇水的函数运行所需要的时间（运行了则在函数内部用两次rospy.get_time()标记时间起始点，没运行则时间为0.0s）

                if self.A_huapen_dis_check() and once_tag == 0:  # 如果避障过程检测到花盆,并且action4中尚未浇水#：停车并执行浇水
                    # 停车
                    print("避障中检测到花盆！！！！")
                    print("停车")
                    self.dotime_stop()
                    # 执行避障中处理花盆浇水的函数
                    # 获取起始时间
                    t1 = rospy.get_time()
                    # 执行避障中处理花盆浇水的函数
                    print("执行机械臂动作组ing......")
                    print("Start：", t1)
                    self.A_arm_execute()
                    # 获取结束时间
                    t2 = rospy.get_time()
                    print("Finished", t2)
                    # 赋值，占用了多少时间
                    t = t2 - t1
                    print("Total Time", t)
                    # 设置标志位：避障过程检测到花盆,停车并执行浇水
                    # 供execute_avoidance()使用
                    tag = True
                    once_tag = 1  # 标志位，表示action4中仅限一次的浇水机会用完了
                    print("once_tag = 1,表示action4中仅限一次的浇水机会用完了")
                    # !!!最后的判断相应变成if rospy.get_time() -Spend_Time - start_time1 > A_avoid_obstacle_action4_time
                if rospy.get_time() - t - start_time1 > A_avoid_obstacle_action4_right_time:  # 此段路程不用系数放大A_avoid_obstacle_action4_time
                    print("rospy.get_time() -t- start_time1 =", rospy.get_time() - t - start_time1)
                    print("A_avoid_obstacle_action4_time:", A_avoid_obstacle_action4_right_time)
                    print("rospy.get_time() -t- start_time1 > A_avoid_obstacle_action4_time")
                    print("跳出while循环")
                    break
                else:
                    continue

        print("是否避章时停车浇水.....",tag)
        return tag#供execute_avoidance()使用

    def A_avoid_action_5(self):#第三次转弯
        global want_yaw, current_yaw, scope_law
        want_yaw = 90.0 * self.avoidance_direction  # 左侧有障碍物（self.avoidance_direction=1），右画弧
        print("避障ing......Action5")  # 任务进程
        if self.avoidance_direction < 0:  # 左侧障碍，原地右转
            while True:  # 永循环，直到误差abs(current_yaw-want_yaw)<2.5
                current_yaw, scope_law = self.read_imu_data()
                if current_yaw > 0.0:  # 0度朝向偏左，顺时针
                    self.cmd_vel_msg.angular.z = -A_avoid_obstacle_action5_left_law
                    self.cmd_vel_msg.linear.x = A_avoid_obstacle_action5_left_vel
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                elif current_yaw < 0.0 and current_yaw >= want_yaw + 5.0:  # 0度朝向偏右，或转向过程中偏到目标角度范围正方向以外，顺时针转
                    self.cmd_vel_msg.angular.z = -A_avoid_obstacle_action5_left_law
                    self.cmd_vel_msg.linear.x = A_avoid_obstacle_action5_left_vel
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                elif current_yaw <= want_yaw - 5.0:  # 转向过程中偏到目标角度范围负方向以外，逆时针转
                    self.cmd_vel_msg.angular.z = A_avoid_obstacle_action5_left_law
                    self.cmd_vel_msg.linear.x = A_avoid_obstacle_action5_left_vel
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                else:  # 在want_yaw上下5度以内，开始原地微调，微调是顺时针还是逆时针？？？
                    self.cmd_vel_msg.angular.z = A_avoid_obstacle_action5_left_law  # 此时原地微调
                    self.cmd_vel_msg.linear.x = 0.0  # 应该继续直行还是直接停住？？？
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                if abs(current_yaw - want_yaw) < 2.5:
                    self.cmd_vel_msg.angular.z = 0.0
                    self.cmd_vel_msg.linear.x = 0.0
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                    break
                else:
                    continue
        elif self.avoidance_direction > 0:  # 右侧障碍，原地左转到90度
            while True:  # 永循环，直到误差abs(current_yaw-want_yaw)<2.5
                current_yaw, scope_law = self.read_imu_data()
                if current_yaw < 0.0:  # 0度朝向偏右，逆时针
                    self.cmd_vel_msg.angular.z = A_avoid_obstacle_action5_right_law
                    self.cmd_vel_msg.linear.x = A_avoid_obstacle_action5_right_vel
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                elif current_yaw > 0.0 and current_yaw <= want_yaw - 5.0:  # 0度朝向偏左，或转向过程中偏到目标角度范围负方向以外，逆时针转
                    self.cmd_vel_msg.angular.z = A_avoid_obstacle_action5_right_law
                    self.cmd_vel_msg.linear.x = A_avoid_obstacle_action5_right_vel
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                elif current_yaw >= want_yaw + 5.0:  # 转向过程中偏到目标角度范围正方向以外，顺时针转
                    self.cmd_vel_msg.angular.z = -A_avoid_obstacle_action5_right_law
                    self.cmd_vel_msg.linear.x = A_avoid_obstacle_action5_right_vel
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                else:  # 在want_yaw上下5度以内，开始原地微调，微调是顺时针还是逆时针？？？
                    self.cmd_vel_msg.angular.z = A_avoid_obstacle_action5_right_law  # 此时原地微调
                    self.cmd_vel_msg.linear.x = 0.0  # 应该继续直行还是直接停住？？？
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                if abs(current_yaw - want_yaw) < 2.5:
                    self.cmd_vel_msg.angular.z = 0.0
                    self.cmd_vel_msg.linear.x = 0.0
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                    break
                else:
                    continue
    def A_avoid_action_6(self):  # 第三次转弯后的直行
        print("避障ing......Action6")  # 任务进程
        global want_yaw, current_yaw, start_time1, scope_law
        want_yaw = 90.0 * self.avoidance_direction
        start_time1 = rospy.get_time()
        if self.avoidance_direction > 0:  # 左侧障碍，原地右转
            while True:
                current_yaw, scope_law = self.read_imu_data()
                pid_output = self.PID1.update1(want_yaw, current_yaw)  # PID1专用于A区浇水或避障的PID
                self.cmd_vel_msg.angular.z = pid_output
                self.cmd_vel_msg.linear.x = A_avoid_obstacle_action6_left_vel
                self.cmd_vel_pub.publish(self.cmd_vel_msg)
                if rospy.get_time() - start_time1 > (A_avoid_obstacle_action6_left_time * self.avoidance_c_time):  # 与放大后的时间比较
                    break
                else:
                    continue
        elif self.avoidance_direction < 0:  # 右侧障碍
            while True:
                current_yaw, scope_law = self.read_imu_data()
                pid_output = self.PID1.update1(want_yaw, current_yaw)  # PID1专用于A区浇水或避障的PID
                self.cmd_vel_msg.angular.z = pid_output
                self.cmd_vel_msg.linear.x = A_avoid_obstacle_action6_right_vel
                self.cmd_vel_pub.publish(self.cmd_vel_msg)
                if rospy.get_time() - start_time1 > (A_avoid_obstacle_action6_right_time * self.avoidance_c_time):  # 与放大后的时间比较
                    break
                else:
                    continue

    def A_avoid_action_7(self):
        global want_yaw, current_yaw, scope_law
        want_yaw = 0
        print("避障ing......Action7")  # 任务进程
        if self.avoidance_direction > 0:  # 左侧障碍，原地右转
            while True:
                current_yaw, scope_law = self.read_imu_data()
                if current_yaw < want_yaw:
                    self.cmd_vel_msg.angular.z = A_avoid_obstacle_action3_left_law
                    self.cmd_vel_msg.linear.x = A_avoid_obstacle_action3_left_vel
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                elif current_yaw > want_yaw:
                    self.cmd_vel_msg.angular.z = -A_avoid_obstacle_action3_left_law
                    self.cmd_vel_msg.linear.x = A_avoid_obstacle_action3_left_vel
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                else:
                    self.cmd_vel_msg.angular.z = 0.0
                    self.cmd_vel_msg.linear.x = 0.0
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                if abs(current_yaw - want_yaw) < 2.5:
                    self.cmd_vel_msg.angular.z = 0.0
                    self.cmd_vel_msg.linear.x = 0.0
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                    break
                else:
                    continue
        elif self.avoidance_direction < 0:  # 右侧障碍
            while True:
                current_yaw, scope_law = self.read_imu_data()
                if current_yaw < want_yaw:
                    self.cmd_vel_msg.angular.z = A_avoid_obstacle_action3_right_law
                    self.cmd_vel_msg.linear.x = A_avoid_obstacle_action3_right_vel
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                elif current_yaw > want_yaw:
                    self.cmd_vel_msg.angular.z = -A_avoid_obstacle_action3_right_law
                    self.cmd_vel_msg.linear.x = A_avoid_obstacle_action3_right_vel
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                else:
                    self.cmd_vel_msg.angular.z = 0.0
                    self.cmd_vel_msg.linear.x = 0.0
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                if abs(current_yaw - want_yaw) < 2.5:
                    self.cmd_vel_msg.angular.z = 0.0
                    self.cmd_vel_msg.linear.x = 0.0
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                    break
                else:
                    continue
    def A_execute_avoidance(self):#执行避障动作：写死，转向，越过，转回，摆正，可用class MOVE_ARRIVE:time_delay(self,s):函数强迫运行完动作组，再回归正确浇水前进
         """执行避障动作"""
         print("..................Execute_Avoidance..............")
         self.executing_avoidance = True#正在执行标志为挂起
         #执行避障动作
         self.A_avoid_action_1()
         print("Action1结束")
         self.A_avoid_action_2()
         print("Action2结束")
         self.A_avoid_action_3()
         print("Action3结束")

         #"Action4（直行部分），需要兼顾检测花盆
         t=self.A_avoid_action_4()#标志位，标志是否在避障过程检测到花盆并执行了浇水：检测花盆并执行则为Ture

         print("Action4结束")
         self.A_avoid_action_5()
         print("Action5结束")
         self.A_avoid_action_6()
         print("Action6结束")
         self.A_avoid_action_7()
         print("Action7结束")

         # 避障动作持续时间检查
         #真的需要保证时检查吗？？？
         #if rospy.get_time() - self.avoidance_start_time > self.avoidance_duration:
         #   self.obstacle_detected = False
         #   return
         self.obstacle_detected = False#执行动作组完必，标志位还原为false
         self.executing_avoidance = False  # 正在执行标志为落下
         print("self.obstacle_detected = False")
         print("self.executing_avoidance = False")
         print("..................Finished_Avoidance..................")
         #避障动作组函数不执行完出不来

         return t #供A_dotime_0_015等函数使用


    def B_huapen_dis_check(self):#A区避障过程中特殊的花盆检测函数
        #只要本函数被运行，则self.avoidance_direction必然是1 or -1
        if self.avoidance_direction == 1:#左侧有障碍物，检测右侧花盆距离
            if self.min_dis[1]<B_huapen_check_thresh and self.min_dis[1] != 0:#A区避障时检测阈值，小于正常花盆检测阈值，一般等于"正常阈值 - 横移阈值"
                #返回信号“检测到花盆！！！”
                return True
            else:
                return False
        elif self.avoidance_direction == -1:#右侧有障碍物，左绕避障
            #检测左侧花盆距离
            if self.min_dis[0]<B_huapen_check_thresh and self.min_dis[0] != 0:
                #返回信号“检测到花盆！！！”
                return True
            else:
                return False
        return None
    def B_arm_execute(self):#A区机械臂执行
        print("...............arm_execute..............")
        print("Before：self.current_arm_index = ", self.current_arm_index)
        print("检查是否正在避障：")
        if self.avoidance_c_time ==0 :
            print("未进行避障")
            print(COLOR_YELLOW + ">>>>>>>>>>>Arm active first." + STYLE_RESET)
            self.cmd_arm_msg.data = 9#标准情况下B赛道的向左摆动机械臂动作组
            self.cmd_arm_pub.publish(self.cmd_arm_msg)
            self.time_delay(1.0)
            print(COLOR_YELLOW + ">>>>>>>>>>>Pump work. Times=" + str(self.area_B_dry[self.current_arm_index][0]) + STYLE_RESET)
            playsound(A[self.area_B_dry[self.current_arm_index][0] - 1])
            self.control_irrigation(self.pump_on_time, self.pump_off_time, self.area_B_dry[self.current_arm_index][0])
            self.time_delay(1.0)
            self.cmd_arm_msg.data = 2#机械臂复位
            self.cmd_arm_pub.publish(self.cmd_arm_msg)
            self.time_delay(5.0)
            print(COLOR_YELLOW + ">>>>>>>>>>>Arm active second." + STYLE_RESET)
            self.cmd_arm_msg.data = 10#标准情况下B赛道的向右摆动机械臂动作组
            self.cmd_arm_pub.publish(self.cmd_arm_msg)
            self.time_delay(2.0)
            print(COLOR_YELLOW + ">>>>>>>>>>>Pump work. Times=" + str(self.area_B_dry[self.current_arm_index][1]) + STYLE_RESET)
            playsound(A[self.area_B_dry[self.current_arm_index][1] - 1])
            self.control_irrigation(self.pump_on_time, self.pump_off_time, self.area_B_dry[self.current_arm_index][1])
            self.time_delay(1.0)
            print(COLOR_YELLOW + ">>>>>>>>>>>Arm reset." + STYLE_RESET)
            self.cmd_arm_msg.data = 2#机械臂复位
            self.cmd_arm_pub.publish(self.cmd_arm_msg)
            self.time_delay(2.0)

        elif self.avoidance_c_time ==  1:#标准避障情况下，需额外检验左避障还是右避障
            print("正在避障")
            print(COLOR_YELLOW + ">>>>>>>>>>>Arm active first." + STYLE_RESET)
            #先向左摆动
            if self.avoidance_direction == 1 :#左侧有障碍物，向右避障时
                print("子状态:左侧有障碍物，向右避障")
                print("机械臂:B_左摆动伸长")
                self.cmd_arm_msg.data = 11 #B_左摆动伸长
            elif self.avoidance_direction == -1 :#右侧有障碍物，向左避障时
                print("子状态:右侧有障碍物，向左避障")
                print("机械臂:B_左摆动缩短")
                self.cmd_arm_msg.data = 12 #B_左摆动缩短
            self.cmd_arm_pub.publish(self.cmd_arm_msg)
            self.time_delay(1.0)
            print(COLOR_YELLOW + ">>>>>>>>>>>Pump work. Times=" + str(self.area_B_dry[self.current_arm_index][0]) + STYLE_RESET)
            playsound(A[self.area_B_dry[self.current_arm_index][0] - 1])
            self.control_irrigation(self.pump_on_time, self.pump_off_time, self.area_B_dry[self.current_arm_index][0])
            self.time_delay(1.0)
            self.cmd_arm_msg.data = 2#机械臂复位
            self.cmd_arm_pub.publish(self.cmd_arm_msg)
            self.time_delay(5.0)
            print(COLOR_YELLOW + ">>>>>>>>>>>Arm active second." + STYLE_RESET)
            if self.avoidance_direction == 1:  # 左侧有障碍物，向右避障时
                print("子状态:左侧有障碍物，向右避障")
                print("机械臂:B_右摆动缩短")
                self.cmd_arm_msg.data = 14 #B_右摆动缩短_动作组
            elif self.avoidance_direction == -1:  # 右侧有障碍物，向左避障时
                print("子状态:右侧有障碍物，向左避障")
                print("机械臂:B_右摆动伸长")
                self.cmd_arm_msg.data = 13 #B_右摆动伸长_浇水动作组
            self.cmd_arm_pub.publish(self.cmd_arm_msg)
            self.time_delay(2.0)
            print(COLOR_YELLOW + ">>>>>>>>>>>Pump work. Times=" + str(self.area_B_dry[self.current_arm_index][1]) + STYLE_RESET)
            playsound(A[self.area_B_dry[self.current_arm_index][1] - 1])
            self.control_irrigation(self.pump_on_time, self.pump_off_time, self.area_B_dry[self.current_arm_index][1])
            self.time_delay(1.0)
            print(COLOR_YELLOW + ">>>>>>>>>>>Arm reset." + STYLE_RESET)
            self.cmd_arm_msg.data = 2 #机械臂复位
            self.cmd_arm_pub.publish(self.cmd_arm_msg)
            self.time_delay(2.0)

        self.current_arm_index += 1#直行之后，标志位+1
        print("After：self.current_arm_index = ",self.current_arm_index)
        print("..................arm_execute Finished!!!!......................")
    def B_avoid_action_1(self):  # 原地右转
        global want_yaw, current_yaw, scope_law
        want_yaw = 90.0 * self.avoidance_direction  # 左侧有障碍物（self.avoidance_direction=1），右画弧
        print("避障ing......Action1")  # 任务进程
        if self.avoidance_direction > 0:  # 左侧障碍，原地右转，目标90度
            while True:  # 永循环，直到误差abs(current_yaw-want_yaw)<2.5
                current_yaw, scope_law = self.read_imu_data()
                if current_yaw < 0.0 and current_yaw > -180.0:  # 180度朝向偏左，顺时针
                    self.cmd_vel_msg.angular.z = -B_avoid_obstacle_action1_right_law
                    self.cmd_vel_msg.linear.x = B_avoid_obstacle_action1_right_vel
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                elif current_yaw < 180.0 and current_yaw >= want_yaw + 5.0:  # 180度朝向偏右，或转向过程中偏到目标角度范围正方向以外，顺时针
                    self.cmd_vel_msg.angular.z = -B_avoid_obstacle_action1_right_law
                    self.cmd_vel_msg.linear.x = B_avoid_obstacle_action1_right_vel
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                elif current_yaw <= want_yaw - 5.0:  # 转向过程中偏到目标角度范围负方向以外，逆时针转
                    self.cmd_vel_msg.angular.z = B_avoid_obstacle_action1_right_law
                    self.cmd_vel_msg.linear.x = B_avoid_obstacle_action1_right_vel
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                else:  # 在want_yaw上下5度以内，开始原地微调，微调是顺时针还是逆时针？？？
                    # print("我在微调...")#验证是否会困于微调函数
                    self.cmd_vel_msg.angular.z = B_avoid_obstacle_action1_right_law  # 此时原地微调
                    self.cmd_vel_msg.linear.x = 0.0  # 应该继续直行还是直接停住？？？
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                if abs(current_yaw - want_yaw) < 2.5:
                    self.cmd_vel_msg.angular.z = 0.0
                    self.cmd_vel_msg.linear.x = 0.0
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                    break
                else:
                    continue
        elif self.avoidance_direction < 0:  # 右侧障碍，原地左转到90度
            while True:  # 永循环，直到误差abs(current_yaw-want_yaw)<2.5
                current_yaw, scope_law = self.read_imu_data()
                if current_yaw > 0.0 and current_yaw < 180.0:# 180度朝向偏右，逆时针
                    self.cmd_vel_msg.angular.z = B_avoid_obstacle_action1_left_law
                    self.cmd_vel_msg.linear.x = B_avoid_obstacle_action1_left_vel
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                elif current_yaw > -180.0 and current_yaw <= want_yaw - 5.0:# 180度朝向偏右，或转向过程中偏到目标角度范围负方向以外，逆时针
                    self.cmd_vel_msg.angular.z = B_avoid_obstacle_action1_left_law
                    self.cmd_vel_msg.linear.x = B_avoid_obstacle_action1_left_vel
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                elif current_yaw < 0.0 and current_yaw >= want_yaw + 5.0:#转向过程中偏到目标角度范围正方向以外，顺时针转
                    self.cmd_vel_msg.angular.z = -B_avoid_obstacle_action1_left_law
                    self.cmd_vel_msg.linear.x = B_avoid_obstacle_action1_left_vel
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                else:
                    self.cmd_vel_msg.angular.z = B_avoid_obstacle_action1_left_law  # 此时原地微调
                    self.cmd_vel_msg.linear.x = 0.0
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                if abs(current_yaw - want_yaw) < 5.0:
                    self.cmd_vel_msg.angular.z = 0.0
                    self.cmd_vel_msg.linear.x = 0.0
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                    break
                else:
                    continue
    def B_avoid_action_2(self):  # 第一次转弯后直行
        print("避障ing......Action2")  # 任务进程
        global want_yaw, current_yaw, start_time1, scope_law
        want_yaw = 90.0 * self.avoidance_direction
        start_time1 = rospy.get_time()
        if self.avoidance_direction > 0:  # 左侧障碍
            while True:
                current_yaw, scope_law = self.read_imu_data()
                pid_output = self.PID2.update2(want_yaw, current_yaw)  # PID1专用于A区浇水或避障的PID
                self.cmd_vel_msg.angular.z = pid_output
                self.cmd_vel_msg.linear.x = B_avoid_obstacle_action2_left_vel
                self.cmd_vel_pub.publish(self.cmd_vel_msg)
                if rospy.get_time() - start_time1 > (
                        B_avoid_obstacle_action2_left_time * self.avoidance_c_time):  # 与放大后的时间比较
                    break
                else:
                    continue
        elif self.avoidance_direction < 0:  # 右侧障碍
            while True:
                current_yaw, scope_law = self.read_imu_data()
                pid_output = self.PID2.update2(want_yaw, current_yaw)  # PID1专用于A区浇水或避障的PID
                self.cmd_vel_msg.angular.z = pid_output
                self.cmd_vel_msg.linear.x = B_avoid_obstacle_action2_right_vel
                self.cmd_vel_pub.publish(self.cmd_vel_msg)
                if rospy.get_time() - start_time1 > (
                        B_avoid_obstacle_action2_right_time * self.avoidance_c_time):  # 与放大后的时间比较
                    break
                else:
                    continue
    def B_avoid_action_3(self):  # 转回0度
        global want_yaw, current_yaw, scope_law

        print("避障ing......Action3")  # 任务进程
        if self.avoidance_direction > 0:  # 左侧障碍
            want_yaw = 180
            while True:
                current_yaw, scope_law = self.read_imu_data()
                if current_yaw>0 and current_yaw < want_yaw:#转向过程中偏到目标角度范围负方向以外，逆时针
                    self.cmd_vel_msg.angular.z = B_avoid_obstacle_action3_left_law
                    self.cmd_vel_msg.linear.x = B_avoid_obstacle_action3_left_vel
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                elif current_yaw <0 : # 转向超越180度朝向偏右突变为180-，并且限制于-0到-180度或转向过程中偏到目标角度范围正方向以外，顺时针
                    self.cmd_vel_msg.angular.z = -B_avoid_obstacle_action3_left_law
                    self.cmd_vel_msg.linear.x = B_avoid_obstacle_action3_left_vel
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                else:
                    self.cmd_vel_msg.angular.z = 0.0
                    self.cmd_vel_msg.linear.x = 0.0
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                if abs(current_yaw - want_yaw) < 2.5:
                    self.cmd_vel_msg.angular.z = 0.0
                    self.cmd_vel_msg.linear.x = 0.0
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                    break
                else:
                    continue
        elif self.avoidance_direction < 0:  # 右侧障碍
            want_yaw = -180
            while True:
                current_yaw, scope_law = self.read_imu_data()
                if current_yaw<0.0 and current_yaw>want_yaw:#转向过程中偏到目标角度范围正方向以外，顺时针
                    self.cmd_vel_msg.angular.z = -B_avoid_obstacle_action3_right_law
                    self.cmd_vel_msg.linear.x = B_avoid_obstacle_action3_right_vel
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                elif current_yaw > 0.0 :# 转向超越-180度朝向偏右突变为180+，并且限制于170到180度或转向过程中偏到目标角度范围正方向以外，逆时针
                    self.cmd_vel_msg.angular.z = B_avoid_obstacle_action3_right_law
                    self.cmd_vel_msg.linear.x = B_avoid_obstacle_action3_right_vel
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                else:
                    self.cmd_vel_msg.angular.z = 0.0
                    self.cmd_vel_msg.linear.x = 0.0
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                if abs(current_yaw-want_yaw)<5.0:#不再精调，避免-179与179的角度突变
                    self.cmd_vel_msg.angular.z = 0.0
                    self.cmd_vel_msg.linear.x = 0.0
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                    break
                else:
                    continue
    def B_avoid_action_4(self):  # 第二次转弯后直行
        print("避障ing......Action4")  # 任务进程
        global want_yaw, current_yaw, start_time1, scope_law
        want_yaw = -180
        t = 0.0  # 存储：避障过程浇水占用了多少时间

        # 设置标志位：避障过程检测到花盆,停车并执行浇水
        # 供execute_avoidance()使用
        tag = False

        # action4中限制最多执行一次浇水
        once_tag = 0  # 标志action4中有没有执行过浇水

        start_time1 = rospy.get_time()
        if self.avoidance_direction > 0:  # 左侧障碍
            while True:
                current_yaw, scope_law = self.read_imu_data()
                pid_output = self.PID2.update2(want_yaw, current_yaw)  # PID1专用于A区浇水或避障的PID
                self.cmd_vel_msg.angular.z = pid_output
                self.cmd_vel_msg.linear.x = B_avoid_obstacle_action4_left_vel
                self.cmd_vel_pub.publish(self.cmd_vel_msg)

                # 如果避障过程中浇水了，必然占用直行时间，耽误直行距离
                # 解决方法：记录执行避障中处理花盆浇水的函数运行所需要的时间（运行了则在函数内部用两次rospy.get_time()标记时间起始点，没运行则时间为0.0s）

                if self.B_huapen_dis_check() and once_tag == 0:  # 如果避障过程检测到花盆,并且action4中尚未浇水#：停车并执行浇水
                    # 停车
                    print("避障中检测到花盆！！！！")
                    print("停车")
                    self.dotime_stop()
                    # 执行避障中处理花盆浇水的函数
                    # 获取起始时间
                    t1 = rospy.get_time()
                    # 执行避障中处理花盆浇水的函数
                    print("执行机械臂动作组ing......")
                    print("Start：", t1)
                    self.B_arm_execute()
                    # 获取结束时间
                    t2 = rospy.get_time()
                    print("Finished", t2)
                    # 赋值，占用了多少时间
                    t = t2 - t1
                    print("Total Time", t)
                    # 设置标志位：避障过程检测到花盆,停车并执行浇水
                    # 供execute_avoidance()使用
                    tag = True
                    once_tag = 1  # 标志位，表示action4中仅限一次的浇水机会用完了
                    print("once_tag = 1,表示action4中仅限一次的浇水机会用完了")
                    # !!!最后的判断相应变成if rospy.get_time() -Spend_Time - start_time1 > A_avoid_obstacle_action4_time
                if rospy.get_time() - t - start_time1 > A_avoid_obstacle_action4_left_time:  # 此段路程不用系数放大A_avoid_obstacle_action4_time
                    print("rospy.get_time() -t- start_time1 =", rospy.get_time() - t - start_time1)
                    print("B_avoid_obstacle_action4_time:", B_avoid_obstacle_action4_left_time)
                    print("rospy.get_time() -t- start_time1 > B_avoid_obstacle_action4_time")
                    print("跳出while循环")
                    break
                else:
                    continue
        elif self.avoidance_direction < 0:  # 右侧障碍
            while True:
                current_yaw, scope_law = self.read_imu_data()
                pid_output = self.PID2.update2(want_yaw, current_yaw)  # PID1专用于A区浇水或避障的PID
                self.cmd_vel_msg.angular.z = pid_output
                self.cmd_vel_msg.linear.x = B_avoid_obstacle_action4_right_vel
                self.cmd_vel_pub.publish(self.cmd_vel_msg)

                # 如果避障过程中浇水了，必然占用直行时间，耽误直行距离
                # 解决方法：记录执行避障中处理花盆浇水的函数运行所需要的时间（运行了则在函数内部用两次rospy.get_time()标记时间起始点，没运行则时间为0.0s）

                if self.B_huapen_dis_check() and once_tag == 0:  # 如果避障过程检测到花盆,并且action4中尚未浇水#：停车并执行浇水
                    # 停车
                    print("避障中检测到花盆！！！！")
                    print("停车")
                    self.dotime_stop()
                    # 执行避障中处理花盆浇水的函数
                    # 获取起始时间
                    t1 = rospy.get_time()
                    # 执行避障中处理花盆浇水的函数
                    print("执行机械臂动作组ing......")
                    print("Start：", t1)
                    self.B_arm_execute()
                    # 获取结束时间
                    t2 = rospy.get_time()
                    print("Finished", t2)
                    # 赋值，占用了多少时间
                    t = t2 - t1
                    print("Total Time", t)
                    # 设置标志位：避障过程检测到花盆,停车并执行浇水
                    # 供execute_avoidance()使用
                    tag = True
                    once_tag = 1  # 标志位，表示action4中仅限一次的浇水机会用完了
                    print("once_tag = 1,表示action4中仅限一次的浇水机会用完了")
                    # !!!最后的判断相应变成if rospy.get_time() -Spend_Time - start_time1 > A_avoid_obstacle_action4_time
                if rospy.get_time() - t - start_time1 > B_avoid_obstacle_action4_right_time:  # 此段路程不用系数放大B_avoid_obstacle_action4_time
                    print("rospy.get_time() -t- start_time1 =", rospy.get_time() - t - start_time1)
                    print("B_avoid_obstacle_action4_time:", B_avoid_obstacle_action4_right_time)
                    print("rospy.get_time() -t- start_time1 > B_avoid_obstacle_action4_time")
                    print("跳出while循环")
                    break
                else:
                    continue

        print("是否避章时停车浇水.....", tag)
        return tag  # 供execute_avoidance()使用
    def B_avoid_action_4_no_detect(self):#供最后一个花盆到转向点路段的避障，此段路径无需避障时检测花盆
        print("避障ing......Action4_no_detect")  # 任务进程
        global want_yaw, current_yaw, start_time1, scope_law
        want_yaw = -180
        t=0.0 #存储：避障过程浇水占用了多少时间
        # 设置标志位：避障过程检测到花盆,停车并执行浇水
        # 供execute_avoidance()使用
        tag = False

        # action4中限制最多执行一次浇水
        once_tag = 0#标志action4中有没有执行过浇水

        start_time1 = rospy.get_time()
        if self.avoidance_direction > 0:  # 左侧障碍
            while True:
                current_yaw, scope_law = self.read_imu_data()
                pid_output = self.PID2.update2(want_yaw, current_yaw)  # PID1专用于A区浇水或避障的PID
                self.cmd_vel_msg.angular.z = pid_output
                self.cmd_vel_msg.linear.x = B_avoid_obstacle_action4_left_vel
                self.cmd_vel_pub.publish(self.cmd_vel_msg)
                if rospy.get_time() - t - start_time1 > A_avoid_obstacle_action4_left_time:  # 此段路程不用系数放大A_avoid_obstacle_action4_time
                    print("rospy.get_time() -t- start_time1 =", rospy.get_time() - t - start_time1)
                    print("B_avoid_obstacle_action4_time:", B_avoid_obstacle_action4_left_time)
                    print("rospy.get_time() -t- start_time1 > B_avoid_obstacle_action4_time")
                    print("跳出while循环")
                    break
                else:
                    continue
        elif self.avoidance_direction < 0:  # 右侧障碍
            while True:
                current_yaw, scope_law = self.read_imu_data()
                pid_output = self.PID2.update2(want_yaw, current_yaw)  # PID1专用于A区浇水或避障的PID
                self.cmd_vel_msg.angular.z = pid_output
                self.cmd_vel_msg.linear.x = B_avoid_obstacle_action4_right_vel
                self.cmd_vel_pub.publish(self.cmd_vel_msg)
                if rospy.get_time() - t - start_time1 > B_avoid_obstacle_action4_right_time:  # 此段路程不用系数放大B_avoid_obstacle_action4_time
                    print("rospy.get_time() -t- start_time1 =", rospy.get_time() - t - start_time1)
                    print("B_avoid_obstacle_action4_time:", B_avoid_obstacle_action4_right_time)
                    print("rospy.get_time() -t- start_time1 > B_avoid_obstacle_action4_time")
                    print("跳出while循环")
                    break
                else:
                    continue
        return tag  # 供execute_avoidance()使用
    def B_avoid_action_5(self):  # 第三次转弯
        global want_yaw, current_yaw, scope_law
        want_yaw = -90.0 * self.avoidance_direction  # 左侧有障碍物（self.avoidance_direction=1），右画弧
        print("避障ing......Action5")  # 任务进程
        if self.avoidance_direction > 0:  # 左侧障碍
            while True:  # 永循环，直到误差abs(current_yaw-want_yaw)<2.5
                current_yaw, scope_law = self.read_imu_data()
                if current_yaw > 0.0 and current_yaw < 180.0:  # 180度朝向偏右，逆时针
                    self.cmd_vel_msg.angular.z = B_avoid_obstacle_action5_left_law
                    self.cmd_vel_msg.linear.x = B_avoid_obstacle_action5_left_vel
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                elif current_yaw > -180.0 and current_yaw <= want_yaw - 5.0:  # 180度朝向偏右，或转向过程中偏到目标角度范围负方向以外，逆时针
                    self.cmd_vel_msg.angular.z = B_avoid_obstacle_action5_left_law
                    self.cmd_vel_msg.linear.x = B_avoid_obstacle_action5_left_vel
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                elif current_yaw < 0.0 and current_yaw >= want_yaw + 5.0:  # 转向过程中偏到目标角度范围正方向以外，顺时针转
                    self.cmd_vel_msg.angular.z = -B_avoid_obstacle_action5_left_law
                    self.cmd_vel_msg.linear.x = B_avoid_obstacle_action5_left_vel
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                else:
                    self.cmd_vel_msg.angular.z = -B_avoid_obstacle_action5_left_law  # 此时原地微调
                    self.cmd_vel_msg.linear.x = 0.0
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                if abs(current_yaw - want_yaw) < 5.0:
                    self.cmd_vel_msg.angular.z = 0.0
                    self.cmd_vel_msg.linear.x = 0.0
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                    break
                else:
                    continue
        elif self.avoidance_direction < 0:  # 右侧障碍
            while True:  # 永循环，直到误差abs(current_yaw-want_yaw)<2.5
                current_yaw, scope_law = self.read_imu_data()
                if current_yaw < 0.0 and current_yaw > -180.0:  # 180度朝向偏左，顺时针
                    self.cmd_vel_msg.angular.z = -B_avoid_obstacle_action5_right_law
                    self.cmd_vel_msg.linear.x = B_avoid_obstacle_action5_right_vel
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                elif current_yaw < 180.0 and current_yaw >= want_yaw + 5.0:  # 180度朝向偏右，或转向过程中偏到目标角度范围正方向以外，顺时针
                    self.cmd_vel_msg.angular.z = -B_avoid_obstacle_action5_right_law
                    self.cmd_vel_msg.linear.x = B_avoid_obstacle_action5_right_vel
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                elif current_yaw <= want_yaw - 5.0:  # 转向过程中偏到目标角度范围负方向以外，逆时针转
                    self.cmd_vel_msg.angular.z = B_avoid_obstacle_action5_right_law
                    self.cmd_vel_msg.linear.x = B_avoid_obstacle_action5_right_vel
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                else:  # 在want_yaw上下5度以内，开始原地微调，微调是顺时针还是逆时针？？？
                    # print("我在微调...")#验证是否会困于微调函数
                    self.cmd_vel_msg.angular.z = B_avoid_obstacle_action5_right_law  # 此时原地微调
                    self.cmd_vel_msg.linear.x = 0.0  # 应该继续直行还是直接停住？？？
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                if abs(current_yaw - want_yaw) < 5:
                    self.cmd_vel_msg.angular.z = 0.0
                    self.cmd_vel_msg.linear.x = 0.0
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                    break
                else:
                    continue
    def B_avoid_action_6(self):  # 第三次转弯后的直行
        print("避障ing......Action6")  # 任务进程
        global want_yaw, current_yaw, start_time1, scope_law
        want_yaw = -90.0 * self.avoidance_direction
        start_time1 = rospy.get_time()
        print(self.avoidance_direction)
        if self.avoidance_direction > 0:  # 左侧障碍，原地右转
            while True:
                current_yaw, scope_law = self.read_imu_data()
                pid_output = self.PID2.update2(want_yaw, current_yaw)  # PID1专用于A区浇水或避障的PID
                self.cmd_vel_msg.angular.z = pid_output
                self.cmd_vel_msg.linear.x = B_avoid_obstacle_action6_left_vel
                self.cmd_vel_pub.publish(self.cmd_vel_msg)
                if rospy.get_time() - start_time1 > (
                        B_avoid_obstacle_action6_left_time * self.avoidance_c_time):
                    print("B_avoid_obstacle_action6_left_time") 
                    print("self.avoidance_c_time") # 与放大后的时间比较
                    break
                else:
                    continue
        elif self.avoidance_direction < 0:  # 右侧障碍
            while True:
                current_yaw, scope_law = self.read_imu_data()
                pid_output = self.PID2.update2(want_yaw, current_yaw)  # PID1专用于A区浇水或避障的PID
                self.cmd_vel_msg.angular.z = pid_output
                self.cmd_vel_msg.linear.x = B_avoid_obstacle_action6_right_vel
                self.cmd_vel_pub.publish(self.cmd_vel_msg)
                if rospy.get_time() - start_time1 > (
                        B_avoid_obstacle_action6_right_time * self.avoidance_c_time):
                    print("B_avoid_obstacle_action6_right_time") 
                    print("self.avoidance_c_time") # 与放大后的时间比较  # 与放大后的时间比较
                    break
                else:
                    continue
    def B_avoid_action_7(self):
        global want_yaw, current_yaw, scope_law
        print("避障ing......Action7")  # 任务进程
        if self.avoidance_direction > 0:  # 左侧障碍
            want_yaw = -180
            while True:
                current_yaw, scope_law = self.read_imu_data()
                if current_yaw < 0.0 and current_yaw > want_yaw+5.0:  # 转向过程中偏到目标角度范围正方向以外，顺时针
                    self.cmd_vel_msg.angular.z = -B_avoid_obstacle_action7_right_law
                    self.cmd_vel_msg.linear.x = B_avoid_obstacle_action7_right_vel
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                elif current_yaw > 0.0:  # 转向超越-180度朝向偏右突变为180+，并且限制于170到180度或转向过程中偏到目标角度范围正方向以外，逆时针
                    self.cmd_vel_msg.angular.z = B_avoid_obstacle_action7_right_law
                    self.cmd_vel_msg.linear.x = B_avoid_obstacle_action7_right_vel
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                else:
                    self.cmd_vel_msg.angular.z = 0.0
                    self.cmd_vel_msg.linear.x = 0.0
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                if abs(current_yaw - want_yaw) < 5.0:  # 不再精调，避免-179与179的角度突变
                    self.cmd_vel_msg.angular.z = 0.0
                    self.cmd_vel_msg.linear.x = 0.0
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                    break
                else:
                    continue
        elif self.avoidance_direction < 0:  # 右侧障碍
            want_yaw = 180
            while True:
                current_yaw, scope_law = self.read_imu_data()
                if current_yaw > 0 and current_yaw < want_yaw - 5.0:  # 转向过程中偏到目标角度范围负方向以外，逆时针
                    self.cmd_vel_msg.angular.z = B_avoid_obstacle_action7_left_law
                    self.cmd_vel_msg.linear.x = B_avoid_obstacle_action7_left_vel
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                elif current_yaw < 0:  # 转向超越180度朝向偏右突变为180-，并且限制于-0到-180度或转向过程中偏到目标角度范围正方向以外，顺时针
                    self.cmd_vel_msg.angular.z = -B_avoid_obstacle_action7_left_law
                    self.cmd_vel_msg.linear.x = B_avoid_obstacle_action7_left_vel
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                else:
                    self.cmd_vel_msg.angular.z = 0.0
                    self.cmd_vel_msg.linear.x = 0.0
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                if abs(current_yaw - want_yaw) < 5.0:
                    self.cmd_vel_msg.angular.z = 0.0
                    self.cmd_vel_msg.linear.x = 0.0
                    self.cmd_vel_pub.publish(self.cmd_vel_msg)
                    break
                else:
                    continue
    def B_execute_avoidance(self):  # 执行避障动作：写死，转向，越过，转回，摆正，可用class MOVE_ARRIVE:time_delay(self,s):函数强迫运行完动作组，再回归正确浇水前进
        """执行避障动作"""
        print("..................Execute_Avoidance..............")
        self.executing_avoidance = True  # 正在执行标志为挂起
        # 执行避障动作
        self.B_avoid_action_1()
        print("Action1结束")
        self.B_avoid_action_2()
        print("Action2结束")
        self.B_avoid_action_3()
        print("Action3结束")

        # "Action4（直行部分），需要兼顾检测花盆
        t = self.B_avoid_action_4()  # 标志位，标志是否在避障过程检测到花盆并执行了浇水：检测花盆并执行则为Ture

        print("Action4结束")
        self.B_avoid_action_5()
        print("Action5结束")
        self.B_avoid_action_6()
        print("Action6结束")
        self.B_avoid_action_7()
        print("Action7结束")

        # 避障动作持续时间检查
        # 真的需要保证时检查吗？？？
        # if rospy.get_time() - self.avoidance_start_time > self.avoidance_duration:
        #   self.obstacle_detected = False
        #   return
        self.obstacle_detected = False  # 执行动作组完必，标志位还原为false
        self.executing_avoidance = False  # 正在执行标志为落下
        print("self.obstacle_detected = False")
        print("self.executing_avoidance = False")
        print("..................Finished_Avoidance..................")
        # 避障动作组函数不执行完出不来

        return t  # 供A_dotime_0_015等函数使用
    def B_execute_avoidance_no_detect(self):  # 执行避障动作：写死，转向，越过，转回，摆正，可用class MOVE_ARRIVE:time_delay(self,s):函数强迫运行完动作组，再回归正确浇水前进
        """执行避障动作"""
        print("..................Execute_Avoidance..............")
        self.executing_avoidance = True  # 正在执行标志为挂起
        # 执行避障动作
        self.B_avoid_action_1()
        print("Action1结束")
        self.B_avoid_action_2()
        print("Action2结束")
        self.B_avoid_action_3()
        print("Action3结束")

        # "Action4（直行部分），
        self.B_avoid_action_4_no_detect()  #供走出花盆时候的避障，此段路径无需避障时检测花盆

        print("Action4结束")
        self.B_avoid_action_5()
        print("Action5结束")
        self.B_avoid_action_6()
        print("Action6结束")
        self.B_avoid_action_7()
        print("Action7结束")

        # 避障动作持续时间检查
        # 真的需要保证时检查吗？？？
        # if rospy.get_time() - self.avoidance_start_time > self.avoidance_duration:
        #   self.obstacle_detected = False
        #   return
        self.obstacle_detected = False  # 执行动作组完必，标志位还原为false
        self.executing_avoidance = False  # 正在执行标志为落下
        print("self.obstacle_detected = False")
        print("self.executing_avoidance = False")
        print("..................Finished_Avoidance..................")
        # 避障动作组函数不执行完出不来





    def A_dotime_0_015(self):#小车在A区直线行驶
        print("...................开始直行.................")
        global want_yaw,current_yaw,scope_law
        want_yaw = 0.0
        t = False  # 初始化
        while True:
            current_yaw,scope_law=self.read_imu_data()
            #优先避障，如果检测到障碍，会“停止”花盆检测，continue跳到了下一次while循环
            if self.obstacle_detected:
                print("!!!!检测到障碍物！！！！")
                t=self.A_execute_avoidance()#t为标志位，标志避障过程有没有浇水
                print("Irrigated？",t)
                continue#是不是有没有都一样？？？？？？

            #如果避障过程浇水，执行完避障动作后，以下程序不再执行，A_dotime_0_015函数到此为止，直接return，task_A()中i的循环进程需跳到下一次循环
            if t :#避障过程中浇水了，执行完避障动作后，以下程序不再执行，A_dotime_0_015函数到此为止，直接return，task_A()中i的循环进程需跳到下一次循环
                print("避障过程已经浇水")
                print("...................结束本次直行.................")
                return
            # 如果避障过程没有浇水，下面程序正常运行，task_A()中i的循环进程无需变动
            elif not t:#没有执行避障 or 避障过程没检测到花盆进而浇水了
                pid_output = self.PID1.update1(want_yaw, current_yaw)
                self.cmd_vel_msg.angular.z = pid_output
                self.cmd_vel_msg.linear.x = A_SPEED
                self.cmd_vel_pub.publish(self.cmd_vel_msg)
                if self.min_dis[0] < A_huapen_thresh and self.min_dis[0] != 0:
                    print("检测到花盆")
                    print("Left : %f   Right : %f", self.min_dis[0], self.min_dis[1])
                    print("...................结束本次直行.................")
                    break#break后只是跳出了循环，结束函数，并没有设置小车停止
                else:
                    continue
    def A_dotime_0_015_huapen(self):#小车在A区直线行驶,喷完一个花盆后走出这个花盆
        print("...................开始走出花盆.................")
        global want_yaw,current_yaw,scope_law
        want_yaw = 0.0     
        while True:
            current_yaw,scope_law=self.read_imu_data()
            pid_output = self.PID1.update1(want_yaw,current_yaw)        
            self.cmd_vel_msg.angular.z = pid_output
            self.cmd_vel_msg.linear.x = A_SPEED
            self.cmd_vel_pub.publish(self.cmd_vel_msg)      
            if self.min_dis[0]>A_huapen_thresh:
                break
            else:
                continue
        print("...................已经走出花盆.................")
    def A_dotime_0_015end(self):#小车在A区的最后一个花盆之后，十字之前的直线行驶
        print("...................开始前往转向点.................")
        global want_yaw,current_yaw,start_time1,scope_law
        want_yaw = 0.0
        t=0#避障消耗时间/s,默认为0
        t_avoid = 0.0
        start_time=rospy.get_time()
        print("start_time:",start_time)
        while True:
            current_yaw,scope_law=self.read_imu_data()
            pid_output = self.PID1.update1(want_yaw,current_yaw)
            self.cmd_vel_msg.angular.z = pid_output
            self.cmd_vel_msg.linear.x = A_avoid_obstacle_action4_left_vel#保证此速度与避障的直行速度相等，是下面执行时间的不等式具有有效性的前提
            self.cmd_vel_pub.publish(self.cmd_vel_msg)
            if  rospy.get_time()-start_time-t>diyicizhuanwanqiandezhixing_time:
                print("Time_Costed：", rospy.get_time()-start_time-t)
                print("大于预设直行(除去浇水时间）:", diyicizhuanwanqiandezhixing_time)
                print("跳出while循环")
                break
            else:
                continue
        print("...................已到达转向点.................")
    def A_dotime_right_90(self):#小车在A区白色十字向右旋转刚好90度
        global want_yaw,current_yaw,scope_law
        want_yaw = -90.0
        print("A to B:first 90-turn")#renwujincheng
        while True:#永循环，直到误差abs(current_yaw-want_yaw)<2.5
            current_yaw,scope_law=self.read_imu_data()
            
            if current_yaw>0.0:
                self.cmd_vel_msg.angular.z = -diyicizhuanwan_law
                self.cmd_vel_msg.linear.x = 0.0
                self.cmd_vel_pub.publish(self.cmd_vel_msg)
            elif current_yaw<0.0 and current_yaw>=want_yaw+5.0:
                self.cmd_vel_msg.angular.z = -diyicizhuanwan_law
                self.cmd_vel_msg.linear.x = 0.0
                self.cmd_vel_pub.publish(self.cmd_vel_msg)
            elif current_yaw<=want_yaw-5.0:
                self.cmd_vel_msg.angular.z = diyicizhuanwan_law
                self.cmd_vel_msg.linear.x = 0.0
                self.cmd_vel_pub.publish(self.cmd_vel_msg)
            else:
                self.cmd_vel_msg.angular.z = 0.0
                self.cmd_vel_msg.linear.x = 0.0
                self.cmd_vel_pub.publish(self.cmd_vel_msg)
            if abs(current_yaw-want_yaw)<2.5:
                self.cmd_vel_msg.angular.z = 0.0
                self.cmd_vel_msg.linear.x = 0.0
                self.cmd_vel_pub.publish(self.cmd_vel_msg)
                break
            else:
                continue
    def A_B_dotime_right_90_015(self):#小车在A区白色十字到达B区第一个白色十字的直线行驶过程
        global want_yaw,current_yaw,start_time1,scope_law
        want_yaw = -90.0
        print("A to B:straight after first 90-turn")#renwujincheng
        start_time1=rospy.get_time()
        while True:
            current_yaw,scope_law=self.read_imu_data()
            
            pid_output = self.PID1.update1(want_yaw,current_yaw)        
            self.cmd_vel_msg.angular.z = pid_output
            self.cmd_vel_msg.linear.x = 0.15
            self.cmd_vel_pub.publish(self.cmd_vel_msg)
            if rospy.get_time()-start_time1>diyicizhuanwanhoudezhixing_time:#保证转弯完成，检测转弯时间是否超时
                break
            else:
                continue
    def B_dotime_right_180(self):#小车在B区第一个白色十字右转刚好90度，与初始角度成180度
        global want_yaw,current_yaw,scope_law
        want_yaw=-180.0
        print("A to B:second 90-turn")#任务进程
        while True:#永循环，直到误差abs(current_yaw-want_yaw)<2.5
            current_yaw,scope_law=self.read_imu_data()
            
            if current_yaw<0.0 and current_yaw>want_yaw+5.0:
                self.cmd_vel_msg.angular.z = -diyicizhuanwan_law
                self.cmd_vel_msg.linear.x = 0.0
                self.cmd_vel_pub.publish(self.cmd_vel_msg)
            elif current_yaw>0.0 and current_yaw<abs(want_yaw)-5.0:                  
                self.cmd_vel_msg.angular.z = diyicizhuanwan_law
                self.cmd_vel_msg.linear.x = 0.0
                self.cmd_vel_pub.publish(self.cmd_vel_msg)
            else:
                self.cmd_vel_msg.angular.z = 0.0
                self.cmd_vel_msg.linear.x = 0.0  
                self.cmd_vel_pub.publish(self.cmd_vel_msg)
            if abs(current_yaw-want_yaw)<5.0:#不再精调，避免-179与179的角度突变
                self.cmd_vel_msg.angular.z = 0.0
                self.cmd_vel_msg.linear.x = 0.0
                self.cmd_vel_pub.publish(self.cmd_vel_msg)
                break
            else:
                continue

    def B_dotime_180_020(self):#小车在B区直线行驶
        print("...................开始直行.................")
        global want_yaw,current_yaw,scope_law
        want_yaw=-180.0
        t = False  # 初始化
        self.PID2 = PID(Kp=10.0, Ki=0.0, Kd=0.0)
        while True:
            current_yaw,scope_law=self.read_imu_data()
            #优先避障，如果检测到障碍，会“停止”花盆检测，continue跳过了下面的花盆检测
            if self.obstacle_detected:
                print("!!!!检测到障碍物！！！！")
                t = self.B_execute_avoidance()  # t为标志位，标志避障过程有没有浇水
                print("Irrigated？", t)
                continue  # 是不是有没有都一样？？？？？？

            # 如果避障过程浇水，执行完避障动作后，以下程序不再执行，A_dotime_0_015函数到此为止，直接return，task_A()中i的循环进程需跳到下一次循环
            if t:  # 避障过程中浇水了，执行完避障动作后，以下程序不再执行，A_dotime_0_015函数到此为止，直接return，task_A()中i的循环进程需跳到下一次循环
                print("避障过程已经浇水")
                print("...................结束本次直行.................")
                return
                # 如果避障过程没有浇水，下面程序正常运行，task_A()中i的循环进程无需变动
            elif not t:  # 没有执行避障 or 避障过程没检测到花盆进而浇水了
                pid_output = self.PID2.update2(want_yaw, current_yaw)
                self.cmd_vel_msg.angular.z = pid_output
                self.cmd_vel_msg.linear.x = B_SPEED
                self.cmd_vel_pub.publish(self.cmd_vel_msg)
                if self.min_dis[1]<B_huapen_thresh and self.min_dis[1] != 0:
                    print("检测到花盆")
                    print("Left : %f   Right : %f",self.min_dis[0],self.min_dis[1])
                    print("结束循环！！！！！！！")
                    break
                else:
                    continue
    #由于B区花盆较长，走出花盆过程中可能遇到障碍物，需增加避障（no_detect，仅避障不检测花盆浇水）
    def B_dotime_180_020_huapen(self):#小车在B区直线行驶,在喷完一个花盆后，越过一个花盆！！！！！走出长花盆过程中需增加避障功能（no_detect）
        print("...................开始走出花盆.................")
        global want_yaw,current_yaw,scope_law
        want_yaw=-180.0
        while True:
            current_yaw,scope_law=self.read_imu_data()
            if self.obstacle_detected:
                print("!!!!检测到障碍物！！！！")
                #此段路程不可能检测花盆浇水
                self.B_execute_avoidance_no_detect()
                continue  # 是不是有没有都一样？？？？？？
            pid_output = self.PID2.update2(want_yaw, current_yaw)
            self.cmd_vel_msg.angular.z = pid_output
            self.cmd_vel_msg.linear.x = B_SPEED
            self.cmd_vel_pub.publish(self.cmd_vel_msg)
            if self.min_dis[1]>B_huapen_thresh:
                break
            else:
                continue
        print("...................已经走出花盆.................")
    def B_dotime_180_020end(self):#小车在B区的最后一个花盆之后，十字之前的直线行驶
        print("...................开始前往终点.................")
        global want_yaw,current_yaw,start_time1,scope_law
        want_yaw = -180.0
        start_time=rospy.get_time()
        while True:
            current_yaw,scope_law=self.read_imu_data()

            #优先避障，如果检测到障碍，会“停止”花盆检测，continue跳过了下面的花盆检测
            #if self.obstacle_detected:
            #   self.execute_avoidance()
            #   continue

            pid_output = self.PID2.update2(want_yaw,current_yaw)        
            self.cmd_vel_msg.angular.z = pid_output
            self.cmd_vel_msg.linear.x = B_SPEED
            self.cmd_vel_pub.publish(self.cmd_vel_msg)
            if  rospy.get_time()-start_time>diercizhuanwanqiandezhixing_time:
                break
            else:
                continue
        print("...................已到达终点.................")


    def control_irrigation(self, on_duration, off_duration, repeat_count):
        """
        控制继电器打开和关闭，支持设置继电器的打开时间、关闭时间和连续执行次数。

        :param on_duration: 继电器保持打开的时间（秒）
        :param off_duration: 继电器保持关闭的时间（秒）
        :param repeat_count: 继电器连续执行的次数
        """
        try:
            # 打开串口连接
            with serial.Serial(SERIAL_PORT4, BAUD_RATE4, timeout=1) as ser:
                for i in range(repeat_count):
                    print("第 {} 次：正在打开继电器...".format(i + 1))
                    ser.write(TURN_ON_RELAY)  # 发送打开继电器的指令
                    time.sleep(on_duration)  # 继电器保持打开指定的时间（秒）

                    print("第 {} 次：正在关闭继电器...".format(i + 1))
                    ser.write(TURN_OFF_RELAY)  # 发送关闭继电器的指令
                    time.sleep(off_duration)  # 继电器保持关闭指定的时间（秒）

        except serial.SerialException as e:
            print("串口打开错误: {}".format(e))
        except Exception as e:
            print("发生了错误: {}".format(e))

    def task_A(self):

        print(COLOR_YELLOW+ ">>>>>Start Task A." + STYLE_RESET)
        for i in range(7):
            print(COLOR_YELLOW+ ">>>>>>>>>>>Task A No"+str(i+1) +" ..."+ STYLE_RESET)
            if i<3:
                 self.A_dotime_0_015()
            if i == 3:
                 self.A_dotime_0_015end()
            elif i == 4:
                 self.A_dotime_right_90()
            elif i == 5:
                 self.A_B_dotime_right_90_015()
            elif i == 6:
                 self.B_dotime_right_180()
            print(COLOR_YELLOW+ ">>>>>>>>>>>Move. Speed is "+str(self.linear_x) +"m/s. Move time is "\
                                        +str(self.go_along_time[self.task_A_go_along[i]][0])+"s. Stop time is "\
                                            +str(self.go_along_time[self.task_A_go_along[i]][2])+"s."+ STYLE_RESET)
            # 可以尝试把延时换为激光雷达左右两侧距离数据的判断
            #self.time_delay(self.go_along_time[self.task_A_go_along[i]][0])
            # 打印激光雷达两侧的距离数据，单位：m
            rospy.loginfo("Left Dis = %fm, Right Dis = %fm", self.min_dis[0], self.min_dis[1])



            if i<3:
                if i == self.current_arm_index:#灌溉执行的标志位并没因避障过程浇水而+1

                    print("当前 i 与 self.current_arm_index", i, "....", self.current_arm_index)
                    print("i = index：执行正常浇水")
                    while True:
                        print("卡task_A()在循环里了！！")
                        if self.min_dis[0] < A_huapen_thresh and self.min_dis[0] != 0:
                            print(COLOR_YELLOW+ ">>>>>>>>>>>Stop."+ STYLE_RESET)
                            self.dotime_stop()
                            break
                        else:
                            continue
                    #执行正常浇水
                    self.A_arm_execute()#封装机械臂运行动作组
                    # 执行完浇水后，走出这个花盆
                    self.A_dotime_0_015_huapen()#执行完浇水后，走出这个花盆
                else:
                    print("当前 i 与 self.current_arm_index",i,"....",self.current_arm_index)
                    print("i != index：避障过程已执行过浇水")
                    print("在task_A()中跳过本次机械臂执行")
                    continue#越过后面，直接进入下次判断i的循环
            elif i==3:
                self.time_delay(0.1)
                self.dotime_stop()
            elif i==4:
                self.time_delay(1.0)
                self.dotime_stop()
            elif i==5:
                self.time_delay(0.1)
                self.dotime_stop()
            else:
                self.time_delay(1.0)
                self.dotime_stop()
        self.current_arm_index = 0#task_A执行完毕，要把self.current_arm_index初始化，留给task_B使用
        print(COLOR_YELLOW+ ">>>>>Task A finished." + STYLE_RESET)
    def task_B(self):
        print(COLOR_YELLOW + ">>>>>Start Task B." + STYLE_RESET)

        for i in range(4):
            print(COLOR_YELLOW + ">>>>>>>>>>>Task B No" + str(i + 1) + " ..." + STYLE_RESET)
            if i < 3:
                self.time_delay(0.1)
                self.B_dotime_180_020()
            elif i == 3:
                self.B_dotime_180_020end()
            print(COLOR_YELLOW + ">>>>>>>>>>>Move. Speed is " + str(self.linear_x) + "m/s. Move time is " \
                  + str(self.go_along_time[self.task_B_go_along[i]][0]) + "s. Stop time is " \
                  + str(self.go_along_time[self.task_B_go_along[i]][2]) + "s." + STYLE_RESET)

            # 可以尝试把延时换为激光雷达左右两侧距离数据的判断
            # self.time_delay(self.go_along_time[self.task_A_go_along[i]][0])
            # 打印激光雷达两侧的距离数据，单位：m
            rospy.loginfo("Left Dis = %fm, Right Dis = %fm", self.min_dis[0], self.min_dis[1])

            if i < 3:
                if i == self.current_arm_index:  # 灌溉执行的标志位并没因避障过程浇水而+1

                    print("当前 i 与 self.current_arm_index", i, "....", self.current_arm_index)
                    print("i = index：执行正常浇水")
                    while True:
                        print("卡task_B()在循环里了！！")
                        if self.min_dis[1] < B_huapen_thresh and self.min_dis[1] !=0:
                            print(COLOR_YELLOW + ">>>>>>>>>>>Stop." + STYLE_RESET)
                            self.dotime_stop()
                            break
                        else:
                            continue
                    # 执行正常浇水
                    self.B_arm_execute()  # 封装机械臂运行动作组
                    # 执行完浇水后，走出这个花盆
                    self.B_dotime_180_020_huapen()  # 执行完浇水后，走出这个花盆
                else:
                    print("当前 i 与 self.current_arm_index", i, "....", self.current_arm_index)
                    print("i != index：避障过程已执行过浇水")
                    print("在task_A()中跳过本次机械臂执行")
                    continue  # 越过后面，直接进入下次判断i的循环
            elif i == 3:
                self.time_delay(0.1)
                self.dotime_stop()
        self.current_arm_index = 0  # task_A执行完毕，要把self.current_arm_index初始化，留给task_C使用
        print(COLOR_YELLOW + ">>>>>Task B finished." + STYLE_RESET)



    def check_obstacles(self):  # 只负责实时设置标志位self.obstacle_detected，并配置避障动作所需的self.avoidance_direction，self.avoidance_start_time
        # 正常直行赛道速度A_Speed，可以设置A_end到转向之间专属的避障函数，速度略快，弥补“障碍物放在最后一个花盆与拐点之间”固定时间点无法及时到达的缺点
        """检测障碍物并设置避障标志"""

        # 如果已经在避障过程中，则不重复检测：
        #方法一：时间检测
        #if self.obstacle_detected and (rospy.get_time() - self.avoidance_start_time < self.avoidance_duration):
        #    return  # 如果标志位为1且rospy.get_time() - self.avoidance_start_time < self.avoidance_duration，则判定正处于避障过程中，直接终结check程序
        #self.obstacle_detected = False

        #方法二：使用另一个“正在执行避障”的标志位：self.executing_avoidance = False

        # 左侧有障碍物(防止避障时撞到左侧)
        if self.obstacle_detected and self.executing_avoidance:#如果发现障碍物并且避障动作正在执行，直接结束此处check_obstacles函数
            return  #如果运行，下面将不再执行

        # 左侧存在障碍物：左侧障碍距离小于安全阈值
        if self.area_left_min_dis < self.side_obstacle_thresh and self.area_left_min_dis!=0:
            print("Left_Obstacle_Detected!!!")
            print("避障距离：Left: %.2f m,  Right: %.2f m"%(self.area_left_min_dis,self.area_right_min_dis))
            self.obstacle_detected = True  # 标志位置1
            self.avoidance_direction = 1  # 标志：向左画弧
            self.avoidance_c_time = 1  # 横向直行时间：标准不放大
            self.avoidance_start_time = rospy.get_time()

        # 左侧存在障碍物：右侧障碍距离小于安全阈值
        elif self.area_right_min_dis < self.side_obstacle_thresh and self.area_right_min_dis!=0:
            print("Right_Obstacle_Detected!!!")
            print("避障距离：Left: %.2f m, Right: %.2f m" % (self.area_left_min_dis,self.area_right_min_dis))
            self.obstacle_detected = True
            self.avoidance_direction = -1  # 标志：向右画弧
            self.avoidance_c_time = 1  # 横向直行时间：标准不放大
            self.avoidance_start_time = rospy.get_time()

        # 无障碍物时，设置直行 or 什么都不设置
        else:#不需要再次重置self.obstacle_detected和self.executing_avoidance，因为如果这两个置True了，execute_avoid()会自动执行完后重置
            self.avoidance_direction = 0  # 标志：向右画弧
            self.avoidance_c_time = 0  # 横向直行时间：标准不放大
        #    self.cmd_vel_msg.linear.x = 0.3  # 正常直行赛道速度A_Speed，可以略快，弥补“障碍物放在最后一个花盆与拐点之间”固定时间点无法及时到达的缺点
        #    self.cmd_vel_msg.angular.z = 0.0
        #    rospy.loginfo("Clear path, moving forward")

    def get_laser_min_dis(self, scan_data):
        # 先找左右两侧最小距离
        # 获取角度的索引，例如角度为30度
        left_desired_degrees = self.left_dis_angle  # self.left_dis_angle与self.right_dis_angle都是init中定义的角度值
        right_desired_degrees = self.right_dis_angle
        left_desired_angle_rad = left_desired_degrees / 180.0 * PI
        right_desired_angle_rad = right_desired_degrees / 180.0 * PI

        # scan_data.angle_min激光起始角度（通常为-π）
        # scan_data.angle_increment角度分辨率（相邻激光束间的角度差）
        # 计算180°方向在激光数组中的索引
        left_angle_index = int((left_desired_angle_rad - scan_data.angle_min) / scan_data.angle_increment)

        # 计算0°方向在激光数组中的索引
        right_angle_index = int((right_desired_angle_rad - scan_data.angle_min) / scan_data.angle_increment)

        # rospy.loginfo("angle_min = %f, angle_max = %f, angle_increment = %f", scan_data.angle_min, scan_data.angle_max, scan_data.angle_increment)
        # rospy.loginfo("left_angle_index = %d, right_angle_index = %d",left_angle_index, right_angle_index)

        if 0 <= left_angle_index < len(scan_data.ranges):
            left_dis = min(scan_data.ranges[left_angle_index:left_angle_index + 2])  # 取连续2个激光点的最小值（提高鲁棒性）
            # rospy.loginfo("Left Dis at %f degrees: %f meters", left_desired_degrees, left_dis)
        else:
            left_dis = 10.0  # 无效数据默认值
            # rospy.logwarn("Left Desired angle is out of range")

        if 0 <= right_angle_index < len(scan_data.ranges):
            right_dis = min(scan_data.ranges[right_angle_index:right_angle_index + 2])  # 取连续2个激光点的最小值（提高鲁棒性）
            # rospy.loginfo("Right Dis at %f degrees: %f meters", right_desired_degrees, right_dis)
        else:
            right_dis = 10.0  # 无效数据默认值
            # rospy.logwarn("Left Desired angle is out of range")
        self.min_dis = [left_dis, right_dis]

        # 再找避障两侧最小距离
        # 获取各区域最小距离

        left_area_desired = self.left_area
        right_area_desired = self.right_area


        left_area_desired_rad = [x / 180.0 * PI for x in left_area_desired]
        right_area_desired_rad = [x / 180.0 * PI for x in right_area_desired]


        left_area_index = [int((x - scan_data.angle_min) / scan_data.angle_increment) for x in left_area_desired_rad]
        right_area_index = [int((x - scan_data.angle_min) / scan_data.angle_increment) for x in right_area_desired_rad]


        self.area_left_min_dis = min(
            scan_data.ranges[i] for i in left_area_index)  # self.area_left_min_dis 需要在 __init__ 内部定义
        self.area_right_min_dis = min(
            scan_data.ranges[i] for i in right_area_index)  # self.area_right_min_dis 需要在 __init__ 内部定义
        #每时每刻查看障碍物（每时每刻执行回调函数）
        self.check_obstacles()

    def __init__(self):

        self.cmd_vel_topic = rospy.get_param('cmd_vel_topic', '/cmd_vel')
        self.cmd_arm_topic = rospy.get_param('cmd_arm_topic', '/arm_cmd')  ###teb

        self.go_along_time = rospy.get_param("go_along_time",
                                             [[0.5, 20.0, 0], [0.62, 2.0, 0], [1.5, 30.0, 0], [0.2, 5.0, 0],
                                              [1.1, 10.0, 0], [1.6, 10.0, 0], [1.3, 30.0, 0], [15.0, 30.0, 4.0]])
        self.task_A_go_along = rospy.get_param("task_A_go_along", [2, 0, 0, 0, 0, 0, 0, 1, 2, 1])
        self.task_B_go_along = rospy.get_param("task_B_go_along", [2, 0, 0, 0, 0, 0, 3, 4, 5, 6])
        self.task_C_go_along = rospy.get_param("task_C_go_along", [2, 0, 0, 0, 0, 0, 0, 1, 2, 1])
        self.task_D_go_along = rospy.get_param("task_D_go_along", [2, 0, 0, 2])

        self.area_A_dry = [[1, 2],[2, 1], [3, 3]]#self.area_A_dry[self.current_arm_index][0]表示第...行小车左侧的花盆干旱程度
        self.area_B_dry = [[2, 2], [3, 3], [1, 2]]
        #self.area_C_dry = [[1, 3], [3, 1], [2, 2]]
        #self.area_D_dry = [3, 2, 2, 3, 1, 1]

        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        self.cmd_arm_pub = rospy.Publisher(self.cmd_arm_topic, Int32, queue_size=10)

        #索引，用于定位封装浇水动作组，在task_A()中与循环变量i同步，但是会因避障过程中浇水动作而与i产生不同
        self.current_arm_index = 0


        # 订阅雷达数据，获取左右两侧障碍物的最小距离
        # 检测花盆所需参数
        self.left_dis_angle = 180
        self.right_dis_angle = 0
        self.min_dis = [10, 10]  # 足够大出检测范围，防止初始化后直接“符合障碍物检测条件”
        #min_dis[0]是左侧距离，min_dis[1]是右侧距离

        # 避障所需参数
        # 定义检测角度范围，正左侧是180度，正右侧是0度（！！赋值过程要注意区域重叠！！）
        self.left_area = list(range(91,157))  # self.left_area 需要在 __init__ 内部定义
        self.right_area = list(range(23,90))  # self.right_area 需要在 __init__ 内部定义
        # 避障过程实时检测值
        self.area_left_min_dis = 10  # 足够大出检测范围，防止初始化后直接“符合障碍物检测条件”
        self.area_right_min_dis = 10  # 足够大出检测范围，防止初始化后直接“符合障碍物检测条件”
        # 障碍物检测阈值(单位:米)
        self.side_obstacle_thresh = 0.25  # 障碍物检测阈值
        #避障所需标志位
        self.obstacle_detected = False  # 初始化为false，防止一运行就“检测到障碍物”
        self.executing_avoidance = False #初始化为false，表示避障动作是否正在执行
        #避障所需系数
        self.avoidance_direction = 0  # 0:未选择, “-1”:障碍在右, “1”:障碍在左
        self.avoidance_c_time = 0#决定“LL”横向直行时间的时间系数
        #同时！！！！！self.avoidance_c_time与self.avoidance_direction还有另一个作用！！！！
        #标志位，用于判定机械臂是在直行过程中浇水还是向左避障过程中浇水还是向左避障过程中浇水
        #决定机械臂的动作组，避障过程中，动作组需要调整：
        #self.avoidance_c_time=1，self.avoidance_direction=1标准向右避障时浇水，左边浇得远，右边浇得近
        # 反之，self.avoidance_c_time=1，self.avoidance_direction=-1标准向左避障时浇水，左边浇得近，右边浇得远
        #同时，self.avoidance_c_time>1时（如1.5），表示横移距离更远，机械臂也应伸得更远

        #避障时间存储变量
        self.avoidance_start_time = 0
        self.avoidance_duration = 1.5  # 避障动作持续时间

        rospy.Subscriber('/scan', LaserScan, self.get_laser_min_dis)

        # 水泵运行时间 s
        self.pump_on_time = rospy.get_param("pump_on_time", 0.5)
        # 水泵关闭时间 s
        self.pump_off_time = rospy.get_param("pump_off_time", 1.0)

        self.cmd_vel_msg = Twist()
        self.linear_x = rospy.get_param('linear_x', 0.2)
        self.anglular_z = rospy.get_param('anglular_z', 2.5)
        self.cmd_arm_msg = Int32()

        # self.timer_vel = rospy.Timer(rospy.Duration(0.05),self.ljy_vel)###定时器发布所有速度
        # 初始化串行端口，将其设置为类的属性
        self.imu_serial = None
        self.PID1 = PID(Kp=10.0, Ki=0.0, Kd=0.0)
        self.PID2 = PID(Kp=10.0, Ki=0.0, Kd=0.0)
        self.PID3 = PID(Kp=10.0, Ki=0.0, Kd=0.0)
        self.imu_serial = serial.Serial('/dev/ttyUSB3', 9600, timeout=0.5)  # imu传感器初始化

    def draw_rectangles(self, columns, data_lists, title):
        """
        绘制矩形图。
        columns: 矩形列数
        data_lists: 每列对应的数据列表
        title: 图表标题
        """
        fig, ax = plt.subplots()
        ax.set_xlim(0, columns * 2)  # 每列宽度为2，列数为columns
        ax.set_ylim(0, 6)  # 每列高度为12，共6行
        ax.set_title(title)

        # 颜色映射：1-绿色，2-蓝色，3-红色
        color_map = {1: "green", 2: "blue", 3: "red"}

        # 绘制每列矩形
        for col in range(columns):
            data = data_lists[col]
            for row in range(3):
                value = data[row]
                color = color_map.get(value, "gray")  # 如果值不在映射中，则默认为灰色
                y_pos = 4 - row * 2
                rect = patches.Rectangle((col * 2, y_pos), 1.5, 1.5, color=color)
                ax.add_patch(rect)
                ax.text(col * 2 + 0.75, y_pos + 0.75, str(value), ha='center', va='center', fontsize=10, color="white")

        plt.axis("off")
        plt.show()

    def RobotStart(self):
        # 初始化串行端口状态标志
        serial_ports_initialized = False
        # 尝试初始化串行端口，直到成功或超过最大重试次数
        max_retries = 5
        retries = 0
        while not serial_ports_initialized and retries < max_retries:
            try:
                self.imu_serial = serial.Serial('/dev/ttyUSB3', 9600, timeout=0.5)
                # 检查端口是否成功初始化
                if self.imu_serial:
                    serial_ports_initialized = True
                else:
                    raise ValueError("One serial ports failed to initialize.")
            except Exception as e:
                rospy.logerr("Error initializing serial ports: {}".format(e))
                retries += 1
                time.sleep(1)  # 等待一秒后重试

        # 如果端口初始化失败，抛出异常或安全退出
        if not serial_ports_initialized:
            rospy.logerr("Failed to initialize all required serial ports. Shutting down.")
            raise RuntimeError("Serial port initialization failed after retries.")
        # 程序主体
        try:
            # data = self.receive_data()
            # print("接收到的数据为:", data)
            # area_A_dry, area_B_dry, area_C_dry, area_D_dry = data

            #print("area_A_dry:", area_A_dry)
            #print("area_B_dry:", area_B_dry)
            # print("area_C_dry:", area_C_dry)
            # print("area_D_dry:", area_D_dry)
            #self.area_A_dry = area_A_dry
            #self.area_B_dry = area_B_dry
            # self.area_C_dry = area_C_dry
            # self.area_D_dry = area_D_dry

            # 用于省赛：在任务执行前绘制个矩形
            #self.draw_rectangles(2, [self.area_A_dry, self.area_B_dry], "Initial Display of A, B")
            # 执行任务
            
            #self.task_A()
            
            
            self.task_B()
            # self.task_C()
            # self.task_D()

            # 在执行完所有任务后绘制2矩形
            # self.draw_rectangles(4, [self.area_A_dry, self.area_B_dry, self.area_C_dry, self.area_D_dry], "Updated Display of A, B, C, D after task_D")
        except rospy.ROSInterruptException:
            rospy.loginfo("Program interrupted.")
        except KeyboardInterrupt:
            print("Shutting down")

        # except Exception as e:
        #    rospy.logerr(f"An error occurred during task execution: {e}")


# main function
if __name__ == "__main__":
    rospy.init_node('compitation_node', anonymous=True)
    rospy.loginfo('compitation_irrigate_node instruction start...')
    try:
        MA = MOVE_ARRIVE()
        MA.RobotStart()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted.")

    except KeyboardInterrupt:
        print("Shutting down")

