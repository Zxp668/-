#! /usr/bin/env python
# -*- coding: utf-8 -*-
import os
import binascii
import time
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
diyicizhuanwanqiandezhixing_time=4.9 #A区后区
diyicizhuanwanhoudezhixing_time=3.9
diyicizhuanwan_law=1.3#A区和B区第一个白色十字转弯角速度

B_SPEED=0.25
diercizhuanwanqiandezhixing_time=2.35#B区后区
diercizhuanwanhoudezhixing_time=4.0
diercizhuanwan_law=1.3#B区第二个白色十字和C区第一个白色十字转弯角速度

C_jiupian=0.0
C_SPEED=0.25
disancizhuanwanqiandezhixing_time=2.9#C区后区
disancizhuanwanhoudezhixing_time=3.9
disancizhuanwan_law=1.3#C区第一个白色十字和D区第一个白色十字转弯角速度

D_jiupian=-180.0
D_SPEED=0.15
zhongdian_time=3.0 #D区在白色终点区域停下来

class PID:
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
        self.cmd_vel_pub.publish(self.cmd_vel_msg)


    def A_dotime_0_015(self):#小车在A区直线行驶
        global want_yaw,current_yaw,scope_law
        want_yaw = 0.0     
        while True:
            current_yaw,scope_law=self.read_imu_data()
            #优先避障，如果检测到障碍，会“停止”花盆检测，continue跳过了下面的花盆检测
            if self.obstacle_detected:
               self.execute_avoidance()
               continue
            #正常运行
            pid_output = self.PID1.update1(want_yaw,current_yaw)        
            self.cmd_vel_msg.angular.z = pid_output
            self.cmd_vel_msg.linear.x = A_SPEED
            self.cmd_vel_pub.publish(self.cmd_vel_msg)

            if self.min_dis[0]<0.4:
                break
            else:
                continue

    def A_dotime_0_015_huapen(self):#小车在A区直线行驶,喷完一个花盆后走出这个花盆
        global want_yaw,current_yaw,scope_law
        want_yaw = 0.0     
        while True:
            current_yaw,scope_law=self.read_imu_data()

            #优先避障，如果检测到障碍，会“停止”花盆检测，continue跳过了下面的花盆检测
            if self.obstacle_detected:
               self.execute_avoidance()
               continue

            pid_output = self.PID1.update1(want_yaw,current_yaw)        
            self.cmd_vel_msg.angular.z = pid_output
            self.cmd_vel_msg.linear.x = A_SPEED
            self.cmd_vel_pub.publish(self.cmd_vel_msg)      
            if self.min_dis[0]>0.4:
                break
            else:
                continue

    def A_dotime_0_015end(self):#小车在A区的最后一个花盆之后，十字之前的直线行驶
        global want_yaw,current_yaw,start_time1,scope_law
        want_yaw = 0.0     
        start_time1=rospy.get_time()
        while True:
            current_yaw,scope_law=self.read_imu_data()

            #优先避障，如果检测到障碍，会“停止”花盆检测，continue跳过了下面的花盆检测
            if self.obstacle_detected:
               self.execute_avoidance()
               continue
            pid_output = self.PID1.update1(want_yaw,current_yaw)        
            self.cmd_vel_msg.angular.z = pid_output
            self.cmd_vel_msg.linear.x = 0.15
            self.cmd_vel_pub.publish(self.cmd_vel_msg) 
            if  rospy.get_time()-start_time1>diyicizhuanwanqiandezhixing_time:
                break
            else:
                continue

    def A_dotime_right_90(self):#小车在A区白色十字向右旋转刚好90度
        global want_yaw,current_yaw,scope_law
        want_yaw = -90.0
        print("A to B:first 90-turn")#renwujincheng
        while True:
            current_yaw,scope_law=self.read_imu_data()
            print("current_yaw:%f ",current_yaw)#baogao jiaodu
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
            print("current_yaw:%f ",current_yaw)#baogao jiaodu
            pid_output = self.PID1.update1(want_yaw,current_yaw)        
            self.cmd_vel_msg.angular.z = pid_output
            self.cmd_vel_msg.linear.x = 0.15
            self.cmd_vel_pub.publish(self.cmd_vel_msg)
            if rospy.get_time()-start_time1>diyicizhuanwanhoudezhixing_time:
                break
            else:
                continue

    def B_dotime_right_180(self):#小车在B区第一个白色十字右转刚好90度，与初始角度成180度
        global want_yaw,current_yaw,scope_law
        want_yaw=-180.0
        print("A to B:second 90-turn")#renwujincheng
        while True:
            current_yaw,scope_law=self.read_imu_data()
            print("current_yaw:%f ",current_yaw)#baogao jiaodu
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
            if abs(current_yaw-want_yaw)<5.0:
                self.cmd_vel_msg.angular.z = 0.0
                self.cmd_vel_msg.linear.x = 0.0
                self.cmd_vel_pub.publish(self.cmd_vel_msg)
                break
            else:
                continue

    def B_dotime_180_020(self):#小车在B区直线行驶
        print("正在运行B_dotime_180_020#小车在B区直线行驶")
        global want_yaw,current_yaw,scope_law
        want_yaw=-180.0
        while True:
            print("循环ing...")
            
            current_yaw,scope_law=self.read_imu_data()

            #优先避障，如果检测到障碍，会“停止”花盆检测，continue跳过了下面的花盆检测
            if self.obstacle_detected:
               self.execute_avoidance()
               continue
          
            if self.min_dis[1]<0.65:
                print("Left : %f   Right : %f",self.min_dis[0],self.min_dis[1])
                print("结束循环！！！！！！！")
                break
            else:
                pid_output = self.PID2.update2(want_yaw,current_yaw)        
                self.cmd_vel_msg.angular.z = pid_output
                self.cmd_vel_msg.linear.x = B_SPEED
                self.cmd_vel_pub.publish(self.cmd_vel_msg)        
                continue 

    def B_dotime_180_020_huapen(self):#小车在B区直线行驶,在喷完一个花盆后，越过一个花盆
        print("正在运行B_dotime_180_020_huapen#小车在B区直线行驶,在喷完一个花盆后，越过一个花盆")
        global want_yaw,current_yaw,scope_law
        want_yaw=-180.0
        while True:      
            print("循环ing...")
            print("Left : %f   Right : %f",self.min_dis[0],self.min_dis[1])
            current_yaw,scope_law=self.read_imu_data()

            #优先避障，如果检测到障碍，会“停止”花盆检测，continue跳过了下面的花盆检测
            if self.obstacle_detected:
               self.execute_avoidance()
               continue
           
            if self.min_dis[1]>0.66:
                print("结束循环！！！！！！！")
                break
            else:
                pid_output = self.PID2.update2(want_yaw,current_yaw)        
                self.cmd_vel_msg.angular.z = pid_output
                self.cmd_vel_msg.linear.x = B_SPEED
                self.cmd_vel_pub.publish(self.cmd_vel_msg)
                continue 

    def B_dotime_180_020end(self):#小车在B区的最后一个花盆之后，十字之前的直线行驶
        global want_yaw,current_yaw,start_time1,scope_law
        want_yaw = -180.0
        start_time1=rospy.get_time()
        while True:
            current_yaw,scope_law=self.read_imu_data()

            #优先避障，如果检测到障碍，会“停止”花盆检测，continue跳过了下面的花盆检测
            if self.obstacle_detected:
               self.execute_avoidance()
               continue

            pid_output = self.PID2.update2(want_yaw,current_yaw)        
            self.cmd_vel_msg.angular.z = pid_output
            self.cmd_vel_msg.linear.x = B_SPEED
            self.cmd_vel_pub.publish(self.cmd_vel_msg)
            
            if  rospy.get_time()-start_time1>diercizhuanwanqiandezhixing_time:
                break
            else:
                continue

    def B_dotime_left_90(self):#小车在B区第二个白色十字左转刚好90度
        global want_yaw,current_yaw,scope_law
        want_yaw=-90.0
        while True:
            current_yaw,scope_law=self.read_imu_data()
            if current_yaw>0.0 and current_yaw<180.0:
                self.cmd_vel_msg.angular.z = diercizhuanwan_law
                self.cmd_vel_msg.linear.x = 0.0    
                self.cmd_vel_pub.publish(self.cmd_vel_msg)         
            elif current_yaw>-180.0 and current_yaw<=want_yaw-5.0:
                self.cmd_vel_msg.angular.z = diercizhuanwan_law
                self.cmd_vel_msg.linear.x = 0.0      
                self.cmd_vel_pub.publish(self.cmd_vel_msg)
            elif current_yaw<0.0 and current_yaw>=want_yaw+5.0:
                self.cmd_vel_msg.angular.z = -diercizhuanwan_law
                self.cmd_vel_msg.linear.x = 0.0
                self.cmd_vel_pub.publish(self.cmd_vel_msg)
            else:
                self.cmd_vel_msg.angular.z = 0.0
                self.cmd_vel_msg.linear.x = 0.0
                self.cmd_vel_pub.publish(self.cmd_vel_msg)
            if abs(current_yaw-want_yaw)<5.0:
                self.cmd_vel_msg.angular.z = 0.0
                self.cmd_vel_msg.linear.x = 0.0
                self.cmd_vel_pub.publish(self.cmd_vel_msg)
                break
            else:
                continue                                                  

    def B_C_dotime_right_90_015(self):#小车在B区白色十字到达C区第一个白色十字的直线行驶过程
        global want_yaw,current_yaw,start_time1,scope_law
        want_yaw = -90.0
        start_time1=rospy.get_time()
        while True:
            current_yaw,scope_law=self.read_imu_data()
            pid_output = self.PID1.update1(want_yaw,current_yaw)        
            self.cmd_vel_msg.angular.z = pid_output
            self.cmd_vel_msg.linear.x = 0.15
            self.cmd_vel_pub.publish(self.cmd_vel_msg)
            if rospy.get_time()-start_time1>diercizhuanwanhoudezhixing_time:
                break
            else:
                continue

    def C_dotime_left_0(self): #小车在C区第一个白色十字左转刚好90度
        global want_yaw,current_yaw,scope_law
        want_yaw = 0.0     
        while True:
            current_yaw,scope_law=self.read_imu_data()
            if  current_yaw<want_yaw:
                self.cmd_vel_msg.angular.z = diercizhuanwan_law
                self.cmd_vel_msg.linear.x = 0.0
                self.cmd_vel_pub.publish(self.cmd_vel_msg)
            elif current_yaw>want_yaw: 
                self.cmd_vel_msg.angular.z = -diercizhuanwan_law
                self.cmd_vel_msg.linear.x = 0.0
                self.cmd_vel_pub.publish(self.cmd_vel_msg)
            else:
                self.cmd_vel_msg.angular.z = 0.0
                self.cmd_vel_msg.linear.x = 0.0    
                self.cmd_vel_pub.publish(self.cmd_vel_msg)  
            if abs(current_yaw-want_yaw)<5.0:
                self.cmd_vel_msg.angular.z = 0.0
                self.cmd_vel_msg.linear.x = 0.0
                self.cmd_vel_pub.publish(self.cmd_vel_msg)
                break
            else:
                continue

  
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
                while True:
                    if self.min_dis[0]<0.50:
                        self.dotime_stop()
                        break
                    else:
                        continue
            elif i==3:
                self.time_delay(0.1)
                self.dotime_stop()
            elif i==4:
                self.time_delay(1.0)
                self.dotime_stop()
            elif i==5:
                self.A_B_dotime_right_90_015()
                self.time_delay(0.1)
                self.dotime_stop()
            else:
                self.time_delay(1.0)
                self.dotime_stop()
            
            print(COLOR_YELLOW+ ">>>>>>>>>>>Stop."+ STYLE_RESET)
            if i<3:
                print(COLOR_YELLOW+ ">>>>>>>>>>>Arm active first."+ STYLE_RESET)
                self.cmd_arm_msg.data = 2
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(1.0)
                print(COLOR_YELLOW+ ">>>>>>>>>>>Pump work. Times="+str(self.area_A_dry[i])+ STYLE_RESET)
                playsound(A[self.area_A_dry[i]-1])
                self.control_irrigation(self.pump_on_time,self.pump_off_time,self.area_A_dry[i])
                self.time_delay(1.0)
                self.cmd_arm_msg.data = 3
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(5.0)
                print(COLOR_YELLOW+ ">>>>>>>>>>>Arm active second."+ STYLE_RESET)
                self.cmd_arm_msg.data = 4
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(2.0)
                print(COLOR_YELLOW+ ">>>>>>>>>>>Pump work. Times="+str(self.area_A_dry[i])+ STYLE_RESET)
                self.control_irrigation(self.pump_on_time,self.pump_off_time,self.area_A_dry[i])
                self.time_delay(1.0)
                print(COLOR_YELLOW+ ">>>>>>>>>>>Arm reset."+ STYLE_RESET)
                self.cmd_arm_msg.data = 3
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(2.0)
                self.A_dotime_0_015_huapen()
            else:
                self.time_delay(1.0)
        print(COLOR_YELLOW+ ">>>>>Task A finished." + STYLE_RESET)

    def task_B(self):
        print(COLOR_YELLOW+ ">>>>>Start Task B." + STYLE_RESET)
        
        for i in range(7):
            print("dog")
            print("Left : %f   Right : %f",self.min_dis[0],self.min_dis[1])
            print(COLOR_YELLOW+ ">>>>>>>>>>>Task B No"+str(i+1) +" ..."+ STYLE_RESET)
            if i<3:
                self.time_delay(0.1)
                self.B_dotime_180_020()
            elif i == 3:
                 self.B_dotime_180_020end()
            elif i == 4:
                 self.B_dotime_left_90()
            elif i == 5:
                 self.B_C_dotime_right_90_015()
            elif i == 6:
                 self.C_dotime_left_0()
            print(COLOR_YELLOW+ ">>>>>>>>>>>Move. Speed is "+str(self.linear_x) +"m/s. Move time is "\
                                        +str(self.go_along_time[self.task_B_go_along[i]][0])+"s. Stop time is "\
                                            +str(self.go_along_time[self.task_B_go_along[i]][2])+"s."+ STYLE_RESET)
            
            # 可以尝试把延时换为激光雷达左右两侧距离数据的判断
            #self.time_delay(self.go_along_time[self.task_A_go_along[i]][0])
            # 打印激光雷达两侧的距离数据，单位：m
            rospy.loginfo("Left Dis = %fm, Right Dis = %fm", self.min_dis[0], self.min_dis[1])
            if i==0:
                while True:
                    if self.min_dis[1]<0.65:
                        self.dotime_stop()
                        break
                    else:
                        continue
            elif i==1 or i==2 :
                while True:
                    if self.min_dis[1]<0.65:
                        self.dotime_stop()
                        break
                    else:
                        continue
            elif i==3:
                self.time_delay(0.1)
                self.dotime_stop()
            elif i==4:
                self.time_delay(1.0)
                self.dotime_stop()
            elif i==5:
                self.B_C_dotime_right_90_015()
                self.time_delay(0.1)
                self.dotime_stop()
            else:
                self.time_delay(1.0)
                self.dotime_stop()
            print(COLOR_YELLOW+ ">>>>>>>>>>>Stop."+ STYLE_RESET)
            if i<3:
                print(COLOR_YELLOW+ ">>>>>>>>>>>Arm active first."+ STYLE_RESET)
                self.cmd_arm_msg.data = 5
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(1.0)
                print(COLOR_YELLOW+ ">>>>>>>>>>>Pump work. Times="+str(self.area_B_dry[i])+ STYLE_RESET)
                playsound(A[self.area_B_dry[i]-1])
                self.control_irrigation(self.pump_on_time,self.pump_off_time,self.area_B_dry[i])
                self.time_delay(1.0)
                self.cmd_arm_msg.data = 3
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(5.0)
                print(COLOR_YELLOW+ ">>>>>>>>>>>Arm active second."+ STYLE_RESET)
                self.cmd_arm_msg.data = 6
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                self.time_delay(2.0)
                print(COLOR_YELLOW+ ">>>>>>>>>>>Pump work. Times="+str(self.area_B_dry[i])+ STYLE_RESET)
                self.control_irrigation(self.pump_on_time,self.pump_off_time,self.area_B_dry[i])
                self.time_delay(1.0)
                print(COLOR_YELLOW+ ">>>>>>>>>>>Arm reset."+ STYLE_RESET)
                self.cmd_arm_msg.data = 3
                self.cmd_arm_pub.publish(self.cmd_arm_msg)
                print("dog")
                self.time_delay(2.0)
                print("dog")
                self.B_dotime_180_020_huapen()
                print("dog")
            else:
                self.time_delay(1.0)
        print(COLOR_YELLOW+ ">>>>>Task B finished." + STYLE_RESET)


    def check_obstacles(self):#只负责实时设置标志位self.obstacle_detected，并配置避障动作所需的self.avoidance_direction，self.avoidance_start_time
        # 正常直行赛道速度A_Speed，可以设置A_end到转向之间专属的避障函数，速度略快，弥补“障碍物放在最后一个花盆与拐点之间”固定时间点无法及时到达的缺点
        """检测障碍物并设置避障标志"""

        # 左侧有障碍物(防止避障时撞到左侧)

        if self.area_left_min_dis < self.side_obstacle_thresh:#左侧存在障碍物：左侧障碍距离小于安全阈值
            print("Left_Obstacle_Detected!!!")
            print("避障距离：Left: %.2f m, Front: %.2f m, Right: %.2f m"%(self.area_left_min_dis,self.area_front_min_dis,self.area_right_min_dis))
            self.obstacle_detected = True#标志位置1
            self.avoidance_direction = 1  # 向右微调
            self.avoidance_start_time = rospy.get_time()

        # 右侧有障碍物(防止避障时撞到右侧)
        elif self.area_right_min_dis < self.side_obstacle_thresh:#左侧存在障碍物：右侧障碍距离小于安全阈值
            print("Right_Obstacle_Detected!!!")
            print("避障距离：Left: %.2f m, Front: %.2f m, Right: %.2f m"%(self.area_left_min_dis,self.area_front_min_dis,self.area_right_min_dis))
            self.obstacle_detected = True
            self.avoidance_direction = -1  # 向左微调
            self.avoidance_start_time = rospy.get_time()
        elif self.area_front_min_dis < self.side_obstacle_thresh:#前侧存在障碍物：右侧障碍距离小于安全阈值
            print("避障距离：Left: %.2f m, Front: %.2f m, Right: %.2f m"%(self.area_left_min_dis,self.area_front_min_dis,self.area_right_min_dis))
            print("Front_Obstacle_Detected!!!")
            self.obstacle_detected = True
            self.avoidance_direction = -1  # 向左微调
            self.avoidance_start_time = rospy.get_time()

        # 无障碍物时，设置直行 or 什么都不设置
        #else:    #设置直行
        #    self.cmd_vel_msg.linear.x = 0.3  # 正常直行赛道速度A_Speed，可以略快，弥补“障碍物放在最后一个花盆与拐点之间”固定时间点无法及时到达的缺点
        #    self.cmd_vel_msg.angular.z = 0.0
        #    rospy.loginfo("Clear path, moving forward")


    def get_laser_min_dis(self, scan_data):
        #先找左右两侧最小距离
        # 获取角度的索引，例如角度为30度
        left_desired_degrees = self.left_dis_angle  # self.left_dis_angle与self.right_dis_angle都是init中定义的角度值
        right_desired_degrees = self.right_dis_angle
        left_desired_angle_rad = left_desired_degrees / 180.0 * PI
        right_desired_angle_rad = right_desired_degrees / 180.0 * PI

        #scan_data.angle_min激光起始角度（通常为-π）
        #scan_data.angle_increment角度分辨率（相邻激光束间的角度差）
        # 计算180°方向在激光数组中的索引
        left_angle_index = int((left_desired_angle_rad - scan_data.angle_min) / scan_data.angle_increment)

        # 计算0°方向在激光数组中的索引
        right_angle_index = int((right_desired_angle_rad - scan_data.angle_min) / scan_data.angle_increment)

        # rospy.loginfo("angle_min = %f, angle_max = %f, angle_increment = %f", scan_data.angle_min, scan_data.angle_max, scan_data.angle_increment)
        # rospy.loginfo("left_angle_index = %d, right_angle_index = %d",left_angle_index, right_angle_index)

        if 0 <= left_angle_index < len(scan_data.ranges):
            left_dis = min(scan_data.ranges[left_angle_index:left_angle_index+2])# 取连续2个激光点的最小值（提高鲁棒性）
            #rospy.loginfo("Left Dis at %f degrees: %f meters", left_desired_degrees, left_dis)
        else:
            left_dis = 10.0 # 无效数据默认值
            #rospy.logwarn("Left Desired angle is out of range")

        if 0 <= right_angle_index < len(scan_data.ranges):
            right_dis = min(scan_data.ranges[right_angle_index:right_angle_index+2])# 取连续2个激光点的最小值（提高鲁棒性）
            #rospy.loginfo("Right Dis at %f degrees: %f meters", right_desired_degrees, right_dis)
        else:
            right_dis = 10.0 # 无效数据默认值
            #rospy.logwarn("Left Desired angle is out of range")
        self.min_dis = [left_dis, right_dis]

        # 再找避障两侧最小距离
        # 获取各区域最小距离
        front_area_desired = self.front_area
        left_area_desired = self.left_area
        right_area_desired = self.right_area
        
        front_area_desired_rad = [x /180.0 * PI for x in front_area_desired]
        left_area_desired_rad = [x /180.0 * PI for x in left_area_desired]
        right_area_desired_rad = [x /180.0 * PI for x in right_area_desired]
       
        front_area_index = [int((x - scan_data.angle_min) / scan_data.angle_increment) for x in front_area_desired_rad]
        left_area_index = [int((x - scan_data.angle_min) / scan_data.angle_increment) for x in left_area_desired_rad]
        right_area_index = [int((x - scan_data.angle_min) / scan_data.angle_increment) for x in right_area_desired_rad]

        self.area_front_min_dis = min(scan_data.ranges[i] for i in front_area_index)#self.area_front_min_dis 需要在 __init__ 内部定义
        self.area_left_min_dis = min(scan_data.ranges[i] for i in left_area_index)#self.area_left_min_dis 需要在 __init__ 内部定义
        self.area_right_min_dis = min(scan_data.ranges[i] for i in right_area_index)#self.area_right_min_dis 需要在 __init__ 内部定义

        self.check_obstacles()

    def __init__(self):

        self.cmd_vel_topic = rospy.get_param('cmd_vel_topic','/cmd_vel')
        self.cmd_arm_topic = rospy.get_param('cmd_arm_topic','/arm_cmd')###teb
     
        self.go_along_time = rospy.get_param("go_along_time",[[0.5,20.0,0],[0.62,2.0,0],[1.5,30.0,0],[0.2,5.0,0],[1.1,10.0,0],[1.6,10.0,0],[1.3,30.0,0],[15.0,30.0,4.0]])
        self.task_A_go_along = rospy.get_param("task_A_go_along",[2,0,0,0,0,0,0,1,2,1])
        self.task_B_go_along = rospy.get_param("task_B_go_along",[2,0,0,0,0,0,3,4,5,6])
        self.task_C_go_along = rospy.get_param("task_C_go_along",[2,0,0,0,0,0,0,1,2,1])
        self.task_D_go_along = rospy.get_param("task_D_go_along",[2,0,0,2])

        self.area_A_dry = [1,2,2,1,3,3]
        self.area_B_dry = [1,2,2,3,3,1]
        self.area_C_dry = [1,3,3,1,2,2]
        self.area_D_dry = [3,2,2,3,1,1]

        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic,Twist,queue_size=10)
        self.cmd_arm_pub = rospy.Publisher(self.cmd_arm_topic,Int32,queue_size=10)

        # 订阅雷达数据，获取左右两侧障碍物的最小距离
        #检测花盆所需参数
        self.left_dis_angle = 180
        self.right_dis_angle = 0
        self.min_dis = [10,10] #足够大出检测范围，防止初始化后直接“符合障碍物检测条件”

        # 避障所需参数
        # 定义检测角度范围，正左侧是180度，正右侧是0度（！！赋值过程要注意区域重叠！！）
        self.front_area = list(range(60, 119))  # self.front_area需要在 __init__ 内部定义
        self.left_area = list(range(120, 180))  # self.left_area 需要在 __init__ 内部定义
        self.right_area = list(range(0, 59))  # self.right_area 需要在 __init__ 内部定义
        #避障过程实时检测值
        self.area_front_min_dis = 10#足够大出检测范围，防止初始化后直接“符合障碍物检测条件”
        self.area_left_min_dis = 10#足够大出检测范围，防止初始化后直接“符合障碍物检测条件”
        self.area_right_min_dis = 10#足够大出检测范围，防止初始化后直接“符合障碍物检测条件”
        # 障碍物检测阈值(单位:米)
        self.side_obstacle_thresh = 0.3  # 障碍物检测阈值

        self.obstacle_detected = False  #初始化为false，防止一运行就“检测到障碍物”
        self.avoidance_direction = 0  # 0:未选择, -1:左转, 1:右转
        self.avoidance_start_time = 0
        self.avoidance_duration = 1.5  # 避障动作持续时间

        rospy.Subscriber('/scan', LaserScan, self.get_laser_min_dis)


        # 水泵运行时间 s
        self.pump_on_time = rospy.get_param("pump_on_time", 0.5)
        # 水泵关闭时间 s
        self.pump_off_time = rospy.get_param("pump_off_time", 1.0)

        self.cmd_vel_msg = Twist()
        self.linear_x = rospy.get_param('linear_x',0.2)
        self.anglular_z = rospy.get_param('anglular_z',2.5)
        self.cmd_arm_msg = Int32()

        #self.timer_vel = rospy.Timer(rospy.Duration(0.05),self.ljy_vel)###定时器发布所有速度
        # 初始化串行端口，将其设置为类的属性
        self.imu_serial = None
        self.PID1 = PID(Kp=10.0, Ki=0.0, Kd=0.0)
        self.PID2 = PID(Kp=10.0, Ki=0.0, Kd=0.0)
        self.PID3 = PID(Kp=10.0, Ki=0.0, Kd=0.0)
        self.imu_serial = serial.Serial('/dev/ttyUSB3', 9600, timeout=0.5)  #imu传感器初始化

    def draw_rectangles(self,columns, data_lists, title):
        """
        绘制矩形图。
        columns: 矩形列数
        data_lists: 每列对应的数据列表
        title: 图表标题
        """
        fig, ax = plt.subplots()
        ax.set_xlim(0, columns * 2)  # 每列宽度为2，列数为columns
        ax.set_ylim(0, 6)           # 每列高度为12，共6行
        ax.set_title(title)

        # 颜色映射：1-绿色，2-蓝色，3-红色
        color_map = {1: "green", 2: "blue", 3: "red"}

        # 绘制每列矩形
        for col in range(columns):
            data = data_lists[col]
            for row in range(3):
                value = data[row]
                color = color_map.get(value, "gray")  # 如果值不在映射中，则默认为灰色
                y_pos = 4-row*2
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
            print("原地障碍检测已经开始，请开始放置障碍物")
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

