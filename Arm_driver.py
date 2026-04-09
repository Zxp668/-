#!/usr/bin/env python
#coding=utf-8

import roslib
import rospy 
from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

import json
import serial
import os
import sys
import select
import termios
import tty
import re
import threading
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

# 保存终端属性
settings = termios.tcgetattr(sys.stdin)

write_pose_flag = 0
write_count = 0
class arm_driver():

    def __init__(self):

        rospy.init_node('base_arm', anonymous=True)   

        self.P = Float32MultiArray()

        self.arm_init_pose = [163, 78,-41 ,91 ,260]

        self.arm_current_pose = [163, 78,-41 ,91 ,260]

        self.arm_current_speed = rospy.get_param("arm_speed",200)

        self.arm_current_acc = rospy.get_param("arm_acc",60)

        self.arm_delay_time = rospy.get_param("arm_delay_time",2.0)

        self.arm_target_pose = [[[]]]

        self.arm_target_pose = rospy.get_param("arm_target_pose",[[[]]])

        self.arm_target_num = []
       
        self.arm_pose_array = []

        self.read_array_from_yaml = []    #定义二位数组读取yaml文件参数

        self.trajectory = rospy.get_param("trajectory",10)

        self.arm_target_num = rospy.get_param("arm_target_num",[]) 

        self.arm_move = rospy.get_param("arm_move",10)

        self.arm_serial_device = rospy.get_param("arm_serial","/dev/gps0")

        self.serial = serial.Serial(self.arm_serial_device,115200)

        self.sub_coord_topic = rospy.get_param("arm_sub_coord_topic","/arm_coord")
  
        self.sub_coord = rospy.Subscriber(self.sub_coord_topic,Float32MultiArray,self.set_coord_callback,queue_size=20)

        self.sub_cmd_topic = rospy.get_param("arm_sub_cmd_topic","/arm_cmd")

        self.sub_cmd = rospy.Subscriber(self.sub_cmd_topic,Int32,self.listener_callback,queue_size=20)

        self.sub_write_cmd = rospy.Subscriber("/read_array",Int32,self.read_array_callback,queue_size=20)

        self.pub_pose_topic = rospy.get_param("arm_pub_pose","/arm_pose")

        self.pub_pose = rospy.Publisher(self.pub_pose_topic,Float32MultiArray,queue_size=10)

        self.timer_get_angle = rospy.Timer(rospy.Duration(300.0/1000),self.timerGetAngleCB)

        self.serial.write(json.dumps({"T":1,"P1":self.arm_init_pose[0],\
                                    "P2":self.arm_init_pose[1],\
                                    "P3":self.arm_init_pose[2],\
                                    "P4":self.arm_init_pose[3],\
                                    "P5":self.arm_init_pose[4],\
                                    "S1":self.arm_current_speed,\
                                    "S2":self.arm_current_speed,\
                                    "S3":self.arm_current_speed,\
                                    "S4":self.arm_current_speed,\
                                    "S5":self.arm_current_speed,\
                                    "A1":self.arm_current_acc,\
                                    "A2":self.arm_current_acc,\
                                    "A3":self.arm_current_acc,\
                                    "A4":self.arm_current_acc,\
                                    "A5":self.arm_current_acc}).encode())
        print(COLOR_GREEN + ">>>>>Current Pose :" + str(self.arm_init_pose) + STYLE_RESET)
        self.M1_recv_thread = threading.Thread(target=self.read_M1_uart)         #创建一个读取串口的第二线程                       
        self.M1_recv_thread.daemon = True
        self.M1_recv_thread.start()
        self.time_delay(2.0)

    def read_M1_uart(self):
            pose_msg = Float32MultiArray()
            while not rospy.is_shutdown(): 
                length = self.serial.in_waiting
                if length:
                    reading = self.serial.readline().decode('utf-8')
                    if len(reading)>10:
                        match=re.findall(r'"A1":',reading)
                        # print(COLOR_PURPLE + "----->reading:" + str(reading)  + STYLE_RESET)
                        if len(match)>0:
                            pattren=re.compile(r'(?<="A1":)-?\d+(?:\.\d+)?')
                            if(len(pattren.findall(reading))>0):
                                self.arm_current_pose[0] = float(pattren.findall(reading)[0])  
                            pattren=re.compile(r'(?<="A2":)-?\d+(?:\.\d+)?')               
                            if(len(pattren.findall(reading))>0):
                                self.arm_current_pose[1] = float(pattren.findall(reading)[0])
                            pattren=re.compile(r'(?<="A3":)-?\d+(?:\.\d+)?')
                            if(len(pattren.findall(reading))>0):
                                self.arm_current_pose[2] = float(pattren.findall(reading)[0])
                            pattren=re.compile(r'(?<="A4":)-?\d+(?:\.\d+)?')
                            if(len(pattren.findall(reading))>0):
                                self.arm_current_pose[3] = float(pattren.findall(reading)[0])
                            pattren=re.compile(r'(?<="A5":)-?\d+(?:\.\d+)?')
                            if(len(pattren.findall(reading))>0):
                                self.arm_current_pose[4] = float(pattren.findall(reading)[0])
                            # print(COLOR_GREEN + ">>>>>Current Pose :" + str(self.arm_current_pose) + STYLE_RESET)
                        pose_msg.data = self.arm_current_pose
                        self.pub_pose.publish(pose_msg)


    def timerGetAngleCB(self,event):
        global write_pose_flag
        if write_pose_flag == 1:
            self.arm_pose_array.append(self.arm_current_pose[:])
            print(COLOR_GREEN + "Now You are starting to record the trajectory with degree------" + str(self.arm_current_pose) + STYLE_RESET)
        elif  write_pose_flag == 2:
             print(COLOR_YELLOW + ">>>>> self.arm_pose_array:" + str(self.arm_pose_array) + STYLE_RESET)
             os.system("echo write_array{}: {} >> /home/epaicar/talos_ws/src/agrc_base_arm/config/pose_array.yaml".format(write_count,self.arm_pose_array))
             rospy.set_param("write_array{}".format(write_count),self.arm_pose_array)
             self.arm_pose_array = []
             write_pose_flag = 0
        self.serial.write(json.dumps({"T":5}).encode())



    def time_delay(self,s):
        start_time = rospy.get_time()
        while (rospy.get_time() - start_time<s):
            xyz=0

    def set_coord_callback(self,msg):
        self.P = msg.data
        self.serial.write(json.dumps({"T":2,"P1":self.P[0],"P2":self.P[1],"P3":self.P[2],"P4":self.P[3],"P5":self.P[4],"S1":self.P[5],"S5":self.P[6]}).encode())



    def posGet(self, radInput, direcInput, multiInput):
        if radInput == 0:
            return 2047
        else:
            getPos = int(2047 + (direcInput * radInput / 3.1415926 * 2048 * multiInput) + 0.5)
            return getPos

    def listener_callback(self, msg):
        a = msg.data

        if a == 1:
            # init pos
            self.serial.write(json.dumps({"T":1,"P1":self.arm_init_pose[0],"P2":self.arm_init_pose[1],"P3":self.arm_init_pose[2],"P4":self.arm_init_pose[3],"P5":self.arm_init_pose[4],\
                                        "S1":self.arm_current_speed,"S2":self.arm_current_speed,"S3":self.arm_current_speed,"S4":self.arm_current_speed,"S5":self.arm_current_speed,\
                                        "A1":self.arm_current_acc,"A2":self.arm_current_acc,"A3":self.arm_current_acc,"A4":self.arm_current_acc,"A5":self.arm_current_acc}).encode())
        elif (a > 1 and a < 20):
            # left work
            for i in range(self.arm_target_num[a-2]):
                self.serial.write(json.dumps({"T":1,"P1":self.arm_target_pose[a-2][i][0],"P2":self.arm_target_pose[a-2][i][1],"P3":self.arm_target_pose[a-2][i][2],"P4":self.arm_target_pose[a-2][i][3],"P5":self.arm_target_pose[a-2][i][4],\
                                    "S1":self.arm_current_speed,"S2":self.arm_current_speed,"S3":self.arm_current_speed,"S4":self.arm_current_speed,"S5":self.arm_current_speed,\
                                    "A1":self.arm_current_acc,"A2":self.arm_current_acc,"A3":self.arm_current_acc,"A4":self.arm_current_acc,"A5":self.arm_current_acc}).encode())
                self.time_delay( self.arm_delay_time)


        elif a == 20:
            print(COLOR_GREEN + ">>>>>Current Pose :" + str(self.arm_current_pose) + STYLE_RESET) #打印获取当前机械臂各个关节角度值

        elif a == 99:
            # ALL_TORQUE_OFF: {"T":8,"P1":0}
            self.serial.write(json.dumps({"T":8,"P1":0}).encode())
        elif a == 100:
            # ALL_TORQUE_ON: {"T":8,"P1":1}
            self.serial.write(json.dumps({"T":8,"P1":1}).encode())
        elif a == 101:
            # ALL_TORQUE_ON: {"T":8,"P1":1}
            self.serial.write(json.dumps({"T":1,"P1":270,"P2":-15,"P3":50,"P4":90,"P5":270,"S1":500,"S2":500,"S3":500,"S4":500,"S5":500,"A1":60,"A2":60,"A3":60,"A4":60,"A5":60}).encode())
        print(a)

    def read_array_callback(self, msg):
        b = msg.data
        temp_array = rospy.get_param("write_array{}".format(b),[[my_arm.arm_init_pose[0], my_arm.arm_init_pose[1], my_arm.arm_init_pose[2], my_arm.arm_init_pose[3], my_arm.arm_init_pose[4]],[my_arm.arm_init_pose[0], my_arm.arm_init_pose[1], my_arm.arm_init_pose[2], my_arm.arm_init_pose[3], my_arm.arm_init_pose[4]]])
        temp_array_size = len(temp_array)
        
        print(COLOR_YELLOW + "TEMP_ARRAY:" + str(temp_array_size) + STYLE_RESET)
        for array in temp_array:
            self.serial.write(json.dumps({"T":1,"P1":array[0],"P2":array[1],"P3":array[2],"P4":array[3],"P5":array[4],\
                                    "S1":self.arm_current_speed,"S2":self.arm_current_speed,"S3":self.arm_current_speed,"S4":self.arm_current_speed,"S5":self.arm_current_speed,\
                                    "A1":self.arm_current_acc,"A2":self.arm_current_acc,"A3":self.arm_current_acc,"A4":self.arm_current_acc,"A5":self.arm_current_acc}).encode())
            print(COLOR_YELLOW + ">>>>>array :" + str(array) + STYLE_RESET)

            self.time_delay(0.2)
   
        


if __name__ == '__main__':  
    try:  
        my_arm = arm_driver()  
        # 设置循环频率
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            # 设置终端属性，以便实时读取键值
            tty.setcbreak(sys.stdin.fileno())
            # tty.setraw(sys.stdin.fileno())
            if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
                # 从终端读取键值
                key = sys.stdin.read(1)
                if key == 'r':
                    my_arm.serial.write(json.dumps({"T":8,"P1":0}).encode())    #放松关节
                elif key == 't':
                    my_arm.serial.write(json.dumps({"T":8,"P1":1}).encode())    #锁住关节
                elif key == 'y':
                    print(COLOR_GREEN + ">>>>>Current Pose :" + str(my_arm.arm_current_pose) + STYLE_RESET) #打印获取当前机械臂各个关节角度值 
                elif key == 'w':
                    write_pose_flag = 1
                    print(COLOR_GREEN + "start to write the arm array!" + STYLE_RESET)
                elif key == 'c':
                     os.system('echo " " > /home/epaicar/talos_ws/src/agrc_base_arm/config/pose_array.yaml')
                     write_count = 0
                elif key == 's':
                    my_arm.serial.write(json.dumps({"T":1,"P1":my_arm.arm_init_pose[0],"P2":my_arm.arm_init_pose[1],"P3":my_arm.arm_init_pose[2],"P4":my_arm.arm_init_pose[3],"P5":my_arm.arm_init_pose[4],\
                            "S1":1000,"S2":1000,"S3":1000,"S4":1000,"S5":1000,\
                            "A1":100,"A2":100,"A3":100,"A4":100,"A5":100}).encode())
                    write_pose_flag = 2
                    write_count += 1
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    except rospy.ROSInterruptException:  
        my_arm.serial.close
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        rospy.loginfo("arm_driver finished.")
