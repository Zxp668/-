#! /usr/bin/env python
#coding:utf-8
import rospy
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
# 需求：记录初始坐标角度然后oringin_yaw

# 2023/11/22:idea:电机最小值判定，其实当角速度小于某个值的时候电机会做无用功，应消除
#                 同理要是目标值误差较小是否可以舍去，或者用积分累加误差，然后再给个上限
#                 关于任意半径拐弯，有待研究，得去考虑运动学模型

# 初始化一些参数
min_output = 0.0
temp_output = 0.0
max_angle_z = 0.0
min_angle_z = 0.0
speed_x = 0.0
boost_factor = 0.0
Kp_yaw = 0.0
Ki_yaw = 0.0
Kd_yaw = 0.0
want_yaw = 0.0     #控制角度
euler_angles = 0.0 #当前角度
oringin_yaw = 0.0  #初始角度

# pid内部参数
error_yaw = 0.0
previous_error_yaw = 0.0
integral_yaw = 0.0
oringin_flag = 0
stop_flag = 1

pub = rospy.Publisher("/cmd_vel",Twist,queue_size=50)

# 创建pid控制器对象  error0:每次初始化过后都会重置 previous_error_yaw --->kd,
class PID:
    def __init__(self,kp_yaw,kd_yaw,ki_yaw):
    #    初始化一些对象
        self.kp_yaw = kp_yaw
        self.pd_yaw = kd_yaw
        self.ki_yaw = ki_yaw
    # 若目标角度为初始角度，则保持直线运动
    def sum_yaw(self,euler_angles,want_yaw):
        global error_yaw,previous_error_yaw,integral_yaw
        error_yaw = want_yaw - euler_angles 
        integral_yaw = error_yaw + integral_yaw 
        derivative_yaw = error_yaw - previous_error_yaw 
        output_yaw = self.kp_yaw*error_yaw + self.ki_yaw*integral_yaw + self.pd_yaw*derivative_yaw 
        previous_error_yaw = error_yaw
        return output_yaw


def quaternion_to_euler(qw, qx, qy, qz):
    # 计算欧拉角的yaw
    t3 = +2.0 * (qw * qz + qx * qy)
    t4 = +1.0 - 2.0 * (qy**2 + qz**2)
    yaw_z = math.atan2(t3, t4)
    rospy.loginfo('当前角度%.6f',yaw_z)
    return  yaw_z


# 读取imu数据
def ImuCallBack(msg):
    global oringin_yaw,oringin_acc
    global oringin_flag,acc_flag 
    global euler_angles,acceleration_value
    # 将四元素转换为欧拉角
    quaternion =(
        msg.orientation.w,
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z
    )    
    euler_angles = quaternion_to_euler(*quaternion)

    # 获取初始位置（yaw）
    if oringin_flag == 0:
        oringin_yaw = euler_angles
        oringin_flag = oringin_flag + 1

def stop_CallBack(msg):
    global stop_flag
    stop_flag = msg.data
def dotime(event):

    global want_yaw,euler_angles
    global max_angle_z,min_angle_z,temp_output
    want_yaw = oringin_yaw     
    msg = Twist()
    if stop_flag:
    # 进行角速度限制，以免速度太小电机扭不动
        if abs(pid.sum_yaw(euler_angles,want_yaw)) < min_output :
            msg.angular.z = boost_factor*pid.sum_yaw(euler_angles,want_yaw)
            msg.linear.x = speed_x
        
        elif abs(pid.sum_yaw(euler_angles,want_yaw)) >= min_output:
            msg.angular.z = pid.sum_yaw(euler_angles,want_yaw)
            msg.linear.x = speed_x
    else:
        msg.angular.z = 0.0
        msg.linear.x = 0.0
    pub.publish(msg)
    #rospy.loginfo('数值%.2f',pid.sum_yaw(euler_angles,want_yaw))
    

if __name__ == '__main__':
    try:  
        rospy.init_node('ucar_pid_line_node',anonymous = True)
        # 参数服务器加载
        speed_x = rospy.get_param("~speed_x",0.0)
        boost_factor = rospy.get_param("~boost_factor",0.0)
        Kp_yaw = rospy.get_param("~Kp_yaw",0.8)
        Kd_yaw = rospy.get_param("~Kd_yaw",0.0)
        Ki_yaw = rospy.get_param("~Ki_yaw",0.0)
        min_output = rospy.get_param("~min_output",0.0)

        # 改为订阅imu欧拉角
        rospy.Subscriber("/imu",Imu,ImuCallBack)
        rospy.Subscriber("/stop_flag",Int8,stop_CallBack)
        # 定时器 
        pid = PID(Kp_yaw,Kd_yaw,Ki_yaw)
        rospy.loginfo('kp_yaw%.4f,kd_yaw:%.4f,ki_yaw:%.4f',Kp_yaw,Kd_yaw,Ki_yaw)
        rospy.Timer(rospy.Duration(0.01),dotime)
        rospy.spin()
      
    except rospy.ROSInterruptException:
        rospy.loginfo('中断......')
        pass
   
