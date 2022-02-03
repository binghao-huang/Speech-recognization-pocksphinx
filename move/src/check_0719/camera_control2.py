#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial
import rospy
import math
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from multiprocessing import Lock
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Transform 
from geometry_msgs.msg import TransformStamped
import time


class Camera_controller(object):
    def __init__(self):
        rospy.init_node('camera_controller')
        rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.controller = rospy.get_param('controller1_port', '/dev/camera_motor')
        self.con = self.port_open(self.controller)
        time.sleep(1)

        # cmd = [0]*7
        # cmd[0] = 0xff
        # cmd[1] = 0x01
        # cmd[2] = 0x00
        # cmd[3] = 0x08
        # cmd[4] = 0x00
        # cmd[5] = 0xff
        # cmd[6] = (cmd[1]+cmd[2]+cmd[3]+cmd[4]+cmd[5])%0x100  # 注意此处计算是10进制结果，发送过去远端是以16进制接受的
        # print cmd
        # self.con.write(cmd)  #上

        self.control_rate = rospy.Rate(200)
        while not rospy.is_shutdown():
            self.control_rate.sleep()
            self.control_loop()

        # cmd = [0]*7
        # cmd[0] = 0xff
        # cmd[1] = 0x01
        # cmd[2] = 0x00
        # cmd[3] = 0x08
        # cmd[4] = 0x00
        # cmd[5] = 0xff
        # cmd[6] = (cmd[1]+cmd[2]+cmd[3]+cmd[4]+cmd[5])%0x100  # 注意此处计算是10进制结果，发送过去远端是以16进制接受的
        # print cmd
        # self.con.write(cmd)  #上
        # time.sleep(3)
        # cmd[3] = 0x10
        # cmd[4] = 0x00
        # cmd[5] = 0xff
        # cmd[6] = (cmd[1]+cmd[2]+cmd[3]+cmd[4]+cmd[5])%0x100
        # self.con.write(cmd)  #下
        # time.sleep(3)
        # cmd[3] = 0x04
        # cmd[4] = 0xff
        # cmd[5] = 0x00
        # cmd[6] = (cmd[1]+cmd[2]+cmd[3]+cmd[4]+cmd[5])%0x100
        # self.con.write(cmd)  #左
        # time.sleep(2)
        # cmd[3] = 0x02
        # cmd[4] = 0xff
        # cmd[5] = 0x00
        # cmd[6] = (cmd[1]+cmd[2]+cmd[3]+cmd[4]+cmd[5])%0x100
        # self.con.write(cmd)  #右
        # time.sleep(2)
        # cmd[3] = 0x00
        # cmd[4] = 0x00
        # cmd[5] = 0x00
        # cmd[6] = (cmd[1]+cmd[2]+cmd[3]+cmd[4]+cmd[5])%0x100
        # self.con.write(cmd)  #停止
        ## position test
        # cmd[0] = 0xff
        # cmd[1] = 0x01
        # cmd[2] = 0x00
        # cmd[3] = 0x51
        # cmd[4] = 0x00
        # cmd[5] = 0x00
        # cmd[6] = ((cmd[1]+cmd[2]+cmd[3]+cmd[4]+cmd[5])%0x100)
        # self.con.write(cmd)
        # rospy.sleep(0.1)
        # count = self.con.inWaiting()
        # tmp_data = self.con.read(count)
        # print(type(tmp_data))
        # print(tmp_data.decode('hex'))
        # print('end!')



    def joy_callback(self, joy_msg):
        self.joy_msg = joy_msg

    def control_loop(self):
        if len(self.joy_msg.axes) > 0:
            if self.joy_msg.buttons[10] == 1:
                if self.joy_msg.axes[3] > 0.5:
                    cmd = [0]*7
                    cmd[0] = 0xff
                    cmd[1] = 0x01
                    cmd[2] = 0x00
                    cmd[3] = 0x08
                    cmd[4] = 0x00
                    cmd[5] = 0x08
                    cmd[6] = (cmd[1]+cmd[2]+cmd[3]+cmd[4]+cmd[5])%0x100  # 注意此处计算是10进制结果，发送过去远端是以16进制接受的
                    self.con.write(cmd)  #上
                    print('up')
                elif self.joy_msg.axes[3] < -0.5:
                    cmd = [0]*7
                    cmd[0] = 0xff
                    cmd[1] = 0x01
                    cmd[2] = 0x00
                    cmd[3] = 0x10
                    cmd[4] = 0x00
                    cmd[5] = 0x08
                    cmd[6] = (cmd[1]+cmd[2]+cmd[3]+cmd[4]+cmd[5])%0x100
                    self.con.write(cmd)  #下
                    print('down')
                elif self.joy_msg.axes[2] < -0.5:
                    cmd = [0]*7
                    cmd[0] = 0xff
                    cmd[1] = 0x01
                    cmd[2] = 0x00
                    cmd[3] = 0x04
                    cmd[4] = 0x08
                    cmd[5] = 0x00
                    cmd[6] = (cmd[1]+cmd[2]+cmd[3]+cmd[4]+cmd[5])%0x100
                    self.con.write(cmd)  #左
                    print('left')
                elif self.joy_msg.axes[2] > 0.5:
                    cmd = [0]*7
                    cmd[0] = 0xff
                    cmd[1] = 0x01
                    cmd[2] = 0x00
                    cmd[3] = 0x02
                    cmd[4] = 0x08
                    cmd[5] = 0x00
                    cmd[6] = (cmd[1]+cmd[2]+cmd[3]+cmd[4]+cmd[5])%0x100
                    self.con.write(cmd)  #右
                    print('right')
                else:
                    cmd = [0]*7
                    cmd[0] = 0xff
                    cmd[1] = 0x01
                    cmd[2] = 0x00
                    cmd[3] = 0x00
                    cmd[4] = 0x00
                    cmd[5] = 0x00
                    cmd[6] = (cmd[1]+cmd[2]+cmd[3]+cmd[4]+cmd[5])%0x100  # 注意此处计算是10进制结果，发送过去远端是以16进制接受的
                    print cmd
                    self.con.write(cmd)  #停止

            else:
                # 手柄控制松开按键的时候发送速度0，有时候会发送一次会无效需要多次发送速度0.
                cmd = [0]*7
                cmd[0] = 0xff
                cmd[1] = 0x01
                cmd[2] = 0x00
                cmd[3] = 0x00
                cmd[4] = 0x00
                cmd[5] = 0x00
                cmd[6] = (cmd[1]+cmd[2]+cmd[3]+cmd[4]+cmd[5])%0x100  # 注意此处计算是10进制结果，发送过去远端是以16进制接受的
                print cmd
                self.con.write(cmd)  #停止

    def port_open(self, port):
        ser = serial.Serial()
        ser.port = port         # 设置端口号
        ser.baudrate = 2400     # 设置波特率
        ser.bytesize = 8        # 设置数据位
        ser.stopbits = 1        # 设置停止位
        ser.parity = "N"        # 设置校验位
        ser.open()              # 打开串口
        return ser

if __name__ == '__main__':
    Camera_controller()


