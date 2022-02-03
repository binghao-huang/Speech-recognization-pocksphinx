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


class Motor_controller(object):
    def __init__(self):
        rospy.init_node('motor_controller')
        rospy.Subscriber('/motor_super', Joy, self.joy_callback)
        rospy.Subscriber('/joy', Joy, self.joy_callback)
        rospy.Subscriber('/motor_control/input/position', JointState ,
                        self.position_callback)
        rospy.Subscriber('/motor_control/input/velocity', JointState ,
                        self.velocity_callback)
        self.controller_1 = rospy.get_param('~controller1_port', '/dev/motor1')
        self.controller_2 = rospy.get_param('~controller2_port', '/dev/motor2')
        self.controller_3 = rospy.get_param('~controller3_port', '/dev/motor3')
        self.joint_pub = rospy.Publisher('/joint_states', JointState,
                                         queue_size=1)
        self.joint_pub_motor = rospy.Publisher('/joint_states_motor', JointState,
                                         queue_size=1)

        #self.tf_pub = rospy.Publisher('/tf_motor', TFMessage, queue_size=1)
        self.con1 = self.port_open(self.controller_1)
        self.con2 = self.port_open(self.controller_2)
        self.con3 = self.port_open(self.controller_3)
        self.encoder1_count = 0
        self.encoder2_count = 0
        self.encoder3_count = 0
        self.send_zero_count = 0  
        self.encoder1_cmd = None
        self.encoder2_cmd = None
        self.encoder3_cmd = None
        self.writing = False
        self.position1_last = 0
        self.position2_last = 0
        self.position3_last = 0
        # self.motor1_init = False
        # self.motor2_init = False
        # self.motor3_init = False
        self.joint_state = JointState()
        self.joint_state.name = ["motor1_joint", "motor2_joint", "motor3_joint"]
        # self.motor_tf = TFMessage()
        # self.motor_tf.transforms = [TransformStamped(), TransformStamped()
        #                             ,TransformStamped()]
        # self.motor_tf.transforms[0].header.frame_id = 'base_link'
        # self.motor_tf.transforms[1].header.frame_id = 'motor1_link'
        # self.motor_tf.transforms[2].header.frame_id = 'motor2_link'
        # self.motor_tf.transforms[0].child_frame_id = 'motor1_link'
        # self.motor_tf.transforms[1].child_frame_id = 'motor2_link'
        # self.motor_tf.transforms[2].child_frame_id = 'motor3_link'
        # self.motor_tf.transforms[0].transform.translation.x = 0.21
        # self.motor_tf.transforms[0].transform.translation.z = 0.188
        # self.motor_tf.transforms[1].transform.translation.z = 0.212
        # self.motor_tf.transforms[2].transform.translation.z = 0.2365
        # self.motor_tf.transforms[0].transform.rotation.w = 1
        # self.motor_tf.transforms[1].transform.rotation.w = 1
        # self.motor_tf.transforms[2].transform.rotation.w = 1
        self.joy_msg = Joy()
        self.mutex = Lock()
        rospy.Timer(rospy.Duration(0.1), self.update_status)
        while(self.encoder1_count <= 0 and self.encoder3_count <= 0):
            rospy.loginfo("Waiting for encoder data!") 
            rospy.sleep(0.3)
        rospy.sleep(1.0)
        self.init(self.con1, self.encoder1_cmd)
        self.init(self.con2, self.encoder2_cmd)
        self.init(self.con3, self.encoder3_cmd)
        self.control_rate = rospy.Rate(200)
        while not rospy.is_shutdown():
            self.control_rate.sleep()
            self.control_loop()
    def joy_callback(self, joy_msg):
        self.joy_msg = joy_msg
        #self.control_loop()

    def position_callback(self, position_msg):
        if len(position_msg.name) > 0:
            self.writing = True
            for i in range(len(position_msg.name)):
                if position_msg.name[i] == "motor1":
                    position1 = int(387350 + position_msg.position[i]*137358.156)
                    if position1 > 774700:
                        position1 = 774700
                    if position1 < 0:
                        position1 = 0
                    position1 = self.value_to_cammand(position1, 'pose')
                    for i in range(2):
                        self.send_command(self.con1, '1000', '0000')
                        rospy.sleep(0.2)
                        self.send_command(self.con1, '12B2', position1)
                        rospy.sleep(0.2)
                elif position_msg.name[i] == "motor2":
                    position2 = int(2233000 - position_msg.position[i]*1969135.8)
                    if position2 < 0:
                        position2 = 0
                    position2 = self.value_to_cammand(position2,'pose')
                    for i in range(2):
                        self.send_command(self.con2, '1000', '0000')
                        rospy.sleep(0.2)
                        self.send_command(self.con2, '12B2',position2)
                        rospy.sleep(0.2)
                elif position_msg.name[i] == "motor3":
                    position3 = int(3310000 - position_msg.position[i]*13240000)
                    if position3 > 3310000:
                        position3 = 3310000
                    if position3 < 0:
                        position3 =0
                    position3 = self.value_to_cammand(position3, 'pose')
                    for i in range(2):
                        self.send_command(self.con3, '1000', '0000')
                        rospy.sleep(0.2)
                        self.send_command(self.con3, '12B2', position3)
                        rospy.sleep(0.2)
                rospy.sleep(0.1)
            self.writing = False

    def velocity_callback(self, velocity_msg):
        if len(velocity_msg.name) > 0:
            self.writing = True
            for i in range(len(velocity_msg.name)):
                if velocity_msg.name[i] == "motor1":
                    velocity1 = int(velocity_msg.velocity[i]*80)
                    if velocity1 > 80:
                        velocity1 = 80
                    if velocity1 < -80:
                        velocity1 = -80
                    velocity1 = self.value_to_cammand(velocity1)
                    for i in range(2):
                        self.send_command(self.con1, '1000', '0001')
                        #rospy.sleep(0.2)
                        self.send_command(self.con1, '12B0', velocity1)
                        #rospy.sleep(0.2)
                elif velocity_msg.name[i] == "motor2":
                    velocity2 = int(velocity_msg.velocity[i]* -100)
                    if velocity2 < -100:
                        velocity2 = -100
                    if velocity2 > 100:
                        velocity2 = 100
                    velocity2 = self.value_to_cammand(velocity2)
                    for i in range(2):
                        self.send_command(self.con2, '1000', '0001')
                        #rospy.sleep(0.2)
                        self.send_command(self.con2, '12B0',velocity2)
                        #rospy.sleep(0.2)
                elif velocity_msg.name[i] == "motor3":
                    velocity3 = int(velocity_msg.velocity[i]* -200)
                    if velocity3 > 400:
                        velocity3 = 400
                    if velocity3 < -400:
                        velocity3 = -400
                    velocity3 = self.value_to_cammand(velocity3)
                    for i in range(2):
                        self.send_command(self.con3, '1000', '0001')
                        #rospy.sleep(0.2)
                        self.send_command(self.con3, '12B0', velocity3)
                        #rospy.sleep(0.2)
                rospy.sleep(0.1)
            self.writing = False

    def control_loop(self):
        if len(self.joy_msg.axes) > 0:
            if self.joy_msg.buttons[4] == 1:
                value1 = int(80 * self.joy_msg.axes[0])
                value2 = int (-100 * self.joy_msg.axes[1])
                value3 = int(-400 * self.joy_msg.axes[3])
                command1 = self.value_to_cammand(value1)
                command2 = self.value_to_cammand(value2)
                command3 = self.value_to_cammand(value3)
                self.send_command(self.con1, '1000', '0001')
                self.send_command(self.con1, '12B0', command1)
                self.send_command(self.con2, '1000', '0001')
                self.send_command(self.con2, '12B0', command2)
                self.send_command(self.con3, '1000', '0001')
                self.send_command(self.con3, '12B0', command3)
                self.send_zero_count = 10
            else:
                # 手柄控制松开按键的时候发送速度0，有时候会发送一次会无效需要多次发送速度0.
                if self.send_zero_count > 0:
                    stop = self.value_to_cammand(0)
                    self.send_command(self.con1, '12B0', stop)
                    self.send_command(self.con2, '12B0', stop)
                    self.send_command(self.con3, '12B0', stop)
                    self.send_zero_count -= 1

    def port_open(self, port):
        ser = serial.Serial()
        ser.port = port         # 设置端口号
        ser.baudrate = 9600     # 设置波特率
        ser.bytesize = 7        # 设置数据位
        ser.stopbits = 1        # 设置停止位
        ser.parity = "E"        # 设置校验位
        ser.open()              # 打开串口
        return ser

    # 将手柄控制的速度值或电机位置值转化成字符串
    def value_to_cammand(slef, value, type='vel'):
        if value >= 0:
            cmd = hex(abs(value))
        else:
            cmd = bin(abs(value))
            cmd_str = "".join((cmd).split("0b"))
            inver = [1]*16
            inver = int(''.join([str(x) for x in inver]), 2) ^ abs(value)
            tmp = hex(inver + 1)
            cmd = "".join((tmp).split("0x"))
        cmd = "".join((cmd).split("0x")).upper()
        if type == 'vel':
            cammand = [0] * 4
            l = len(cmd) - 3
        else:
            cammand = [0] * 8
            l = len(cmd) - 7
        for i in range(len(cmd), 0, -1):
            cammand[i-l] = cmd[i-1]
        cammand = ''.join(str(i) for i in cammand)
        if len(cammand) == 8 and value >0:
            cammand = cammand[4:] + cammand[:4]
        return cammand

    # 写入字符串
    def send_command(self, port, mode, command):
        self.mutex.acquire()
        try:
            if mode == '12B2':       # 位置（位置环控制）
                cmd = ":011012B2000202"
            elif mode == '10CB':     # 最大转速限制 rpm
                cmd = ":010610CB"
            elif mode == '1000':     # 控制模式切换
                cmd = ":01061000"
            elif mode == '12B0':     # 速度（速度环）
                cmd = ":010612B0"
            elif mode == '12A0':     # 读取位置
                cmd = ':010312A0'
            cmd = cmd + command
            checkcmd = cmd[1:]
            checksum = self.get_checksum(checkcmd)
            cmd = cmd + checksum + '\r\n'
            # print cmd
            port.write(cmd)
        except:
            rospy.logwarn('command write timeout')
        self.mutex.release()

    # 初始化,电机使能
    def init(self, port, cmd):
        try:
            self.send_command(port, '12B2', cmd)
            rospy.sleep(0.1)
            init_cmd = ":01061012000ACD\r\n"
            port.write(init_cmd)
            rospy.loginfo('%s init successful'%port.port)
            rospy.sleep(1)
            if port.port == '/dev/motor1':
                self.send_command(port, '10CB', '2710')
            elif port.port == '/dev/motor3':
                self.send_command(port, '10CB', '2710')

	    # if port.port == '/dev/motor1':
            #    self.motor1_init = True
            # elif port.port == '/dev/motor2':
            #    self.motor2_init = True
            # elif port.port == '/dev/motor3':
            #    self.motor3_init = True
        except:
            rospy.logerr('%s Init Error!'%port.port)

    # 计算校验和
    def get_checksum(self, checkcmd):
        checksum = ['0']*2            # 初始化校验和为00
        # 将校验和字符串加上0x转义字符，变成系统识别的16进制
        sumcmdhex = checkcmd.decode('hex')
        tmp = 0
        # 求和
        for ch in sumcmdhex:
            tmp += ord(ch)
        #校验和：(~tmp) & 0xff + 0x01 = 0x1d
        sum_bin = bin(tmp)  # 将和转换成二进制
        # 去掉二进制转义字符，得到二进制字符串
        # sum_bin_str = "".join((sum_bin).split("0b"))
        inver = [1]*8      # 初始化一个8位全为1的数组
        # 按位取反符不可用，使用异或得到按位取反结果
        inver = int(''.join([str(x) for x in inver]), 2) ^ tmp
        tmp = hex((inver & 0xFF) + 1)
        tmp = "".join((tmp).split("0x"))
        if len(tmp) == 2:
            checksum[0] = tmp[0]
            checksum[1] = tmp[1]
        else:
            checksum[1] = tmp[0]
        checksum = ''.join(checksum)
        return checksum.upper()

    # 读取编码器数值
    def get_value(self,port):
        try:
            cmd = ":010312A0000248\r\n"
            port.write(cmd)
            rospy.sleep(0.1)
            count = port.inWaiting()
            tmp_data = port.read(count)
            if tmp_data != '' and tmp_data[:7] == ':010304':
                encoder_count_str = tmp_data[11:15] + tmp_data[7:11]
                encoder_count_hex = encoder_count_str.decode('hex')
                i = 3 
                encoder_count = 0
                for ch in encoder_count_hex:
                    encoder_count += ord(ch) * pow(16, i * 2)
                    i -= 1
                print 'encoder_count_str: ' ,encoder_count
                encoder_count_str = encoder_count_str[4:] + encoder_count_str[:4]
                return encoder_count, encoder_count_str
            else:
                print('fail!')
                return -1, '0'                
        except:
            print('Get encoder data fail!')
            return -1, '0'

    # 拿到编码器的数据,发布tf和joint_state
    def update_status(self, even):
        try:
            if self.writing == False:
                self.encoder1_count,self.encoder1_cmd = self.get_value(self.con1)
                self.encoder2_count,self.encoder2_cmd = self.get_value(self.con2)
                self.encoder3_count,self.encoder3_cmd = self.get_value(self.con3)
            '''
            motor1 编码器计数范围:[0 ~ 387350] 对应的运动弧度范围:[2.82 ~ -2.82]
            motor2 编码器计数范围:[0 ~ 223300 ~ max] 对应的运动弧度范围:[1.134 ~ 0 ~ min]
            motor3 编码器基数范围:[0 ~ 3310000] 对应的运动距离范围(mm):[0.25 ~ 0]
            '''
            position1_new = (self.encoder1_count - 387350)/137358.156
            position2_new = (2233000 - self.encoder2_count)/1969135.8
            position3_new = (3310000 - self.encoder3_count)/13240000.0
            if (abs(position1_new - self.position1_last) < 0.5 and 
                abs(position2_new - self.position2_last) < 0.5 and
                abs(position3_new - self.position3_last) < 0.03 and
                self.encoder1_count != -1):
                # position1_ori = tf.transformations.quaternion_from_euler(0, 0, position1_new)
                # position2_ori = tf.transformations.quaternion_from_euler(0,position2_new,0)
                # self.motor_tf.transforms[0].transform.rotation.x = position1_ori[0]
                # self.motor_tf.transforms[0].transform.rotation.y = position1_ori[1]
                # self.motor_tf.transforms[0].transform.rotation.z = position1_ori[2]
                # self.motor_tf.transforms[0].transform.rotation.w = position1_ori[3]
                # self.motor_tf.transforms[1].transform.rotation.x = position2_ori[0]
                # self.motor_tf.transforms[1].transform.rotation.y = position2_ori[1]
                # self.motor_tf.transforms[1].transform.rotation.z = position2_ori[2]
                # self.motor_tf.transforms[1].transform.rotation.w = position2_ori[3]
                # self.motor_tf.transforms[2].transform.translation.z = position3_new + 0.2365
                # self.tf_pub.publish(self.motor_tf)
                self.joint_state.position = [position1_new, position2_new,
                                            position3_new]
                self.joint_state.header.stamp = rospy.Time.now()
                self.joint_pub.publish(self.joint_state)
                self.joint_pub_motor.publish(self.joint_state)
            self.position1_last = position1_new
            self.position2_last = position2_new
            self.position3_last = position3_new
        except:
            pass


if __name__ == '__main__':
    Motor_controller()

