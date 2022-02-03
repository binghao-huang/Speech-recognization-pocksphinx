#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import thread,time
from sensor_msgs.msg import Joy
from std_msgs.msg import String 
from geometry_msgs.msg import Twist
from tf.transformations import *
from math import pi
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

judge = 'original'
bulldog_vel_pub = rospy.Publisher('/bulldog_velocity_controller/cmd_vel', Twist, queue_size=10)
command_vel_pub_m = rospy.Publisher('/motor_voice', Joy, queue_size = 100, latch=True)
command_pos_pub_m = rospy.Publisher('/motor_control/input/position', JointState, queue_size = 100, latch=True)

def base_command():	
	while True:
		if judge == "左转":
			vel_msg = Twist()
			vel_msg.linear.x = 0.1
			vel_msg.angular.z = 0.1
			bulldog_vel_pub.publish(vel_msg)
			rospy.loginfo("Publsh bulldog velocity command[%0.2f m/s, %0.2f rad/s]",
                                vel_msg.linear.x, vel_msg.angular.z)
			time.sleep(0.1)
		elif judge == "左自转":
			print ('base')
			vel_msg = Twist()
			vel_msg.linear.x = 0
			vel_msg.angular.z = 0.5
			bulldog_vel_pub.publish(vel_msg)
			rospy.loginfo("Publsh bulldog velocity command[%0.2f m/s, %0.2f rad/s]",
                                vel_msg.linear.x, vel_msg.angular.z)
			time.sleep(0.1)
		elif judge == "右转":
			print ('base')
 			vel_msg = Twist()
			vel_msg.linear.x = 0.1
			vel_msg.angular.z = -0.1
			bulldog_vel_pub.publish(vel_msg)
			rospy.loginfo("Publsh bulldog velocity command[%0.2f m/s, %0.2f rad/s]",
                                vel_msg.linear.x, vel_msg.angular.z)
		        time.sleep(0.1)
		elif judge == "右自转":
			print 'base'
 			vel_msg = Twist()
			vel_msg.linear.x = 0
			vel_msg.angular.z = -0.5
			bulldog_vel_pub.publish(vel_msg)
			rospy.loginfo("Publsh bulldog velocity command[%0.2f m/s, %0.2f rad/s]",
                                vel_msg.linear.x, vel_msg.angular.z)
		        time.sleep(0.1)
		elif judge == "前进":
 			vel_msg = Twist()
			vel_msg.linear.x = 0.05
			vel_msg.angular.z = 0
			bulldog_vel_pub.publish(vel_msg)
			rospy.loginfo("Publsh bulldog velocity command[%0.2f m/s, %0.2f rad/s]",
                                vel_msg.linear.x, vel_msg.angular.z)
		        time.sleep(0.1)
		elif judge == "俯身潜行":
 			vel_msg = Twist()
			vel_msg.linear.x = 0.05
			vel_msg.angular.z = 0
			bulldog_vel_pub.publish(vel_msg)
			rospy.loginfo("Publsh bulldog velocity command[%0.2f m/s, %0.2f rad/s]",
                                vel_msg.linear.x, vel_msg.angular.z)
		        time.sleep(0.1)
		elif judge == "高速":
 			vel_msg = Twist()
			vel_msg.linear.x = 0.1
			vel_msg.angular.z = 0
			bulldog_vel_pub.publish(vel_msg)
			rospy.loginfo("Publsh bulldog velocity command[%0.2f m/s, %0.2f rad/s]",
                                vel_msg.linear.x, vel_msg.angular.z)
		        time.sleep(0.1)
		elif judge == "后退":
 			vel_msg = Twist()
			vel_msg.linear.x = -0.1
			vel_msg.angular.z = 0
			bulldog_vel_pub.publish(vel_msg)
			rospy.loginfo("Publsh bulldog velocity command[%0.2f m/s, %0.2f rad/s]",
                                vel_msg.linear.x, vel_msg.angular.z)
		        time.sleep(0.1)


def body_height():
	while not rospy.is_shutdown():
		if judge == "升高":
			pose_vel = Joy()
			pose_vel.axes = [0.0, 0.0, 0.0, 0.8, -0.0, -0.0]
			pose_vel.buttons = [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0]
			command_vel_pub_m.publish(pose_vel)
			time.sleep(0.1)
		elif judge == "停下":
			pose_vel = Joy()
			pose_vel.axes = [0.0, 0.0, 0.0, 0.0, -0.0, -0.0]
			pose_vel.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
			command_vel_pub_m.publish(pose_vel)
			time.sleep(0.1)
   		elif judge == "降低":
			pose_vel = Joy()
			pose_vel.axes = [0.0, 0.0, 0.0, -0.8, -0.0, -0.0]
			pose_vel.buttons = [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0]
			command_vel_pub_m.publish(pose_vel)
			time.sleep(0.1)

def body_bend():	
	global RV2_motor2_joint
	while not rospy.is_shutdown():
		if judge == "弯腰":
			pose_vel = Joy()
			pose_vel.axes = [0.0, 0.8, 0.0, -0.0, -0.0, -0.0]
			pose_vel.buttons = [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0]
			command_vel_pub_m.publish(pose_vel)
			time.sleep(0.1)
		elif judge == "后仰":
			pose_vel = Joy()
			pose_vel.axes = [0.0, -0.8, 0.0, -0.0, -0.0, -0.0]
			pose_vel.buttons = [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0]
			command_vel_pub_m.publish(pose_vel)
			time.sleep(0.1)

def body_rotate():
    while not rospy.is_shutdown():
		if judge == "左扭":
			pose_vel = Joy()
			pose_vel.axes = [0.8, 0.0, 0.0, -0.0, -0.0, -0.0]
			pose_vel.buttons = [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0]
			command_vel_pub_m.publish(pose_vel)
			time.sleep(0.1)
		elif judge == "右扭":
			pose_vel = Joy()
			pose_vel.axes = [-0.8, 0.0, 0.0, -0.0, -0.0, -0.0]
			pose_vel.buttons = [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0]
			command_vel_pub_m.publish(pose_vel)
			time.sleep(0.1)

def cmd(data):
	global judge
	rospy.loginfo("bulldog: I heard %s", data.data)
	judge = data.data   
	if judge == "左十":
		now = rospy.Time.now()
		motor_pos = JointState()
		motor_pos.header = Header()
		motor_pos.header.stamp = now
		motor_pos.header.frame_id = "voice"
		motor_pos.name = ["motor1"]
		motor_pos.position = [RV2_motor1_joint+0.1745]
		print motor_pos
		command_pos_pub_m.publish(motor_pos)
	elif judge == "右十":
		now = rospy.Time.now()
		motor_pos = JointState()
		motor_pos.header = Header()
		motor_pos.header.stamp = now
		motor_pos.header.frame_id = "voice"
		motor_pos.name = ["motor1"]
		motor_pos.position = [RV2_motor1_joint-0.1745]
		print motor_pos
		command_pos_pub_m.publish(motor_pos)
	elif judge == "弯十":
		now = rospy.Time.now()
		motor_pos = JointState()
		motor_pos.header = Header()
		motor_pos.header.stamp = now
		motor_pos.header.frame_id = "voice"
		motor_pos.name = ["motor2"]
		motor_pos.position = [RV2_motor2_joint+0.1745]
		print motor_pos
		command_pos_pub_m.publish(motor_pos)
	elif judge == "仰十":
		now = rospy.Time.now()
		motor_pos = JointState()
		motor_pos.header = Header()
		motor_pos.header.stamp = now
		motor_pos.header.frame_id = "voice"
		motor_pos.name = ["motor2"]
		motor_pos.position = [RV2_motor2_joint-0.1745]
		print motor_pos
		command_pos_pub_m.publish(motor_pos)
	elif judge == "升五":
		now = rospy.Time.now()
		motor_pos = JointState()
		motor_pos.header = Header()
		motor_pos.header.stamp = now
		motor_pos.header.frame_id = "voice"
		motor_pos.name = ["motor3"]
		motor_pos.position = [RV2_motor3_joint+0.05]
		print motor_pos
		command_pos_pub_m.publish(motor_pos)
	elif judge == "降五":
		now = rospy.Time.now()
		motor_pos = JointState()
		motor_pos.header = Header()
		motor_pos.header.stamp = now
		motor_pos.header.frame_id = "voice"
		motor_pos.name = ["motor3"]
		motor_pos.position = [RV2_motor3_joint-0.05]
		print motor_pos
		command_pos_pub_m.publish(motor_pos)

	elif judge == "升一":
		now = rospy.Time.now()
		motor_pos = JointState()
		motor_pos.header = Header()
		motor_pos.header.stamp = now
		motor_pos.header.frame_id = "voice"
		motor_pos.name = ["motor3"]
		motor_pos.position = [RV2_motor3_joint+0.01]
		print motor_pos
		command_pos_pub_m.publish(motor_pos)
	
	elif judge == "降一":
		now = rospy.Time.now()
		motor_pos = JointState()
		motor_pos.header = Header()
		motor_pos.header.stamp = now
		motor_pos.header.frame_id = "voice"
		motor_pos.name = ["motor3"]
		motor_pos.position = [RV2_motor3_joint-0.01]
		print motor_pos
		command_pos_pub_m.publish(motor_pos)
	elif judge == "俯身潜行":
		now = rospy.Time.now()
		motor_pos = JointState()
		motor_pos.header = Header()
		motor_pos.header.stamp = now
		motor_pos.header.frame_id = "voice"
		motor_pos.name = ["motor2"]
		motor_pos.position = [0.8]
		print motor_pos
		command_pos_pub_m.publish(motor_pos)

def RV2_motorjointstate_callback(data):
    # 定义RV2 motor数据全局变量，进行赋值
	global RV2_motor1_joint, RV2_motor2_joint, RV2_motor3_joint
	RV2_motor1_joint = data.position[0]
	RV2_motor2_joint = data.position[1]
	RV2_motor3_joint = data.position[2]
	print RV2_motor1_joint, RV2_motor2_joint, RV2_motor3_joint

def listener():
	global judge, RV2_motor1_joint, RV2_motor2_joint, RV2_motor3_joint
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("/recognizer/output", String, cmd)
	rospy.Subscriber('/joint_states_motor',JointState,RV2_motorjointstate_callback)
	command_vel_pub_m = rospy.Publisher('/motor_voice', Joy, queue_size = 100, latch=True)
	command_pos_pub_m = rospy.Publisher('/motor_control/input/position', JointState, queue_size = 100, latch=True)

	time.sleep(2)
	thread.start_new_thread(base_command, ())
	thread.start_new_thread(body_height, ())
	thread.start_new_thread(body_bend, ())
	thread.start_new_thread(body_rotate, ())
	rospy.spin()

if __name__ == '__main__':
    listener()
