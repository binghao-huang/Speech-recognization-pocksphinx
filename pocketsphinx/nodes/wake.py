#!/usr/bin/env python3
#coding:utf-8
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32

key = "小白"
answer = "请吩咐"

def callback(data):
    if key in data.data:  
        pub = rospy.Publisher('/voice/xf_tts_topic', String, queue_size=10)  
        pub.publish(answer)    
        pub_key = rospy.Publisher('/voice/xf_asr_topic', Int32, queue_size=10)         
        pub_key.publish(1)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('wake', anonymous=True)  
    rospy.Subscriber('recognizer/output', String, callback)    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

