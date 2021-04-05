#!/usr/bin/env python

import rospy
import tf
import time
from geometry_msgs.msg import TransformStamped ,PoseWithCovarianceStamped 
from std_msgs.msg import Float32

global enc
global init_pose
init_pose = PoseWithCovarianceStamped()
enc = 0

def En_Callback(msg):
    global enc
    enc = msg.data

def init_Callback(msg):
    global init_pose
    init_pose = msg

if __name__ == "__main__":
    rospy.init_node("exca_odom")
    sub = rospy.Subscriber("/Encoder",Float32,En_Callback)
    sub_init = rospy.Subscriber("/initialpose",PoseWithCovarianceStamped ,init_Callback)
    tf = tf.TransformBroadcaster()
    x = 0
    enc_old = 0
    DisPerCount = (3.14159*0.0625)/1440

    init_pose.pose.pose.position.x =  0
    init_pose.pose.pose.position.y =  0
    init_pose.pose.pose.position.z =  0

    r = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        dist = (enc - enc_old)*DisPerCount
        enc_old = enc
        x = x + dist
        
        tf.sendTransform((0,x,0),(0,0,0,1),rospy.Time.now(),"base_footprint","odom")
        tf.sendTransform((init_pose.pose.pose.position.x,init_pose.pose.pose.position.y,0),(0,0,-0.707,0.707),rospy.Time.now(),"odom","map")
        
        r.sleep()
