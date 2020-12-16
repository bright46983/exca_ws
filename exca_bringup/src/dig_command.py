#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist 
import time

def cmd_setup(cmd,x,y,z,w):
    cmd.linear.x = x
    cmd.linear.y = y
    cmd.linear.z = z
    cmd.angular.x = w
    cmd.angular.y = 0
    cmd.angular.z = 0
    pub.publish(cmd)

def step1(cmd): ##Move Boom to ground
    time.sleep(2)
    cmd_setup(cmd,2,-2,0,0)
    time.sleep(2.3)
    cmd_setup(cmd,0,0,0,0)

def step2(cmd): ##Penetrate
    cmd_setup(cmd,0,0,-1.5,0)
    time.sleep(4)
    cmd_setup(cmd,0,-2,0,0)
    time.sleep(1.5)
    cmd_setup(cmd,0,0,0,0)

def step3(cmd): ##Drag
    time.sleep(2)
    cmd_setup(cmd,-1,-1.1,0,0)
    time.sleep(6)
    cmd_setup(cmd,0,0,0,0)

def step4(cmd): ##Bucket
    time.sleep(2)
    cmd_setup(cmd,0,0.0,-1.8,0)
    time.sleep(3.5)
    cmd_setup(cmd,0,0,0,0)

def step5(cmd): ##Pull boom
    time.sleep(2)
    cmd_setup(cmd,-1.5,0,0,0)
    time.sleep(3)
    cmd_setup(cmd,0,0,0,0)

def step6(cmd): ##Swing
    time.sleep(2)
    cmd_setup(cmd,0,0.0,0,2.5)
    time.sleep(2)
    cmd_setup(cmd,0,0,0,0)

def step7(cmd): ##Dump
    time.sleep(2)
    cmd_setup(cmd,0,1.85,2,0)
    time.sleep(2)
    cmd_setup(cmd,0,0,0,0)

def step8(cmd): ##Return
    time.sleep(2)
    cmd_setup(cmd,0,0,0,-2.8)
    time.sleep(2.5)
    cmd_setup(cmd,-2,2,1,0)
    time.sleep(6)
    cmd_setup(cmd,0,0,0,0)
    
    
if __name__ == "__main__":
    rospy.init_node("dig_command")
    pub = rospy.Publisher("/input_joy/cmd_vel",Twist,queue_size=1)
    cmd = Twist()
    step1(cmd)
    print("Finish Step1")
    step2(cmd)
    print("Finish Step2")
    step3(cmd)
    print("Finish Step3")
    step4(cmd)
    print("Finish Step4")
    step5(cmd)
    print("Finish Step5")
    step6(cmd)
    print("Finish Step6")
    step7(cmd)
    print("Finish Step7")
    step8(cmd)
    print("Finish Step8")
    rospy.spin()








    
    