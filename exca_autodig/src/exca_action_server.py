#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist 
import time
import tf
from sensor_msgs.msg import JointState
from exca_autodig.srv import ExcaGoal
from exca_autodig.srv import ExcaGoalRequest
from exca_autodig.srv import ExcaGoalResponse
from  ikpy.chain import Chain
import numpy as np 
from ikpy.link import OriginLink, URDFLink
import numpy as np

global joint_state
joint_state = []



def stateCallback(data):
    global joint_state
    joint_state = np.array(data.position)
    for i in range(0,4):
        joint_state[i] = 0 if joint_state[i] > 5.5 else joint_state[i]
    #print(joint_state)

def cmd_setup(cmd,x,y,z,w):
    cmd.linear.x = x #Boom
    cmd.linear.y = y #Arm
    cmd.linear.z = z #Buk
    cmd.angular.x = w #Swing
    cmd.angular.y = 0
    cmd.angular.z = 0
    pub.publish(cmd)

def GotoPoint(req): 
    global joint_state
    cmd = Twist()
    time_limit = 5
    speed = 0.8

    goal = np.array([[-1 ,0 ,0 ,0],
                       [0 ,0 ,1 ,0],
                       [0 ,1 ,0 ,0],
                       [0 ,0 ,0 ,1]])
    goal[0][3] = req.goal[0]
    goal[1][3] = req.goal[1]
    goal[2][3] = req.goal[2]
    rospy.loginfo('Go to point: %f',req.goal)

    ik = exca_chain.inverse_kinematics(goal,initial_position=[0,0,0.35,0.35,0.35,0])
    goal_diff = ik[2:5] - joint_state[1:4]
    rospy.loginfo('Go to point: '+ req.goal)

    boom_cw = 1 if goal_diff[0] > 0 else -1
    arm_cw = -1 if goal_diff[1] > 0 else 1
    buk_cw = -1 if goal_diff[2] > 0 else 1
    time.sleep(0.1)
    cmd.linear.x = speed*boom_cw #Boom
    cmd.linear.y = speed*arm_cw #Arm
    cmd.linear.z = speed*buk_cw #Buk
    pub.publish(cmd)
    print(cmd)

    start_time = time.time()
    check = 0
    while check < 3:
        if joint_state[1]*boom_cw > ik[2]*boom_cw:
            cmd.linear.x = 0
            check = check + 1
            ik[2] = ik[2]-99999*boom_cw
            rospy.loginfo('Boom Reached')
            pub.publish(cmd)

        if joint_state[2]*-arm_cw > ik[3]*-arm_cw:
            cmd.linear.y = 0
            check = check + 1
            ik[3] = ik[3]-99999*-arm_cw
            rospy.loginfo('Arm Reached')
            pub.publish(cmd)

        if joint_state[3]*-buk_cw > ik[4]*-buk_cw:
            cmd.linear.z = 0
            check = check + 1
            ik[4] = ik[4]-99999*-buk_cw
            rospy.loginfo('Bucket Reached')
            pub.publish(cmd)
        
        time_used = time.time() - start_time 
        if (time_used > time_limit):
             cmd_setup(cmd,0,0,0,0)
             rospy.loginfo('Time Out')
             return ExcaGoalResponse(False)

    cmd_setup(cmd,0,0,0,0)
    rospy.loginfo('Point Reached')
    return ExcaGoalResponse(True)

    

    
    
    

def Penetrate(req): ##Penetrate
    cmd = Twist()
    time_limit = 5
    depth_goal =  req.penetrate_goal #req.depth_goal#From srv
    
    tf.waitForTransform("/base_footprint","/bucket_tip", rospy.Time(), rospy.Duration(1.0))
    t = tf.getLatestCommonTime("/base_footprint","/bucket_tip")
    position, quaternion = tf.lookupTransform("/base_footprint","/bucket_tip", t)
    depth = position[2]#Lookup transform
    
    time.sleep(1)
    cmd_setup(cmd,0,-1.8,0,0)

    start_time = time.time()
    while depth > depth_goal:
         tf.waitForTransform("/base_footprint","/bucket_tip", rospy.Time(), rospy.Duration(0.5))
         t = tf.getLatestCommonTime("/base_footprint","/bucket_tip")
         position, quaternion = tf.lookupTransform("/base_footprint","/bucket_tip", t)
         depth = position[2]#Lookup transform

         rospy.loginfo('Current Depth: %f',depth)
         
         time_used = time.time() - start_time 
         if (time_used > time_limit):
             cmd_setup(cmd,0,0,0,0)
             return ExcaGoalResponse(False)
    cmd_setup(cmd,0,0,0,0)
    return ExcaGoalResponse(True)
        
    
    

def Drag(req): ##Drag
    cmd = Twist()
    time_limit = 10
    length_goal =  req.drag_goal #req.depth_goal#From srv
    
    
    tf.waitForTransform("/base_footprint","/bucket_tip", rospy.Time(), rospy.Duration(1.0))
    t = tf.getLatestCommonTime("/base_footprint","/bucket_tip")
    position, quaternion = tf.lookupTransform("/base_footprint","/bucket_tip", t)
    length = position[0]#Lookup transform
    
    time.sleep(1)
    cmd_setup(cmd,-1,-1.1,0,0) 

    start_time = time.time()
    while length > length_goal:
         tf.waitForTransform("/base_footprint","/bucket_tip", rospy.Time(), rospy.Duration(0.5))
         t = tf.getLatestCommonTime("/base_footprint","/bucket_tip")
         position, quaternion = tf.lookupTransform("/base_footprint","/bucket_tip", t)
         length = position[0]#Lookup transform

         rospy.loginfo('Current Length: %f',length)

         time_used = time.time() - start_time 
         if (time_used > time_limit):
             cmd_setup(cmd,0,0,0,0)
             return ExcaGoalResponse(False)
    cmd_setup(cmd,0,0,0,0)
    return ExcaGoalResponse(True)

def Closing(req): ##Bucket
    cmd = Twist()
    
    time.sleep(1)
    cmd_setup(cmd,0,0.0,-1.8,0)
    rospy.loginfo('Closing the bucket')
    time.sleep(3.5)
    cmd_setup(cmd,-1.5,0,0,0)
    time.sleep(3)
    cmd_setup(cmd,0,0,0,0)
    return ExcaGoalResponse(True)


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

def Stop(req): ##Return
    cmd = Twist()
    cmd_setup(cmd,0,0,0,0)
    return ExcaGoalResponse(True)

    
if __name__ == "__main__":
    rospy.init_node("dig_command")
    exca_chain = Chain(name='base_footprint', links=[
    OriginLink(),
    URDFLink(
      name="swing_frame",
      translation_vector=[0, 0, 0.163],
      orientation=[0, 0, -1.578],
      rotation=[0, 0, 1],
    ),
    URDFLink(
      name="boom_frame",
      translation_vector=[0, 0, 0],
      orientation=[1.5708, 0, -1.5708],
      rotation=[0, 0, 1],
    ),
    URDFLink(
      name="arm_frame",
      translation_vector=[-0.205 ,0.424 ,0.0],
      orientation=[0, 0, 0],
      rotation=[0, 0, 1],
    ),
    URDFLink(
      name="bucket_frame",
      translation_vector=[-0.192 ,0.14 ,0.0],
      orientation=[0, 0, 0],
      rotation=[0, 0, 1],
    ),
    URDFLink(
      name="bucket_tip",
      translation_vector=[-0.055 ,0.153 ,0],
      orientation=[0, 0, 0],
      rotation=[0, 0, 0],
    ),
])
    #fk = exca_chain.forward_kinematics([0] * 6)
    

    tf = tf.TransformListener()
    penetrate_srv = rospy.Service('penetrate',ExcaGoal,Penetrate)
    drag_srv = rospy.Service('drag',ExcaGoal,Drag)
    stop_srv = rospy.Service('stop',ExcaGoal,Stop)
    point_srv = rospy.Service('gotopoint',ExcaGoal,GotoPoint)
    close_srv = rospy.Service('close',ExcaGoal,Closing)

    pub = rospy.Publisher("/input_joy/cmd_vel",Twist,queue_size=1)
    sub = rospy.Subscriber("/joint_states",JointState,stateCallback)
    

    rospy.spin()

