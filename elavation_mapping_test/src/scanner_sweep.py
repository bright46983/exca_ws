#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint


rospy.init_node("scanner_sweep")
pub = rospy.Publisher("/dynamixel_workbench/joint_trajectory",JointTrajectory,queue_size=1)

r = rospy.Rate(15)
ang = -90




while not rospy.is_shutdown():
    try:
        if ang == -90:
            t = -1
        elif ang == -175:
            t= +1
        ang = ang+t
        jt = JointTrajectory()
        jt.joint_names = ["scanner_joint"]
        jtp = JointTrajectoryPoint()
        jtp.positions = [(ang*3.14159)/180]
        jtp.velocities = [0.001]
        jtp.time_from_start = rospy.Duration(0.1)
        jt.points.append(jtp)
        jt.header.stamp = rospy.Time.now()    
        pub.publish(jt)
        r.sleep()
        
        
        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    