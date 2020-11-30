#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped , Pose



if __name__ == "__main__":
    rospy.init_node("fake_pose")
    pub = rospy.Publisher("/fake_pos",PoseWithCovarianceStamped,queue_size=1)

    r = rospy.Rate(1)




while not rospy.is_shutdown():
    try:
        pos = PoseWithCovarianceStamped()
        pos.header.frame_id = "map"
        pos.pose.pose.position.x = 0
        pos.pose.pose.position.y = 0
        pos.pose.pose.position.z = 0
        pos.pose.pose.orientation.x = 0
        pos.pose.pose.orientation.y = 0
        pos.pose.pose.orientation.z = 0
        pos.pose.pose.orientation.w = 1
        pub.publish(pos)
        r.sleep()
        
        
        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    
rospy.spin()
