#!/usr/bin/env python

import rospy
from grid_map_msgs.srv import *



if __name__ == "__main__":
    rospy.wait_for_service('/elevation_mapping/get_submap')
    try:
        req = GetGridMap()
        req.frame_id = "map"
        req.position_x = 0
        req.position_y = 0
        req.lenght_x = 2.5
        req.lenght_y = 1
        get_submap = rospy.ServiceProxy(/elevation_mapping/get_submap, GetGridMap)
        submap = get_submap(req)
        
        data = submap.data
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)