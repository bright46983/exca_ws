#!/usr/bin/env python

import rospy
from grid_map_msgs.srv import *



if __name__ == "__main__":
    rospy.wait_for_service('/elevation_mapping/get_submap')
    try:
        frame_id = "map"
        position_x = 0
        position_y = 0
        lenght_x = 2.5
        lenght_y = 1
        get_submap = rospy.ServiceProxy('/elevation_mapping/get_submap', GetGridMap)
        submap = get_submap(frame_id ,position_x,position_y,lenght_x,lenght_y,[])
        
        data = submap.map.data
        print(data)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)