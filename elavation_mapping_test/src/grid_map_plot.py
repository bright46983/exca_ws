#!/usr/bin/env python
import numpy as np
import rospy
from grid_map_msgs.srv import *
from matplotlib import pyplot as plt



if __name__ == "__main__":
    rospy.wait_for_service('/elevation_mapping/get_submap')
    try:
        frame_id = "map"
        position_x = 0
        position_y = 0
        lenght_x = 2
        lenght_y = 0.5
        get_submap = rospy.ServiceProxy('/elevation_mapping/get_submap', GetGridMap)
        submap = get_submap(frame_id ,position_x,position_y,lenght_x,lenght_y,[])
        
        data = submap.map.data[0]
        y_size = data.layout.dim[0].size
        x_size = data.layout.dim[1].size
        data_array = np.array(data.data).reshape((y_size,x_size))
        real_pos_x = (-np.arange(x_size) + x_size/2 +  position_x)*0.01
        real_pos_y = (-np.arange(y_size) + y_size/2 +  position_y)*0.01
        plt.plot(real_pos_y,data_array[:,25])
        plt.show()
        
        
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)