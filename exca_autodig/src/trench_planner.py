#!/usr/bin/env python
import numpy as np
import rospy
from grid_map_msgs.srv import *
from matplotlib import pyplot as plt
from geometry_msgs.msg import PoseStamped
from exca_autodig.srv import TrenchGoal
from exca_autodig.srv import TrenchGoalRequest
from exca_autodig.srv import TrenchGoalResponse
from astropy.convolution import Gaussian2DKernel, interpolate_replace_nans


global poa
poa = PoseStamped()

def get_submap(data):
    rospy.wait_for_service('/elevation_mapping/get_submap')
    rospy.loginfo("Waiting for submap...")
    try:
        frame_id = "map"
        submap_srv = rospy.ServiceProxy('/elevation_mapping/get_submap', GetGridMap)
        submap = submap_srv(frame_id ,data.position_x,data.position_y,data.length_x,data.length_y,[])
        rospy.loginfo("Recived Submap")
        return submap
        
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def gridmap_plot(req):
    submap = get_submap(req)
    rospy.loginfo("Plotting submap...")
    data = submap.map.data[0]
    y_size = data.layout.dim[0].size
    x_size = data.layout.dim[1].size
    data_array = np.array(data.data).reshape((y_size,x_size))
    kernel = Gaussian2DKernel(5)
    data_array = interpolate_replace_nans(data_array,kernel)

    real_pos_x = (-np.arange(x_size) + x_size/2 +  req.position_x)*0.01
    real_pos_y = (-np.arange(y_size) + y_size/2 +  req.position_y)*0.01
    plt.plot(real_pos_x,data_array[y_size/2,:])
    plt.show() 
    return TrenchGoalResponse(0,0,0,0,0)

def find_poa(req):
    global poa
    #Parameter
    depth_err = 0.02
    depth_max = 0.03
    length_max = 0.20
    length_weigth = 0.02

    submap = get_submap(req)
    data = submap.map.data[0]
    y_size = data.layout.dim[0].size
    x_size = data.layout.dim[1].size
    data_array = np.array(data.data).reshape((y_size,x_size))
    kernel = Gaussian2DKernel(5)
    data_array = interpolate_replace_nans(data_array,kernel)
    

    real_pos_x = (-np.arange(x_size) + x_size/2 +  req.position_x)*0.01
    real_pos_y = (-np.arange(y_size) + y_size/2 +  req.position_y)*0.01

    rospy.loginfo("Finding POA...")
    #Score Array
    depth_score = data_array[y_size/2,:]
    s = depth_score
    s[depth_score <= req.depth+depth_err] = 0
    s[depth_score > req.depth+depth_err] = 1
    length_score = np.arange(x_size,0,-1)*0.01
    total_score =  s*(depth_score + length_score*length_weigth)  

    #POA
    poa_index = np.argmax(total_score)
    poa_x = real_pos_x[poa_index]
    poa_y = real_pos_y[y_size/2]
    poa_z = data_array[y_size/2][poa_index]
    poa.pose.position.x = poa_x 
    poa.pose.position.y = poa_y 
    poa.pose.position.z = poa_z 

    rospy.loginfo("POA: %f , %f , %f",poa_x,poa_y,poa_z)
    #Depth & Length
    penetrate_depth = depth_max if poa_z-req.depth > depth_max else  poa_z-req.depth 
    drag_length = length_max if (real_pos_x[-1]) - poa_x > length_max else (real_pos_x[-1]) - poa_x
    return TrenchGoalResponse(poa_x,poa_y,poa_z,penetrate_depth,drag_length)

if __name__ == "__main__":
    rospy.init_node("trench_planner")
    poa_srv = rospy.Service('find_poa',TrenchGoal,find_poa)
    plot_srv = rospy.Service('gridmap_plot',TrenchGoal,gridmap_plot)

    poa_pub = rospy.Publisher('poa_pose',PoseStamped,queue_size=1)
    poa.header.frame_id = "map"

    r = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        poa.header.stamp = rospy.Time()
        poa_pub.publish(poa)
        r.sleep()

    rospy.spin()
   
    