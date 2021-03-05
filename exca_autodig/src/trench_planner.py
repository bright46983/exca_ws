#!/usr/bin/env python
import numpy as np
import rospy
from grid_map_msgs.srv import *
from matplotlib import pyplot as plt
from geometry_msgs.msg import PoseStamped, PolygonStamped, Point32
from exca_autodig.srv import TrenchGoal
from exca_autodig.srv import TrenchGoalRequest
from exca_autodig.srv import TrenchGoalResponse
from astropy.convolution import Gaussian2DKernel, interpolate_replace_nans


global poa
global submap_area
poa = PoseStamped()
submap_area = PolygonStamped()

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

def set_point(submap,real_pos_x,real_pos_y):
    global submap_area
    #
    submap_area.polygon.points = []
    submap_area.polygon.points.append(Point32(x=real_pos_x[0],y=real_pos_y[0]))
    
    submap_area.polygon.points.append(Point32(x=real_pos_x[-1],y=real_pos_y[0]))
    
    submap_area.polygon.points.append(Point32(x=real_pos_x[-1],y=real_pos_y[-1]))
   
    submap_area.polygon.points.append(Point32(x=real_pos_x[0],y=real_pos_y[-1]))


def gridmap_plot(req):
    submap = get_submap(req)
    rospy.loginfo("Plotting submap...")
    data = submap.map.data[0]
    y_size = data.layout.dim[0].size
    x_size = data.layout.dim[1].size
    data_array = np.array(data.data).reshape((y_size,x_size))
    kernel = Gaussian2DKernel(5)
    data_array = interpolate_replace_nans(data_array,kernel)

    real_pos_x = (-np.arange(x_size) + x_size/2 +  req.position_x*100)*0.01
    real_pos_y = (-np.arange(y_size) + y_size/2 +  req.position_y*100)*0.01

    set_point(submap,real_pos_x,real_pos_y)

    plt.plot(real_pos_x,data_array[y_size/2,:])
    plt.ylim((-0.3,0.1))
    plt.show() 
    return TrenchGoalResponse(0,0,0,0,0,True)

def find_poa(req):
    global poa
    global submap_area
    #Parameter
    depth_err = 0.01
    depth_max = 0.025
    length_max = 0.2
    length_weigth = 0.2

    submap = get_submap(req)
    data = submap.map.data[0]
    y_size = data.layout.dim[0].size
    x_size = data.layout.dim[1].size
    data_array = np.array(data.data).reshape((y_size,x_size))
    kernel = Gaussian2DKernel(5)
    data_array = interpolate_replace_nans(data_array,kernel)

    real_pos_x = (-np.arange(x_size) + x_size/2 +  req.position_x*100)*0.01
    real_pos_y = (-np.arange(y_size) + y_size/2 +  req.position_y*100)*0.01
    set_point(submap,real_pos_x,real_pos_y)

    rospy.loginfo("Finding POA...")
    #Score Array
    depth_score = data_array[y_size/2,:]
    s = np.where(depth_score<= -req.depth+depth_err,0,1)
    print(s)
    length_score = np.arange(0,x_size)*0.01
    total_score =  s*(depth_score + length_score*length_weigth)  
    
    #Check finish
    if np.count_nonzero(s) < 10:
        rospy.loginfo("No POA left")
        return TrenchGoalResponse(0,0,0,0,0,False)

    #POA
    poa_index = np.argmax(total_score)
    poa_x = real_pos_x[poa_index]
    poa_y = real_pos_y[y_size/2]
    poa_z = depth_score[poa_index]
    poa.pose.position.x = poa_x 
    poa.pose.position.y = poa_y 
    poa.pose.position.z = poa_z 

    rospy.loginfo("POA: %f , %f , %f",poa_x,poa_y,poa_z)
    #Depth & Length
    penetrate_depth = depth_max if poa_z+req.depth > depth_max else  poa_z+req.depth 
    drag_length = length_max if (real_pos_x[0]) - poa_x > length_max else (real_pos_x[0]) - poa_x
    return TrenchGoalResponse(poa_x,poa_y,poa_z,penetrate_depth,drag_length,True)

if __name__ == "__main__":
    rospy.init_node("trench_planner")
    poa_srv = rospy.Service('find_poa',TrenchGoal,find_poa)
    plot_srv = rospy.Service('gridmap_plot',TrenchGoal,gridmap_plot)

    poly_pub = rospy.Publisher('submap_area',PolygonStamped,queue_size=1)
    poa_pub = rospy.Publisher('poa_pose',PoseStamped,queue_size=1)
    submap_area.header.frame_id = "map"
    poa.header.frame_id = "map"
    

    r = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        poa.header.stamp = rospy.Time()
        poa_pub.publish(poa)
        poly_pub.publish(submap_area)
        r.sleep()

    rospy.spin()
   
    