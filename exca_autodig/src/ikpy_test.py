from  ikpy.chain import Chain
import numpy as np 
from ikpy.link import OriginLink, URDFLink
import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D

orientation_axis = "X"
target_orientation = np.eye(4)
target_pos = np.array([[-1 ,0 ,0 ,0.65],
                       [0 ,0 ,1 ,0],
                       [0 ,1 ,0 ,0],
                       [0 ,0 ,0 ,1]])
#my_chain = Chain.from_urdf_file("/home/tanakrit/exca_ws/src/exca_bringup/urdf/exca_urdf.urdf.xacro")
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
fk = exca_chain.forward_kinematics([0] * 6)
ik =exca_chain.inverse_kinematics(target_pos,initial_position=[0,0,0.35,0.35,0.35,0])
                    
print(ik[2:5])
ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d')

exca_chain.plot(ik, ax)
matplotlib.pyplot.show()