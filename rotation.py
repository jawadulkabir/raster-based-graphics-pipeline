from scipy.spatial.transform import Rotation
from numpy.linalg import norm
import numpy as np


v = [5, 0, 0]
axis = [0,0,1]
angle = 90

theta = angle*3.14159/180

axis = axis / norm(axis)  # normalize the rotation vector first
rot = Rotation.from_rotvec(theta * axis)
print(rot.as_matrix())
print("\n\n\n")

new_v = rot.apply(v)  
print(new_v)    

print(type(v))

vmat = np.array([v])
vmat = np.transpose(vmat)
print(type(vmat))

mult = np.matmul(rot.as_matrix(), vmat)
print(rot.as_matrix().shape,vmat.shape)
