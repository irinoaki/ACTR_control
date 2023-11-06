# #! /usr/bin/env python3

from fileinput import filename
import math
import scipy.interpolate as spy
import numpy as np
import matplotlib.pyplot as plt
import pickle
from mpl_toolkits.mplot3d import Axes3D
from pid import controller
import rospy
#from cdpr_test01 import CDPR
filename='/home/irono/CDPR_AWS/Codes/ROS/ACTR_Planning/src/actr_planning/script/trajectory_ca.txt'
with open(filename, 'rb') as f:
    trajectory_log = pickle.load(f)
    data = trajectory_log['x']
data_len=len(data)
data_pos=data[0:int(data_len/2)]
data_velo=data[int(data_len/2):data_len]
data_pos=data_pos.reshape(int(data_len/18),9)
data_velo=data_velo.reshape(int(data_len/18),9)
XRefarr=data_pos[:,0]
YRefarr=data_pos[:,1]
ZRefarr=data_pos[:,2]
UAVtagarr0_X=data_pos[:,3]
UAVtagarr0_Y=data_pos[:,4]
UAVtagarr0_Z=data_pos[:,5]
UAVtagarr1_X=data_pos[:,6]
UAVtagarr1_Y=data_pos[:,7]
UAVtagarr1_Z=data_pos[:,8]
XRefarr_velo=data_velo[:,0]
YRefarr_velo=data_velo[:,1]
ZRefarr_velo=data_velo[:,2]
UAVvelo0_X=data_velo[:,3]
UAVvelo0_Y=data_velo[:,4]
UAVvelo0_Z=data_velo[:,5]
UAVvelo1_X=data_velo[:,6]
UAVvelo1_Y=data_velo[:,7]
UAVvelo1_Z=data_velo[:,8]
t=np.linspace(0,1.25*int(data_len/18-1),int(data_len/18))
t_3=np.linspace(t[0],t[-1],int(1.25*int(data_len/18-1)/0.25)+1)
pr3_XRefarr=spy.splrep(t,XRefarr,k=3)
pr3_YRefarr=spy.splrep(t,YRefarr,k=3)
pr3_ZRefarr=spy.splrep(t,ZRefarr,k=3)
pr3_XRefarr_velo=spy.splrep(t,XRefarr_velo,k=3)
pr3_YRefarr_velo=spy.splrep(t,YRefarr_velo,k=3)
pr3_ZRefarr_velo=spy.splrep(t,ZRefarr_velo,k=3)
pr3_UAVtagarr0_X=spy.splrep(t,UAVtagarr0_X,k=3)
pr3_UAVtagarr0_Y=spy.splrep(t,UAVtagarr0_Y,k=3)
pr3_UAVtagarr0_Z=spy.splrep(t,UAVtagarr0_Z,k=3)
pr3_UAVtagarr1_X=spy.splrep(t,UAVtagarr1_X,k=3)
pr3_UAVtagarr1_Y=spy.splrep(t,UAVtagarr1_Y,k=3)
pr3_UAVtagarr1_Z=spy.splrep(t,UAVtagarr1_Z,k=3)
pr3_UAVvelo0_X=spy.splrep(t,UAVvelo0_X,k=3)
pr3_UAVvelo0_Y=spy.splrep(t,UAVvelo0_Y,k=3)
pr3_UAVvelo0_Z=spy.splrep(t,UAVvelo0_Z,k=3)
pr3_UAVvelo1_X=spy.splrep(t,UAVvelo1_X,k=3)
pr3_UAVvelo1_Y=spy.splrep(t,UAVvelo1_Y,k=3)
pr3_UAVvelo1_Z=spy.splrep(t,UAVvelo1_Z,k=3)
XRefarr_it=spy.splev(t_3,pr3_XRefarr)
YRefarr_it=spy.splev(t_3,pr3_YRefarr)
ZRefarr_it=spy.splev(t_3,pr3_ZRefarr)
XRefarr_velo_it=spy.splev(t_3,pr3_XRefarr_velo)
YRefarr_velo_it=spy.splev(t_3,pr3_YRefarr_velo)
ZRefarr_velo_it=spy.splev(t_3,pr3_ZRefarr_velo)
UAVtagarr0_X_it=spy.splev(t_3,pr3_UAVtagarr0_X)
UAVtagarr0_Y_it=spy.splev(t_3,pr3_UAVtagarr0_Y)
UAVtagarr0_Z_it=spy.splev(t_3,pr3_UAVtagarr0_Z)
UAVtagarr1_X_it=spy.splev(t_3,pr3_UAVtagarr1_X)
UAVtagarr1_Y_it=spy.splev(t_3,pr3_UAVtagarr1_Y)
UAVtagarr1_Z_it=spy.splev(t_3,pr3_UAVtagarr1_Z)
UAVvelo0_X_it=spy.splev(t_3,pr3_UAVvelo0_X)
UAVvelo0_Y_it=spy.splev(t_3,pr3_UAVvelo0_Y)
UAVvelo0_Z_it=spy.splev(t_3,pr3_UAVvelo0_Z)
UAVvelo1_X_it=spy.splev(t_3,pr3_UAVvelo1_X)
UAVvelo1_Y_it=spy.splev(t_3,pr3_UAVvelo1_Y)
UAVvelo1_Z_it=spy.splev(t_3,pr3_UAVvelo1_Z)
# a=np.sign(UAVtagarr0_X[0])
# print(a)

# cnt = 0
# while cnt<len(t_3):
#     uavVelo0=np.array([UAVtagarr0_X_it[cnt],UAVtagarr0_Y_it[cnt],UAVtagarr0_Z_it[cnt]])
#     uavVelo1=np.array([UAVtagarr1_X_it[cnt],UAVtagarr1_Y_it[cnt],UAVtagarr1_Z_it[cnt]])
#     print(uavVelo0)
#     print(uavVelo1)
#     cnt+=1    
# fig1=plt.figure(1)
# AX = Axes3D(fig1)
# UAVtaglist0_X=XRefarr_it.tolist()
# UAVtaglist0_Y=YRefarr_it.tolist()
# UAVtaglist0_Z=ZRefarr_it.tolist()
# AX.plot3D(UAVtaglist0_X[:] ,UAVtaglist0_Y[:] ,UAVtaglist0_Z[:])
print(XRefarr_velo)
print(YRefarr_velo)
print(ZRefarr_velo)
# print(YRefarr_velo_it)
# print(ZRefarr_velo_it)
# print(YRefarr_velo)
# print(ZRefarr_velo)
# fig2=plt.figure(2)
# AX = Axes3D(fig2)
# UAVtaglist1_X=UAVtagarr1_X.tolist()
# UAVtaglist1_Y=UAVtagarr1_Y.tolist()
# UAVtaglist1_Z=UAVtagarr1_Z.tolist()
# AX.plot3D(UAVtaglist0_X[:] ,UAVtaglist0_Y[:] ,UAVtaglist0_Z[:])
# AX.plot3D(UAVtaglist1_X[:] ,UAVtaglist1_Y[:] ,UAVtaglist1_Z[:])
# plt.show()
# xlist=[]
# print(type(xlist))
# plt.show()
# plt.scatter(t,UAVvelo0_X)
# plt.scatter(t_3,UAVvelo0_X_it)
# plt.show()
# print(len(UAVvelo0_X_it))

# print(pr3_UAVtagarr0_Y)
# plt.plot(t,UAVvelo0_Z)
# plt.plot(t_3,UAVvelo0_Z_it)
# plt.show()
# data=scipy.io.loadmat(filename)
# data2=np.array(data)
# data2_pos={}
# data2_pos=data2[0:116]
# XReflist,YReflist,ZReflist=[],[],[]
# UAVtaglist0_x,UAVtaglist0_y,UAVtaglist0_z={},{},{}
# UAVtaglist1_x,UAVtaglist1_y,UAVtaglist1_z={},{},{}
# UAVvelo0_x,UAVvelo0_y,UAVvelo0_z=[],[],[]
# UAVvelo0_x,UAVvelo0_y,UAVvelo0_z=[],[],[]
# for i in range(0,12):
#     UAVtaglist0_x.append(data2[4+9*i])
#     UAVtaglist0_y.append(data2[5+9*i])
#     UAVtaglist0_z.append(data2[6+9*i])
#     UAVtaglist1_x.append(data2[7+9*i])
#     UAVtaglist1_y.append(data2[8+9*i])
#     UAVtaglist1_z.append(data2[9+9*i])

# print(UAVtaglist0_x)

# if __name__=="__main__":
#     cdpr=CDPR()
#     N=4
#     rate_N=rospy.Rate(N)
#     uavXList,uavYList,uavZList=[],[],[]
#     uavTagXList, uavTagYList, uavTagZList = [], [], []
#     while True:
#         uavPos = cdpr.getUAVPos()
#         uavXList.append(uavPos[0])
#         uavYList.append(uavPos[1])
#         uavZList.append(uavPos[2])
#         uavPos = np.array(uavPos)
#         print('uavPos: {}'.format(uavPos))

#         # uav target pose of moment k  (unit: m)
#         uavTagPos = np.array([0,0,0])
#         #print('uavTagPos: {}'.format(uavTagPos))
#         # output UAV velo  (unit: m/s)
         
#         uavPosDis = np.linalg.norm(uavTagPos - uavPos)
#         pid = controller(0.15, 0, 0.1, (-0.6, 0.6))
#         pid.set_point(0)
#         u = -pid.controller(uavPosDis)
#         print('pid output: {}'.format(u))
#         fbVelo = u * (uavTagPos - uavPos) / np.linalg.norm(uavTagPos - uavPos)
#         print('feedback velo: {}'.format(fbVelo))
#         rate_N.sleep()

