#! /usr/bin/env python3

import copy
import time
import numpy as np
import math
import rospy
import scipy.io as io
import scipy.interpolate as spy
import pickle
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from cdpr_test02 import CDPR
from transform import euler2quaternion
from Jacobian import getJacobian
from pid import controller

if __name__=="__main__":

    cdpr = CDPR()

    T = 0.25    # control period
    rate = rospy.Rate(1/T)
    
    filename='/home/irono/CDPR_AWS/CDPR_AWS/Codes/ROS/ACTR_Planning/src/actr_planning/script/trajectory_ca.txt'
    with open(filename, 'rb') as f:
        trajectory_log = pickle.load(f)
        data = trajectory_log['x']
    data_len=len(data)
    data_pos=data[0:int(data_len/2)]
    data_velo=data[int(data_len/2):data_len]
    data_pos=data_pos.reshape(int(data_len/18),9)
    data_velo=data_velo.reshape(int(data_len/18),9)
    XRef_org=data_pos[:,0]
    YRef_org=data_pos[:,1]
    ZRef_org=data_pos[:,2]
    UAVtag0_X_org=data_pos[:,3]
    UAVtag0_Y_org=data_pos[:,4]
    UAVtag0_Z_org=data_pos[:,5]
    UAVtag1_X_org=data_pos[:,6]
    UAVtag1_Y_org=data_pos[:,7]
    UAVtag1_Z_org=data_pos[:,8]
    Xtagvelo_org=data_velo[:,0]
    Ytagvelo_org=data_velo[:,1]
    Ztagvelo_org=data_velo[:,2]
    UAVvelo0_X_org=data_velo[:,3]
    UAVvelo0_Y_org=data_velo[:,4]
    UAVvelo0_Z_org=data_velo[:,5]
    UAVvelo1_X_org=data_velo[:,6]
    UAVvelo1_Y_org=data_velo[:,7]
    UAVvelo1_Z_org=data_velo[:,8]
    t=np.linspace(0,0.75*12,13)
    t_3=np.linspace(t[0],t[-1],37)
    pr3_XRefarr=spy.splrep(t,XRef_org,k=3)
    pr3_YRefarr=spy.splrep(t,YRef_org,k=3)
    pr3_ZRefarr=spy.splrep(t,ZRef_org,k=3)
    pr3_UAVtagarr0_X=spy.splrep(t,UAVtag0_X_org,k=3)
    pr3_UAVtagarr0_Y=spy.splrep(t,UAVtag0_Y_org,k=3)
    pr3_UAVtagarr0_Z=spy.splrep(t,UAVtag0_Z_org,k=3)
    pr3_UAVtagarr1_X=spy.splrep(t,UAVtag1_X_org,k=3)
    pr3_UAVtagarr1_Y=spy.splrep(t,UAVtag1_Y_org,k=3)
    pr3_UAVtagarr1_Z=spy.splrep(t,UAVtag1_Z_org,k=3)
    pr3_Xtagvelo=spy.splrep(t,Xtagvelo_org,k=3)
    pr3_Ytagvelo=spy.splrep(t,Ytagvelo_org,k=3)
    pr3_Ztagvelo=spy.splrep(t,Ztagvelo_org,k=3)
    pr3_UAVvelo0_X=spy.splrep(t,UAVvelo0_X_org,k=3)
    pr3_UAVvelo0_Y=spy.splrep(t,UAVvelo0_Y_org,k=3)
    pr3_UAVvelo0_Z=spy.splrep(t,UAVvelo0_Z_org,k=3)
    pr3_UAVvelo1_X=spy.splrep(t,UAVvelo1_X_org,k=3)
    pr3_UAVvelo1_Y=spy.splrep(t,UAVvelo1_Y_org,k=3)
    pr3_UAVvelo1_Z=spy.splrep(t,UAVvelo1_Z_org,k=3)
    XRefArr_it=spy.splev(t_3,pr3_XRefarr)
    YRefArr_it=spy.splev(t_3,pr3_YRefarr)
    ZRefArr_it=spy.splev(t_3,pr3_ZRefarr)
    XRefVeloArr_it=spy.splev(t_3,pr3_Xtagvelo)
    YRefVeloArr_it=spy.splev(t_3,pr3_Ytagvelo)
    ZRefVeloArr_it=spy.splev(t_3,pr3_Ztagvelo)
    UAVRefArr0_X_it=spy.splev(t_3,pr3_UAVtagarr0_X)
    UAVRefArr0_Y_it=spy.splev(t_3,pr3_UAVtagarr0_Y)
    UAVRefArr0_Z_it=spy.splev(t_3,pr3_UAVtagarr0_Z)
    UAVRefArr1_X_it=spy.splev(t_3,pr3_UAVtagarr1_X)
    UAVRefArr1_Y_it=spy.splev(t_3,pr3_UAVtagarr1_Y)
    UAVRefArr1_Z_it=spy.splev(t_3,pr3_UAVtagarr1_Z)
    UAVRefVeloArr0_X_it=spy.splev(t_3,pr3_UAVvelo0_X)
    UAVRefVeloArr0_Y_it=spy.splev(t_3,pr3_UAVvelo0_Y)
    UAVRefVeloArr0_Z_it=spy.splev(t_3,pr3_UAVvelo0_Z)
    UAVRefVeloArr1_X_it=spy.splev(t_3,pr3_UAVvelo1_X)
    UAVRefVeloArr1_Y_it=spy.splev(t_3,pr3_UAVvelo1_Y)
    UAVRefVeloArr1_Z_it=spy.splev(t_3,pr3_UAVvelo1_Z)
    xList,yList,zList=[],[],[]
    uavXList0, uavYList0, uavZList0 = [], [], []
    uavXList1, uavYList1, uavZList1 = [], [], []
    traject_pos = np.zeros((0, 20))
    traject_velo = np.zeros((0, 20))    

    time.sleep(0.1)
    ##################################### pretighten #########################################

    # cnt = 0

    # while not rospy.is_shutdown() and cnt < 35:

    #     print('-----------------------------')
    #     print('            run {}           '.format(cnt))
    #     print('-----------------------------')
    #     start_time = time.time()
        
    #     # referrence pose of moment k  (unit: degree)
    #     uavXRef0 = cdpr.xOff + math.sin(math.pi/4)*math.sin(math.pi/4)*cdpr.uavCableLen0+0.03
    #     uavYRef0 = cdpr.yOff + math.sin(math.pi/4)*math.cos(math.pi/4)*cdpr.uavCableLen0+0.03
    #     uavZRef0 = cdpr.zOff + math.cos(math.pi/4)*cdpr.uavCableLen0+0.05
    #     uavPosRef0 = np.array([uavXRef0, uavYRef0, uavZRef0])
    #     print('uavPosRef0: {}'.format(uavPosRef0))

    #     uavXRef1 = cdpr.xOff - math.sin(math.pi/4)*math.sin(math.pi/4)*cdpr.uavCableLen1+0.03
    #     uavYRef1 = cdpr.yOff - math.sin(math.pi/4)*math.cos(math.pi/4)*cdpr.uavCableLen1+0.03
    #     uavZRef1 = cdpr.zOff + math.cos(math.pi/4)*cdpr.uavCableLen1+0.05
    #     uavPosRef1 = np.array([uavXRef1, uavYRef1, uavZRef1])
    #     print('uavPosRef1: {}'.format(uavPosRef1))

    #     cnt += 1

    #     # uav pose of moment k  (unit: m)
    #     uavPos0 = cdpr.getUAVPos0()
    #     uavPos0 = np.array(uavPos0)
    #     print('uav0Pos: {}'.format(uavPos0))

    #     uavPos1 = cdpr.getUAVPos1()
    #     uavPos1 = np.array(uavPos1)
    #     print('uav1Pos: {}'.format(uavPos1))

    #     # error of moving platform pose of moment k  (unit: m)
    #     uavPosErr0 = uavPosRef0 - uavPos0
    #     uavPosErr1 = uavPosRef1 - uavPos1
    #     # output velocity in task space of moment k  (unit: m/s)
    #     eps = 0.002         # dmax
    #     k1 = 1
    #     k2 = 1
    #     k3 = 1.2
    #     uavVelo0 = eps * np.sign(uavPosErr0) + np.array([k1, k2, k3]) * uavPosErr0       # control law
    #     uavVelo0 = uavVelo0 + np.array([0, 0, 0.06])
    #     uavVelo1 = eps * np.sign(uavPosErr1) + np.array([k1, k2, k3]) * uavPosErr1       # control law
    #     uavVelo1 = uavVelo1 + np.array([0, 0, 0.06])
    #     #uavVelo[0]=-uavVelo[0]
    #     #uavVelo[1]=-uavVelo[1]
    #     print('uavVelo0: {}'.format(uavVelo0))
    #     print('uavVelo1: {}'.format(uavVelo1))
    #     # set uav velocity
    #     cdpr.setUAVVelo0(uavVelo0)
    #     cdpr.setUAVVelo1(uavVelo1)

    #     rate.sleep()
        
    ################################ track ####################################
    cnt=0

    while not rospy.is_shutdown() and cnt < len(t_3):

        print('-----------------------------')
        print('            run {}           '.format(cnt))
        print('-----------------------------')
        start_time = time.time()
        
        runTime = T * cnt
        x, y, z, orientation = cdpr.getMovingPlatformPose()
        pos = np.array([x, y, z])
        posRef=np.array([XRefArr_it[cnt],YRefArr_it[cnt],ZRefArr_it[cnt]])
        print('pos: {}'.format(pos))
        print('posRef:{}',format(posRef))
        xList.append(x)
        yList.append(y)
        zList.append(z)

        veloTask=np.array([Xtagvelo_org[cnt],Ytagvelo_org[cnt],Ztagvelo_org[cnt]])
        posDis=np.linalg.norm(posRef-pos)
        pid=controller(0.1,0,0,(-0.5,0.5))
        pid.set_point(0)
        u=-pid.controller(posDis)
        fbvelo=u*(posRef-pos)/np.linalg.norm(posRef-pos)
        veloTask=veloTask+fbvelo
        print('veloTask: {}'.format(veloTask))

        J=getJacobian(cdpr.getAnchorAPoses(),cdpr.getAnchorBPoses(),pos,orientation)
        veloJoint=np.matmul(J,veloTask.reshape(3,1))
        veloJoint=veloJoint.reshape(4,)
        print('veloJoint: {}'.format(veloJoint))

        uavPos0=cdpr.getUAVPos0()
        uavXList0.append(uavPos0[0])
        uavYList0.append(uavPos0[1])
        uavZList0.append(uavPos0[2])
        uavPos0 = np.array(uavPos0)
        uavPos1 = cdpr.getUAVPos1()
        uavXList1.append(uavPos1[0])
        uavYList1.append(uavPos1[1])
        uavZList1.append(uavPos1[2])
        uavPos1 = np.array(uavPos1)
        print('uavPos: {}'.format(uavPos0))        
        print('uavPos: {}'.format(uavPos1))

        uavVelo0=np.array(UAVRefVeloArr0_X_it[cnt],UAVRefVeloArr0_Y_it[cnt],UAVRefVeloArr0_Z_it[cnt])
        uavVelo1=np.array(UAVRefVeloArr1_X_it[cnt],UAVRefVeloArr1_Y_it[cnt],UAVRefVeloArr1_Z_it[cnt])
        uavTagPos0=np.array(UAVRefArr0_X_it[cnt],UAVRefArr0_Y_it[cnt],UAVRefArr0_Z_it[cnt])
        uavTagPos1=np.array(UAVRefArr1_X_it[cnt],UAVRefArr1_Y_it[cnt],UAVRefArr1_Z_it[cnt])
        print('uavTagPos: {}'.format(uavTagPos0))
        print('uavTagPos: {}'.format(uavTagPos1))
        print('feedforward velo0: {}',format(uavVelo0))
        print('feedforward velo0: {}',format(uavVelo0))

        uavPosDis0 = np.linalg.norm(uavTagPos0 - uavPos0)
        uavPosDis1 = np.linalg.norm(uavTagPos1 - uavPos1)
        pid = controller(2.1, 0, 0, (-0.6, 0.6))
        pid.set_point(0)
        u0 = -pid.controller(uavPosDis0)
        u1 = -pid.controller(uavPosDis1)
        fbVelo0 = u0 * (uavTagPos0 - uavPos0) / np.linalg.norm(uavTagPos0 - uavPos0)
        fbVelo1 = u1 * (uavTagPos1 - uavPos1) / np.linalg.norm(uavTagPos1 - uavPos1)
        uavVelo0=uavVelo0+fbVelo0
        uavVelo1=uavVelo1+fbVelo1
        print('feedback velo0: {}'.format(fbVelo0))
        print('feedback velo1: {}'.format(fbVelo1))

        print(time.time() - start_time)       

        cdpr.setUAVVelo0(uavVelo0)
        print('uavVelo0: {}'.format(uavVelo0))
        cdpr.setUAVVelo1(uavVelo1)
        print('uavVelo1: {}'.format(uavVelo1))
        cdpr.setMotorVelo(-int(veloJoint[0]*60/(0.03*math.pi)*10), -int(veloJoint[1]*60/(0.03*math.pi)*10))
        print('motorVelo :{}'.format([int(veloJoint[0]*60/(0.051*math.pi)*10), int(veloJoint[1]*60/(0.051*math.pi)*10)]))
        print(time.time() - start_time)

        cnt+=1

        pos_data = (np.array([runTime, x, y, z, uavTagPos0[0], uavTagPos0[1], uavTagPos0[2], uavTagPos1[0], uavTagPos1[1], uavTagPos1[2], uavPos0[0], uavPos0[1], uavPos0[2], uavPos1[0], uavPos1[1], uavPos1[2], orientation[0], orientation[1], orientation[2], orientation[3]])).reshape(1, 20)
        velo_data = (np.array([runTime, veloTask[0], veloTask[1], veloTask[2], veloJoint[0], veloJoint[1], veloJoint[2], veloJoint[3], uavVelo0[0], uavVelo0[1], uavVelo0[2], uavVelo1[0], uavVelo1[1], uavVelo1[2], fbVelo0[0], fbVelo0[1], fbVelo0[2], fbVelo1[0], fbVelo1[1], fbVelo1[2]])).reshape(1, 20)
        traject_pos = np.append(traject_pos, pos_data, axis=0)
        traject_velo = np.append(traject_velo, velo_data, axis=0)

        io.savemat('data/traject_pos.mat', {'name': traject_pos})
        io.savemat('data/traject_velo.mat', {'name': traject_velo})
        print('data saved')

        end_time = time.time()
        print(end_time - start_time)

        rate.sleep()
    
    cdpr.setMotorVelo(0,0)
    cdpr.setUAVVelo0([0,0,0])
    cdpr.setUAVVelo1([0,0,0])

    fig=plt.figure(1)
    ax=Axes3D(fig)
    ax.plot3D(XRefArr_it, YRefArr_it, ZRefArr_it)
    ax.plot3D(xList[:], yList[:], zList[:])




        


