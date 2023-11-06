#! /usr/bin/env python3

import copy
import time
import numpy as np
import math
import rospy
import scipy.io as io
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from cdpr_test02 import CDPR
from transform import euler2quaternion
from Jacobian import getJacobian
from pid import controller



if __name__=="__main__":

    cdpr = CDPR()

    T = 0.2     # control period
    rate = rospy.Rate(1/T)
    
    xRefList, yRefList, zRefList = [], [], []
    xList, yList, zList = [], [], []
    xErrList, yErrList, zErrList = [], [], []

    traject_pos = np.zeros((0, 23))
    traject_velo = np.zeros((0, 32))

    uavXList0, uavYList0, uavZList0 = [], [], []
    uavTagXList0, uavTagYList0, uavTagZList0 = [], [], []
    uavXList1, uavYList1, uavZList1 = [], [], []
    uavTagXList1, uavTagYList1, uavTagZList1 = [], [], []

    lastPos = np.array([cdpr.xOff, cdpr.yOff+0.4, cdpr.zOff])



    time.sleep(0.1)


    ##################################### pretighten #########################################

    cnt = 0

    while not rospy.is_shutdown() and cnt < 35:

        print('-----------------------------')
        print('            run {}           '.format(cnt))
        print('-----------------------------')
        start_time = time.time()
        
        # referrence pose of moment k  (unit: degree)
        uavXRef0 = cdpr.xOff + math.sin(math.pi/4)*math.sin(math.pi/4)*cdpr.uavCableLen0+0.03
        uavYRef0 = cdpr.yOff + math.sin(math.pi/4)*math.cos(math.pi/4)*cdpr.uavCableLen0+0.03
        uavZRef0 = cdpr.zOff + math.cos(math.pi/4)*cdpr.uavCableLen0+0.05
        uavPosRef0 = np.array([uavXRef0, uavYRef0, uavZRef0])
        print('uavPosRef0: {}'.format(uavPosRef0))

        uavXRef1 = cdpr.xOff - math.sin(math.pi/4)*math.sin(math.pi/4)*cdpr.uavCableLen1+0.03
        uavYRef1 = cdpr.yOff - math.sin(math.pi/4)*math.cos(math.pi/4)*cdpr.uavCableLen1+0.03
        uavZRef1 = cdpr.zOff + math.cos(math.pi/4)*cdpr.uavCableLen1+0.05
        uavPosRef1 = np.array([uavXRef1, uavYRef1, uavZRef1])
        print('uavPosRef1: {}'.format(uavPosRef1))

        cnt += 1

        # uav pose of moment k  (unit: m)
        uavPos0 = cdpr.getUAVPos0()
        uavPos0 = np.array(uavPos0)
        print('uav0Pos: {}'.format(uavPos0))

        uavPos1 = cdpr.getUAVPos1()
        uavPos1 = np.array(uavPos1)
        print('uav1Pos: {}'.format(uavPos1))

        # error of moving platform pose of moment k  (unit: m)
        uavPosErr0 = uavPosRef0 - uavPos0
        uavPosErr1 = uavPosRef1 - uavPos1
        # output velocity in task space of moment k  (unit: m/s)
        eps = 0.002         # dmax
        k1 = 1
        k2 = 1
        k3 = 1.2
        uavVelo0 = eps * np.sign(uavPosErr0) + np.array([k1, k2, k3]) * uavPosErr0       # control law
        uavVelo0 = uavVelo0 + np.array([0, 0, 0.06])
        uavVelo1 = eps * np.sign(uavPosErr1) + np.array([k1, k2, k3]) * uavPosErr1       # control law
        uavVelo1 = uavVelo1 + np.array([0, 0, 0.06])
        #uavVelo[0]=-uavVelo[0]
        #uavVelo[1]=-uavVelo[1]
        print('uavVelo0: {}'.format(uavVelo0))
        print('uavVelo1: {}'.format(uavVelo1))
        # set uav velocity
        cdpr.setUAVVelo0(uavVelo0)
        cdpr.setUAVVelo1(uavVelo1)

        rate.sleep()


    ################################ track1 ####################################

    cnt = 0

    while not rospy.is_shutdown() and cnt < 35:

        print('-----------------------------')
        print('            run {}           '.format(cnt))
        print('-----------------------------')
        start_time = time.time()
        
        runTime = T * cnt
        

        if cnt < 30:
            xRef = 0.3 / 30 * (cnt % 30) + cdpr.xOff
            yRef = cdpr.yOff
            zRef = 0.3 / 30 * (cnt % 30) + cdpr.zOff

        posRef = np.array([xRef, yRef, zRef])
        print('posRef: {}'.format(posRef))
        xRefList.append(xRef)
        yRefList.append(yRef)
        zRefList.append(zRef)

        cnt += 1

        if cnt < 30:
            xRefNext = 0.3 / 30 * (cnt % 30) + cdpr.xOff
            yRefNext = cdpr.yOff
            zRefNext = 0.3 / 30 * (cnt % 30) + cdpr.zOff

        posRefNext = np.array([xRefNext, yRefNext, zRefNext])
        print('posRefNext: {}'.format(posRefNext))

        # moving platform pose of moment k  (unit: m)
        x, y, z, orientation = cdpr.getMovingPlatformPose()
        pos = np.array([x, y, z])
        print('pos: {}'.format(pos))
        xList.append(x)
        yList.append(y)
        zList.append(z)
        if cnt == 1:
            lastPos = pos
        print('lastPos: {}'.format(lastPos))


        # error of moving platform pose of moment k  (unit: m)
        posErr = posRef - pos

        # output velocity in task space of moment k  (unit: m/s)
        eps = 0.002         # dmax
        k1 = 1
        k2 = 1
        k3 = 1.2
        veloTask = (posRefNext - posRef) / T + eps * np.sign(posErr) + np.array([k1, k2, k3]) * posErr       # control law
        print('veloTask: {}'.format(veloTask))

        # inverse kinematics
        J = getJacobian(cdpr.getAnchorAPoses(), cdpr.getAnchorBPoses(), pos, orientation)
        veloJoint = np.matmul(J, veloTask.reshape(3, 1))
        veloJoint = veloJoint.reshape(4,)
        print('veloJoint: {}'.format(veloJoint))

        # uav pose of moment k  (unit: m)
        uavPos0 = cdpr.getUAVPos0()
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

        # uav target pose of moment k  (unit: m)
        uavTagPos0 = pos + np.array([math.sin(math.pi/4)*math.sin(math.pi/4)*cdpr.uavCableLen0, math.sin(math.pi/4)*math.cos(math.pi/4)*cdpr.uavCableLen0, math.cos(math.pi/4)*cdpr.uavCableLen0])
        uavTagXList0.append(uavTagPos0[0])
        uavTagYList0.append(uavTagPos0[1])
        uavTagZList0.append(uavTagPos0[2])
        uavTagPos1 = pos + np.array([-math.sin(math.pi/4)*math.sin(math.pi/4)*cdpr.uavCableLen1, -math.sin(math.pi/4)*math.cos(math.pi/4)*cdpr.uavCableLen1, math.cos(math.pi/4)*cdpr.uavCableLen1])
        uavTagXList1.append(uavTagPos1[0])
        uavTagYList1.append(uavTagPos1[1])
        uavTagZList1.append(uavTagPos1[2])
        print('uavTagPos: {}'.format(uavTagPos0))
        print('uavTagPos: {}'.format(uavTagPos1))

        # output UAV velo  (unit: m/s)
        uavPosDis0 = np.linalg.norm(uavTagPos0 - uavPos0)
        uavPosDis1 = np.linalg.norm(uavTagPos1 - uavPos1)
        pid = controller(2.1, 0, 0, (-0.6, 0.6))
        pid.set_point(0)
        u0 = -pid.controller(uavPosDis0)
        u1 = -pid.controller(uavPosDis1)
        print('pid output0: {}'.format(u0))
        print('pid output1: {}'.format(u1))
        fbVelo0 = u0 * (uavTagPos0 - uavPos0) / np.linalg.norm(uavTagPos0 - uavPos0)
        ffVelo0 = 1 * (pos - lastPos)
        jVelo0 = - veloJoint[2] * (uavPos0 - pos) / np.linalg.norm(uavPos0 - pos)
        uavVelo0 = fbVelo0 + 0.1*ffVelo0 + jVelo0 + np.array([0.03, 0.04, 0.05])
        fbVelo1 = u1 * (uavTagPos1 - uavPos1) / np.linalg.norm(uavTagPos1 - uavPos1)
        ffVelo1 = 1 * (pos - lastPos)
        jVelo1 = - veloJoint[3] * (uavPos1 - pos) / np.linalg.norm(uavPos1 - pos)
        uavVelo1 = fbVelo1 + 0.1*ffVelo1 + jVelo1 + np.array([-0.03, -0.04, 0.05])
        print('feedback velo0: {}'.format(fbVelo0))
        print('feedforward velo0: {}'.format(ffVelo0))
        print('joint velo0: {}'.format(jVelo0))

        print('feedback velo1: {}'.format(fbVelo1))
        print('feedforward velo1: {}'.format(ffVelo1))
        print('joint velo1: {}'.format(jVelo1))

        print(time.time() - start_time)

        # set actuator velocity
        cdpr.setUAVVelo0(uavVelo0)
        print('uavVelo: {}'.format(uavVelo0))
        cdpr.setUAVVelo1(uavVelo1)
        print('uavVelo: {}'.format(uavVelo1))
        print(time.time() - start_time)
        cdpr.setMotorVelo(-int(veloJoint[0]*60/(0.03*math.pi)*10), -int(veloJoint[1]*60/(0.03*math.pi)*10))
        print('motorVelo :{}'.format([int(veloJoint[0]*60/(0.051*math.pi)*10), int(veloJoint[1]*60/(0.051*math.pi)*10)]))
        print(time.time() - start_time)

        # update last position of platform
        lastPos = pos

        # save data
        pos_data = (np.array([runTime, xRef, yRef, zRef, x, y, z, uavTagPos0[0], uavTagPos0[1], uavTagPos0[2], uavTagPos1[0], uavTagPos1[1], uavTagPos1[2], uavPos0[0], uavPos0[1], uavPos0[2], uavPos1[0], uavPos1[1], uavPos1[2], orientation[0], orientation[1], orientation[2], orientation[3]])).reshape(1, 23)
        velo_data = (np.array([runTime, veloTask[0], veloTask[1], veloTask[2], veloJoint[0], veloJoint[1], veloJoint[2], veloJoint[3], uavVelo0[0], uavVelo0[1], uavVelo0[2], uavVelo1[0], uavVelo1[1], uavVelo1[2], fbVelo0[0], fbVelo0[1], fbVelo0[2], fbVelo1[0], fbVelo1[1], fbVelo1[2], ffVelo0[0], ffVelo0[1], ffVelo0[2], ffVelo1[0], ffVelo1[1], ffVelo1[2], jVelo0[0], jVelo0[1], jVelo0[2], jVelo1[0], jVelo1[1], jVelo1[2]])).reshape(1, 32)
        traject_pos = np.append(traject_pos, pos_data, axis=0)
        traject_velo = np.append(traject_velo, velo_data, axis=0)

        io.savemat('data/test46sq_pos.mat', {'name': traject_pos})
        io.savemat('data/test46sq_velo.mat', {'name': traject_velo})
        print('data saved')

        end_time = time.time()
        print(end_time - start_time)

        rate.sleep()
            
    cdpr.setMotorVelo(0, 0)
    cdpr.setUAVVelo0([0,0,0])
    cdpr.setUAVVelo1([0,0,0])

    
    
    # plot
    fig1 = plt.figure(1)
    xFig = fig1.add_subplot(4,2,1)
    yFig = fig1.add_subplot(4,2,3)
    zFig = fig1.add_subplot(4,2,5)
    # xErrFig = fig1.add_subplot(4,2,2)
    # yErrFig = fig1.add_subplot(4,2,4)
    # zErrFig = fig1.add_subplot(4,2,6)
    plt.ion()
    
    xFig.plot(np.arange(0, cnt*T, T), xList)
    yFig.plot(np.arange(0, cnt*T, T), yList)
    zFig.plot(np.arange(0, cnt*T, T), zList)
    # xErrFig.plot(np.arange(0, cnt*T, T), xRefList)
    # yErrFig.plot(np.arange(0, cnt*T, T), yErrList)
    # zErrFig.plot(np.arange(0, cnt*T, T), zErrList)

    fig2 = plt.figure(2)
    ax = Axes3D(fig2)
    ax.plot3D(xRefList[:], yRefList[:], zRefList[:])
    ax.plot3D(xList[:], yList[:], zList[:])
    
    plt.ioff()
    plt.show()