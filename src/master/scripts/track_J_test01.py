#! /usr/bin/env python3

import copy
import time
import numpy as np
import math
import rospy
import scipy.io as io
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from cdpr_test01 import CDPR
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

    traject_pos = np.zeros((0, 17))
    traject_velo = np.zeros((0, 19))

    uavXList, uavYList, uavZList = [], [], []
    uavTagXList, uavTagYList, uavTagZList = [], [], []


    lastPos = np.array([cdpr.xOff, cdpr.yOff+0.4, cdpr.zOff])



    time.sleep(0.1)


    ##################################### pretighten #########################################

    cnt = 0
    while not rospy.is_shutdown() and cnt < 25:

        print('-----------------------------')
        print('            run {}           '.format(cnt))
        print('-----------------------------')
        start_time = time.time()
        
        # referrence pose of moment k  (unit: degree)
        uavXRef = cdpr.xOff
        uavYRef = cdpr.yOff
        uavZRef = cdpr.zOff+cdpr.uavCableLen+0.05
        uavPosRef = np.array([uavXRef, uavYRef, uavZRef])
        print('uavPosRef: {}'.format(uavPosRef))


        cnt += 1

        # uav pose of moment k  (unit: m)
        uavPos = cdpr.getUAVPos()
        uavPos = np.array(uavPos)
        print('uavPos: {}'.format(uavPos))


        # error of moving platform pose of moment k  (unit: m)
        uavPosErr = uavPosRef - uavPos
        # output velocity in task space of moment k  (unit: m/s)
        eps = 0.002         # dmax
        k1 = 1
        k2 = 1
        k3 = 1.2
        uavVelo = eps * np.sign(uavPosErr) + np.array([k1, k2, k3]) * uavPosErr       # control law
        uavVelo = uavVelo + np.array([0, 0, 0.05])
        #uavVelo[0]=-uavVelo[0]
        #uavVelo[1]=-uavVelo[1]
        print('uavVelo: {}'.format(uavVelo))
        # set uav velocity
        cdpr.setUAVVelo(uavVelo)

        rate.sleep()


    ################################ track ####################################

    cnt = 0

    while not rospy.is_shutdown() and cnt < 30:

        print('-----------------------------')
        print('            run {}           '.format(cnt))
        print('-----------------------------')
        start_time = time.time()
        
        runTime = T * cnt
        
        # referrence pose of moment k  (unit: degree)
        # circle
        # xRef = 0.3 * math.sin(2*math.pi/20 * T * cnt) + cdpr.xOff
        # yRef = 0.3 * math.cos(2*math.pi/20 * T * cnt) + cdpr.yOff
        # zRef = 0.05 * math.sin(2*math.pi/10 * T * cnt) + cdpr.zOff
        # fixed point
        # xRef = cdpr.xOff
        # yRef = cdpr.yOff + 0.3
        # zRef = cdpr.zOff
        # square
        if cnt < 30:
            xRef = cdpr.xOff
            yRef = cdpr.yOff
            zRef = 0.2 / 30 * (cnt % 30) + cdpr.zOff
        # elif cnt < 60:
        #     xRef = 0.2 + cdpr.xOff
        #     yRef = 0.2 - 0.4 / 30 * (cnt % 30) + cdpr.yOff
        #     zRef = 0.05 - 0.1 / 30 * (cnt % 30) + cdpr.zOff
        # elif cnt < 90:
        #     xRef = 0.2 - 0.4 / 30 * (cnt % 30) + cdpr.xOff
        #     yRef = -0.2 + cdpr.yOff
        #     zRef = -0.05 + 0.1 / 30 * (cnt % 30) + cdpr.zOff
        # elif cnt < 120:
        #     xRef = -0.2 + cdpr.xOff
        #     yRef = -0.2 + 0.4 / 30 * (cnt % 30) + cdpr.yOff
        #     zRef = 0.05 - 0.1 / 30 * (cnt % 30) + cdpr.zOff

        posRef = np.array([xRef, yRef, zRef])
        print('posRef: {}'.format(posRef))
        xRefList.append(xRef)
        yRefList.append(yRef)
        zRefList.append(zRef)

        cnt += 1

        # referrence pose of moment k+1  (unit: m)
        # circle
        # xRefNext = 0.3 * math.sin(2*math.pi/20 * T * cnt) + cdpr.xOff
        # yRefNext = 0.3 * math.cos(2*math.pi/20 * T * cnt) + cdpr.yOff
        # zRefNext = 0.05 * math.sin(2*math.pi/10 * T * cnt) + cdpr.zOff
        # fixed point
        # xRefNext = xRef
        # yRefNext = yRef
        # zRefNext = zRef
        # square
        if cnt < 30:
            xRefNext = cdpr.xOff
            yRefNext = cdpr.yOff
            zRefNext = 0.2 / 30 * (cnt % 30) + cdpr.zOff
        # elif cnt < 60:
        #     xRefNext = 0.2 + cdpr.xOff
        #     yRefNext = 0.2 - 0.4 / 30 * (cnt % 30) + cdpr.yOff
        #     zRefNext = 0.05 - 0.1 / 30 * (cnt % 30) + cdpr.zOff
        # elif cnt < 90:
        #     xRefNext = 0.2 - 0.4 / 30 * (cnt % 30) + cdpr.xOff
        #     yRefNext = -0.2 + cdpr.yOff
        #     zRefNext = -0.05 + 0.1 / 30 * (cnt % 30) + cdpr.zOff
        # elif cnt < 120:
        #     xRefNext = -0.2 + cdpr.xOff
        #     yRefNext = -0.2 + 0.4 / 30 * (cnt % 30) + cdpr.yOff
        #     zRefNext = 0.05 - 0.1 / 30 * (cnt % 30) + cdpr.zOff

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

        # if(z < 0.01):
        #     cdpr.setMotorVelo(0, 0)
        #     break


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
        veloJoint = veloJoint.reshape(3,)
        print('veloJoint: {}'.format(veloJoint))

        # uav pose of moment k  (unit: m)
        uavPos = cdpr.getUAVPos()
        uavXList.append(uavPos[0])
        uavYList.append(uavPos[1])
        uavZList.append(uavPos[2])
        uavPos = np.array(uavPos)
        print('uavPos: {}'.format(uavPos))

        # uav target pose of moment k  (unit: m)
        uavTagPos = pos + np.array([0, 0, cdpr.uavCableLen])
        uavTagXList.append(uavTagPos[0])
        uavTagYList.append(uavTagPos[1])
        uavTagZList.append(uavTagPos[2])
        print('uavTagPos: {}'.format(uavTagPos))


        # output UAV velo  (unit: m/s)
        uavPosDis = np.linalg.norm(uavTagPos - uavPos)
        pid = controller(2.1, 0, 0, (-0.6, 0.6))
        pid.set_point(0)
        u = -pid.controller(uavPosDis)
        print('pid output: {}'.format(u))
        fbVelo = u * (uavTagPos - uavPos) / np.linalg.norm(uavTagPos - uavPos)
        ffVelo = 1 * (pos - lastPos)
        jVelo = - veloJoint[2] * (uavPos - pos) / np.linalg.norm(uavPos - pos)
        uavVelo = fbVelo + 0.1*ffVelo + jVelo + np.array([0, -0.04, 0.05])
        print('feedback velo: {}'.format(fbVelo))
        print('feedforward velo: {}'.format(ffVelo))
        print('joint velo: {}'.format(jVelo))

        print(time.time() - start_time)

        # set actuator velocity
        cdpr.setUAVVelo(uavVelo)
        print('uavVelo: {}'.format(uavVelo))
        print(time.time() - start_time)
        cdpr.setMotorVelo(-int(veloJoint[0]*60/(0.03*math.pi)*10), -int(veloJoint[1]*60/(0.03*math.pi)*10))
        print('motorVelo :{}'.format([int(veloJoint[0]*60/(0.051*math.pi)*10), int(veloJoint[1]*60/(0.051*math.pi)*10)]))
        print(time.time() - start_time)

        # update last position of platform
        lastPos = pos

        # save data
        pos_data = (np.array([runTime, xRef, yRef, zRef, x, y, z, uavTagPos[0], uavTagPos[1], uavTagPos[2], uavPos[0], uavPos[1], uavPos[2], orientation[0], orientation[1], orientation[2], orientation[3]])).reshape(1, 17)
        velo_data = (np.array([runTime, veloTask[0], veloTask[1], veloTask[2], veloJoint[0], veloJoint[1], veloJoint[2], uavVelo[0], uavVelo[1], uavVelo[2], fbVelo[0], fbVelo[1], fbVelo[2], ffVelo[0], ffVelo[1], ffVelo[2], jVelo[0], jVelo[1], jVelo[2]])).reshape(1, 19)
        traject_pos = np.append(traject_pos, pos_data, axis=0)
        traject_velo = np.append(traject_velo, velo_data, axis=0)

        io.savemat('data/test46sq_pos.mat', {'name': traject_pos})
        io.savemat('data/test46sq_velo.mat', {'name': traject_velo})
        print('data saved')

        end_time = time.time()
        print(end_time - start_time)

        rate.sleep()
            

    cdpr.setMotorVelo(0, 0)
    cdpr.setUAVVelo([0,0,0])

    
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

    
