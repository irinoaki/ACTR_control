#! /usr/bin/env python3

import time
import numpy as np
import math
import rospy
import matplotlib.pyplot as plt
import scipy.io as io

from cdpr import CDPR
from transform import euler2quaternion
from Jacobian import getJacobian


if __name__=="__main__":

    cdpr = CDPR()

    T = 0.1     # control period
    rate = rospy.Rate(1/T)
    
    x_r_list, y_r_list, z_r_list = [], [], []
    x_list, y_list, z_list = [], [], []

    ########## main loop ##########

    cnt = 0

    cdpr.pretighten()

    while not rospy.is_shutdown() and cnt < 40 / T:

        print(cnt)
        start_time = time.time()
        
        # referrence pose of moment k  (unit: degree)
        t = cnt * T
        x_r = 0.1 * math.sin(math.pi/20 * t) + cdpr.xOff
        y_r = 0.1 * math.cos(math.pi/20 * t) + cdpr.yOff
        z_r = 0.08 * math.sin(math.pi/10 * t) + 0 + cdpr.zOff
        # side = 0.3
        # wave = 0.0
        # if cnt < 100:
        #     x_r = -side / 2 + side / 100 * (cnt % 100) + cdpr.xOff
        #     y_r = 0.2 + cdpr.yOff
        #     z_r = -wave / 2 + wave / 100 * (cnt % 100) + cdpr.zOff
        # elif cnt < 200:
        #     x_r = side / 2 + cdpr.xOff
        #     y_r = side / 2 - side / 100 * (cnt % 100) + cdpr.yOff
        #     z_r = wave / 2 - wave / 100 * (cnt % 100) + cdpr.zOff
        # elif cnt < 300:
        #     x_r = side / 2 - side / 100 * (cnt % 100) + cdpr.xOff
        #     y_r = -side / 2 + cdpr.yOff
        #     z_r = -wave / 2 + wave / 100 * (cnt % 100) + cdpr.zOff
        # elif cnt < 400:
        #     x_r = -side / 2 + cdpr.xOff
        #     y_r = -side / 2 + side / 100 * (cnt % 100) + cdpr.yOff
        #     z_r = wave / 2 - wave / 100 * (cnt % 100) + cdpr.zOff
        pos_r = np.array([x_r, y_r, z_r])
        print('posRef: {}'.format(pos_r))

        cnt += 1

        # referrence pose of moment k+1  (unit: degree)
        t = cnt * T
        x_r_next = 0.1 * math.sin(math.pi/20 * t) + cdpr.xOff
        y_r_next = 0.1 * math.cos(math.pi/20 * t) + cdpr.yOff
        z_r_next = 0.08 * math.sin(math.pi/10 * t) + 0 + cdpr.zOff
        # if cnt < 100:
        #     x_r_next = -side / 2 + side / 100 * (cnt % 100) + cdpr.xOff
        #     y_r_next = 0.2 + cdpr.yOff
        #     z_r_next = -wave / 2 + wave / 100 * (cnt % 100) + cdpr.zOff
        # elif cnt < 200:
        #     x_r_next = side / 2 + cdpr.xOff
        #     y_r_next = side / 2 - side / 100 * (cnt % 100) + cdpr.yOff
        #     z_r_next = wave / 2 - wave / 100 * (cnt % 100) + cdpr.zOff
        # elif cnt < 300:
        #     x_r_next = side / 2 - side / 100 * (cnt % 100) + cdpr.xOff
        #     y_r_next = -side / 2 + cdpr.yOff
        #     z_r_next = -wave / 2 + wave / 100 * (cnt % 100) + cdpr.zOff
        # elif cnt < 400:
        #     x_r_next = -side / 2 + cdpr.xOff
        #     y_r_next = -side / 2 + side / 100 * (cnt % 100) + cdpr.yOff
        #     z_r_next = wave / 2 - wave / 100 * (cnt % 100) + cdpr.zOff
        pos_r_next = np.array([x_r_next, y_r_next, z_r_next])

        # pose of moment k  (unit: degree)
        x, y, z, orientation = cdpr.getMovingPlatformPose()
        pos = np.array([x, y, z])
        print('pos: {}'.format(pos))

        # error of moment k  (unit: degree)
        pos_err = pos_r - pos

        # output velocity of moment k  (unit: degree/s)
        eps = 0.002
        k = 1.5
        veloTask = (pos_r_next - pos_r) / T + eps * np.sign(pos_err) + k * pos_err           # control law
        print('veloTask: {}'.format(veloTask))

        # inverse kinematics
        # calculate with model 
        J = getJacobian(cdpr.getAnchorAPoses(), cdpr.getAnchorBPoses(), pos, orientation)
        veloJoint = np.matmul(J, veloTask.reshape(3, 1))
        veloJoint = veloJoint.reshape(4, )
        print('veloJoint: {}'.format(veloJoint))

        # convert linear velocities to velocities of motors
        veloJoint = veloJoint*60*10/(0.03*math.pi)      # 10 is the gear ratio, 0.03 is diameter of the coil

        # set cable velocity in joint space
        for i in range(4):
            if np.abs(veloJoint[i]) > 3000:      # velovity limit
                veloJoint[i] = 3000 * np.sign(veloJoint[i])
        set_start_time = time.time()
        cdpr.setMotorVelo(int(veloJoint[0]), int(veloJoint[1]), int(veloJoint[2]), int(veloJoint[3]))
        print('motor velo: {}, {}, {}'.format(veloJoint[0], veloJoint[1], veloJoint[2], veloJoint[3]))
        print(time.time() - set_start_time)
        
        
        x_r_list.append(x_r)
        y_r_list.append(y_r)
        z_r_list.append(z_r)
                    
        x_list.append(x)
        y_list.append(y)
        z_list.append(z)

        end_time = time.time()
        print(end_time - start_time)

        rate.sleep()

    cdpr.setMotorVelo(0, 0, 0, 0)
    
    # plot
    fig = plt.figure(1)
    x_plot = fig.add_subplot(3,1,1)
    y_plot = fig.add_subplot(3,1,2)
    z_plot = fig.add_subplot(3,1,3)
    plt.ion()

    x_plot.plot(np.arange(0, cnt*T, T), x_r_list)
    x_plot.plot(np.arange(0, cnt*T, T), x_list)
    y_plot.plot(np.arange(0, cnt*T, T), y_r_list)
    y_plot.plot(np.arange(0, cnt*T, T), y_list)
    z_plot.plot(np.arange(0, cnt*T, T), z_r_list)
    z_plot.plot(np.arange(0, cnt*T, T), z_list)

    x_plot.set_ylabel('x')
    y_plot.set_ylabel('y')
    z_plot.set_ylabel('z')

    plt.ioff()
    plt.show()
    
