#! /usr/bin/env python3

# """
#  * File: offb_node.py
#  * Stack and tested in Gazebo Classic 9 SITL
# """

#It's for the offboard mode(two UAVs' velocity)
from operator import truediv
from os import posix_fadvise
import queue
import rospy
import scipy.interpolate as spy
import scipy.io as io
import pickle
import numpy as np
import time
import math

from geometry_msgs.msg import PoseStamped,TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from cdpr_test02 import CDPR
from pid import controller
from transform import euler2quaternion
from Jacobian import getJacobian
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

current_state0 = State()
current_state1 = State()

def state_cb0(msg):
    global current_state0
    current_state0 = msg

def state_cb1(msg):
    global current_state1
    current_state1 = msg

# def velocity_cb0(data):
#     set_velocity0.twist.linear.x=data.twist.linear.x
#     set_velocity0.twist.linear.y=data.twist.linear.y
#     set_velocity0.twist.linear.z=data.twist.linear.z

# def velocity_cb1(data):
#     set_velocity1.twist.linear.x=data.twist.linear.x
#     set_velocity1.twist.linear.y=data.twist.linear.y
#     set_velocity1.twist.linear.z=data.twist.linear.z


if __name__ == "__main__":
    # rospy.init_node("offb_node_py")

    # set the velocity control rate
    T = 0.25
    cdpr=CDPR()
    rate_velo = rospy.Rate(1/T)

    # get the original planning data and the additional data by interpolation
    filename='/home/irono/CDPR_AWS/Codes/ROS/ACTR_Planning/src/actr_planning/script/trajectory_ca.txt'
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
    t=np.linspace(0,1.5*(int(data_len/18)-1),int(data_len/18))
    t_3=np.linspace(t[0],t[-1],int(1.5*(int(data_len/18)-1)/T+1))
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
    traject_pos = np.zeros((0, 23))
    traject_velo = np.zeros((0, 29))
    time.sleep(0.1)

    # set the topics
    set_velocity0 = TwistStamped()
    set_velocity1 = TwistStamped()
    state_sub0 = rospy.Subscriber("/uav0/mavros/state", State, callback = state_cb0,queue_size=10)
    pub_velocity0 = rospy.Publisher("uav0/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
    ## uavVeloSub0 = rospy.Subscriber('/outer_control_0/twist', TwistStamped, velocity_cb0, queue_size=10)
    local_pos_pub0 = rospy.Publisher("uav0/mavros/setpoint_position/local", PoseStamped, queue_size=10)

    state_sub1 = rospy.Subscriber("/uav1/mavros/state", State, callback = state_cb1,queue_size=10)
    pub_velocity1 = rospy.Publisher("uav1/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
    ## uavVeloSub1 = rospy.Subscriber('/outer_control_1/twist', TwistStamped, velocity_cb1, queue_size=10)
    local_pos_pub1 = rospy.Publisher("uav1/mavros/setpoint_position/local", PoseStamped, queue_size=10)

    rospy.wait_for_service("/uav0/mavros/cmd/arming")
    arming_client0 = rospy.ServiceProxy("/uav0/mavros/cmd/arming", CommandBool)
    rospy.wait_for_service("/uav0/mavros/set_mode")
    set_mode_client0 = rospy.ServiceProxy("/uav0/mavros/set_mode", SetMode)

    rospy.wait_for_service("/uav1/mavros/cmd/arming")
    arming_client1 = rospy.ServiceProxy("/uav1/mavros/cmd/arming", CommandBool)
    rospy.wait_for_service("/uav1/mavros/set_mode")
    set_mode_client1 = rospy.ServiceProxy("/uav1/mavros/set_mode", SetMode)

    # Setpoint publishing MUST be faster than 2Hz
    rate_50 = rospy.Rate(50)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state0.connected and not current_state1.connected):
        rate_50.sleep()

    pose0 = PoseStamped()
    pose1 = PoseStamped()

    pose0.pose.position.x = -0.85
    pose0.pose.position.y = 0.17+0.5
    pose0.pose.position.z = 1

    pose1.pose.position.x = -0.85
    pose1.pose.position.y = 0.17-0.5
    pose1.pose.position.z = 1

    # Send a few setpoints before starting
    for i in range(100):
        if(rospy.is_shutdown()):
            break

        local_pos_pub0.publish(pose0)
        local_pos_pub1.publish(pose1)
        rate_50.sleep()

    # set the mode that will be used then
    offb_set_mode0 = SetModeRequest()
    land_set_mode0 = SetModeRequest()
    posi_set_mode0 = SetModeRequest()
    offb_set_mode0.custom_mode = 'OFFBOARD'
    land_set_mode0.custom_mode = 'AUTO.LAND'
    posi_set_mode0.custom_mode = 'POSITION'

    offb_set_mode1 = SetModeRequest()
    land_set_mode1 = SetModeRequest()
    posi_set_mode1 = SetModeRequest()
    offb_set_mode1.custom_mode = 'OFFBOARD'
    land_set_mode1.custom_mode = 'AUTO.LAND'
    posi_set_mode1.custom_mode = 'POSITION'

    arm_cmd0 = CommandBoolRequest()
    arm_cmd0.value = True
    arm_cmd1 = CommandBoolRequest()
    arm_cmd1.value = True

    last_req = rospy.Time.now()
    rate_2hz = rospy.Rate(2)

    #set the offboard mode  
    while True:
        set_mode_client0.call(offb_set_mode0)
        set_mode_client1.call(offb_set_mode1)
        if(set_mode_client0.call(offb_set_mode0).mode_sent == True and set_mode_client1.call(offb_set_mode1).mode_sent == True):
            rospy.loginfo("UAV1 and UAV1 OFFBOARD enabled")
        if(not rospy.is_shutdown() and current_state0.mode == "OFFBOARD" and current_state1.mode == "OFFBOARD"):
            break
        rate_2hz.sleep()
    
    # make two UAVs be armed
    while True:
        arming_client0.call(arm_cmd0)
        arming_client1.call(arm_cmd1)
        if(arming_client0.call(arm_cmd0).success == True and arming_client1.call(arm_cmd1).success == True):
            rospy.loginfo("Vehicle0 and Vehicle1 armed")
        if(not rospy.is_shutdown() and current_state0.armed and current_state1.armed):
            break
        rate_2hz.sleep()
    
    # let the UAVs reach the setting points
    hold_on_time = 15
    loof_freq = 10
    rate_loop = rospy.Rate(loof_freq)
    rospy.loginfo("Sent Position Start")
    for i in range(hold_on_time*loof_freq):
        local_pos_pub0.publish(pose0)
        local_pos_pub1.publish(pose1)
        rate_loop.sleep()
    rospy.loginfo("Sent Position End")

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

        veloTask=np.array([XRefVeloArr_it[cnt],YRefVeloArr_it[cnt],ZRefVeloArr_it[cnt]])
        posDis=np.linalg.norm(posRef-pos)
        pid=controller(0,0,0,(-0.5,0.5))
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
        print('uavPos0: {}'.format(uavPos0))        
        print('uavPos1: {}'.format(uavPos1))

        ff_uavVelo0=np.array([UAVRefVeloArr0_X_it[cnt],UAVRefVeloArr0_Y_it[cnt],UAVRefVeloArr0_Z_it[cnt]])
        ff_uavVelo1=np.array([UAVRefVeloArr1_X_it[cnt],UAVRefVeloArr1_Y_it[cnt],UAVRefVeloArr1_Z_it[cnt]])
        uavTagPos0=np.array([UAVRefArr0_X_it[cnt],UAVRefArr0_Y_it[cnt],UAVRefArr0_Z_it[cnt]])
        uavTagPos1=np.array([UAVRefArr1_X_it[cnt],UAVRefArr1_Y_it[cnt],UAVRefArr1_Z_it[cnt]])
        print('uavTagPos0: {}'.format(uavTagPos0))
        print('uavTagPos1: {}'.format(uavTagPos1))
        print('feedforward velo0: {}',format(ff_uavVelo0))
        print('feedforward velo1: {}',format(ff_uavVelo1))

        uavPosDis0 = np.linalg.norm(uavTagPos0 - uavPos0)
        uavPosDis1 = np.linalg.norm(uavTagPos1 - uavPos1)
        pid = controller(0.25, 0.1, 0.2, (-0.6, 0.6))
        pid.set_point(0)
        u0 = -pid.controller(uavPosDis0)
        u1 = -pid.controller(uavPosDis1)
        fbVelo0 = u0 * (uavTagPos0 - uavPos0) / np.linalg.norm(uavTagPos0 - uavPos0)
        fbVelo1 = u1 * (uavTagPos1 - uavPos1) / np.linalg.norm(uavTagPos1 - uavPos1)
        uavVelo0=ff_uavVelo0+fbVelo0
        uavVelo1=ff_uavVelo1+fbVelo1
        print('feedback velo0: {}'.format(fbVelo0))
        print('feedback velo1: {}'.format(fbVelo1))

        print(time.time() - start_time)       

        # cdpr.setUAVVelo0(uavVelo0)
        set_velocity0.twist.linear.x = uavVelo0[0]
        set_velocity0.twist.linear.y = uavVelo0[1]
        set_velocity0.twist.linear.z = uavVelo0[2]
        print('uavVelo0: {}'.format(uavVelo0))
        # cdpr.setUAVVelo1(uavVelo1)
        set_velocity1.twist.linear.x = uavVelo1[0]
        set_velocity0.twist.linear.y = uavVelo1[1]
        set_velocity0.twist.linear.z = uavVelo1[2]
        pub_velocity0.publish(set_velocity0)
        pub_velocity1.publish(set_velocity1)
        print('uavVelo1: {}'.format(uavVelo1))
        if abs(int(veloJoint[0]*60/(0.051*math.pi)*10))>450 or abs(int(veloJoint[1]*60/(0.051*math.pi)*10))>450:
            cdpr.setMotorVelo(-int(np.sign(veloJoint[0]))*450, -int(np.sign(veloJoint[1]))*450)
            print('mototvelo :{}'.format([np.sign(veloJoint[0])*450, np.sign(veloJoint[1])*450]))
        else:
            cdpr.setMotorVelo(-int(veloJoint[0]*60/(0.051*math.pi)*10), -int(veloJoint[1]*60/(0.051*math.pi)*10))
            print('motorVelo :{}'.format([int(veloJoint[0]*60/(0.051*math.pi)*10), int(veloJoint[1]*60/(0.051*math.pi)*10)]))
        print(time.time() - start_time)

        pos_data = (np.array([runTime, x, y, z, XRefArr_it[cnt], YRefArr_it[cnt], ZRefArr_it[cnt], uavTagPos0[0], uavTagPos0[1], uavTagPos0[2], uavTagPos1[0], uavTagPos1[1], uavTagPos1[2], uavPos0[0], uavPos0[1], uavPos0[2], uavPos1[0], uavPos1[1], uavPos1[2], orientation[0], orientation[1], orientation[2], orientation[3]])).reshape(1, 23)
        velo_data = (np.array([runTime, XRefVeloArr_it[cnt], YRefVeloArr_it[cnt], ZRefVeloArr_it[cnt], veloTask[0], veloTask[1], veloTask[2], veloJoint[0], veloJoint[1], veloJoint[2], veloJoint[3], uavVelo0[0], uavVelo0[1], uavVelo0[2], uavVelo1[0], uavVelo1[1], uavVelo1[2], ff_uavVelo0[0], ff_uavVelo0[1], ff_uavVelo0[2], ff_uavVelo1[0], ff_uavVelo1[1], ff_uavVelo1[2], fbVelo0[0], fbVelo0[1], fbVelo0[2], fbVelo1[0], fbVelo1[1], fbVelo1[2]])).reshape(1, 29)
        traject_pos = np.append(traject_pos, pos_data, axis=0)
        traject_velo = np.append(traject_velo, velo_data, axis=0)

        cnt+=1
        io.savemat('data/traject_pos.mat', {'name': traject_pos})
        io.savemat('data/traject_velo.mat', {'name': traject_velo})
        print('data saved')

        end_time = time.time()
        print(end_time - start_time)

        rate_velo.sleep()
    
    cdpr.setMotorVelo(0,0)
    # cdpr.setUAVVelo0([0,0,0])
    # cdpr.setUAVVelo1([0,0,0])
    set_velocity0.twist.linear.x = 0
    set_velocity0.twist.linear.y = 0
    set_velocity0.twist.linear.z = 0
    set_velocity0.twist.linear.x = 0
    set_velocity0.twist.linear.y = 0
    set_velocity0.twist.linear.z = 0
    pub_velocity0.publish(set_velocity0)
    pub_velocity1.publish(set_velocity1)    


    # cnt = 0
    # while(not rospy.is_shutdown()):
    #     pub_velocity0.publish(set_velocity0)
    #     pub_velocity1.publish(set_velocity1)
    #     cnt += 1
    #     if cnt == 100:
    #         break
    #     rate.sleep()

    rospy.loginfo("Land")
    while True:
        set_mode_client0.call(land_set_mode0)
        set_mode_client1.call(land_set_mode1)
        rospy.loginfo("Set Mode")
        if (current_state0.mode != "AUTO.LAND" and current_state1.mode != "AUTO.LAND"):
            break
        rate_2hz.sleep()

    fig1 = plt.figure(1)
    xFig = fig1.add_subplot(3,1,1)
    yFig = fig1.add_subplot(3,1,2)
    zFig = fig1.add_subplot(3,1,3)
    fig2 = plt.figure(2)
    uav0xFig = fig2.add_subplot(3,2,1)
    uav0yFig = fig2.add_subplot(3,2,3)
    uav0zFig = fig2.add_subplot(3,2,5)
    uav1xFig = fig2.add_subplot(3,2,2)
    uav1yFig = fig2.add_subplot(3,2,4)
    uav1zFig = fig2.add_subplot(3,2,6)
    fig3 = plt.figure(3)
    ax = Axes3D(fig3)
    plt.ion()

    xFig.plot(t_3, xList)
    xFig.plot(t_3, XRefArr_it.tolist()[:])
    yFig.plot(t_3, yList)
    yFig.plot(t_3, YRefArr_it.tolist()[:])
    zFig.plot(t_3, zList)    
    zFig.plot(t_3, ZRefArr_it.tolist()[:])

    uav0xFig.plot(t_3, uavXList0)
    uav0xFig.plot(t_3, UAVRefArr0_X_it.tolist()[:])
    uav0yFig.plot(t_3, uavYList0)
    uav0yFig.plot(t_3, UAVRefArr0_Y_it.tolist()[:])
    uav0zFig.plot(t_3, uavZList0)
    uav0zFig.plot(t_3, UAVRefArr0_Z_it.tolist()[:])    
    uav1xFig.plot(t_3, uavXList1)
    uav1xFig.plot(t_3, UAVRefArr1_X_it.tolist()[:])
    uav1yFig.plot(t_3, uavYList1)
    uav1yFig.plot(t_3, UAVRefArr1_Y_it.tolist()[:])
    uav1zFig.plot(t_3, uavZList1)
    uav1zFig.plot(t_3, UAVRefArr1_Z_it.tolist()[:]) 

    ax.plot3D(XRefArr_it.tolist()[:], YRefArr_it.tolist()[:], ZRefArr_it.tolist()[:])
    ax.plot3D(xList[:], yList[:], zList[:])
    plt.ioff()
    plt.show()    
    


