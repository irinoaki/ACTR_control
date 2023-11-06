#! /usr/bin/env python3

# """
#  * File: offb_node.py
#  * Stack and tested in Gazebo Classic 9 SITL
# """
#from operator import truediv

#It's for the offboard mode(one UAV's velocity)
import rospy
import time
import numpy as np

from geometry_msgs.msg import PoseStamped,TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

current_state0 = State()
current_state1 = State()
# xoff = -0.439
# yoff = 0.296
# zoff = 0.021
# movingPlatformPose = PoseStamped()
velo_flag = False

def state_cb(msg):
    global current_state0
    current_state0 = msg

# def state_cb1(msg):
#     global current_state1
#     current_state1 = msg

def velocity_cb(data):
    set_velocity.twist.linear.x=data.twist.linear.x
    set_velocity.twist.linear.y=data.twist.linear.y
    set_velocity.twist.linear.z=data.twist.linear.z

# def poseCallback(data):
#     movingPlatformPose.pose.position.x = data.pose.position.x / 1000
#     movingPlatformPose.pose.position.y = data.pose.position.y / 1000
#     movingPlatformPose.pose.position.z = data.pose.position.z / 1000
#     movingPlatformPose.pose.orientation = data.pose.orientation

#     # header
#     movingPlatformPose.header.frame_id = data.header.frame_id
#     movingPlatformPose.header.stamp = data.header.stamp


if __name__ == "__main__":
    rospy.init_node("offb_node_py")
    set_velocity = TwistStamped()
    state_sub0 = rospy.Subscriber("/mavros/state", State, callback = state_cb,queue_size=10)
    pub_velocity0 = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
    uavVeloSub = rospy.Subscriber('/outer_control/twist', TwistStamped, velocity_cb, queue_size=10)
    local_pos_pub0 = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
    # rospy.Subscriber('/vrpn_client_node/BOX/pose', PoseStamped, poseCallback, queue_size=10)

    # state_sub1 = rospy.Subscriber("/uav1/mavros/state", State, callback = state_cb1,queue_size=10)
    # pub_velocity1 = rospy.Publisher("uav1/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
    # uavVeloSub = rospy.Subscriber('/outer_control/twist', TwistStamped, velocity_cb, queue_size=10)
    # local_pos_pub1 = rospy.Publisher("uav1/mavros/setpoint_position/local", PoseStamped, queue_size=10)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client0 = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client0 = rospy.ServiceProxy("/mavros/set_mode", SetMode)

    # rospy.wait_for_service("/uav1/mavros/cmd/arming")
    # arming_client1 = rospy.ServiceProxy("/uav1/mavros/cmd/arming", CommandBool)
    # rospy.wait_for_service("/uav1/mavros/set_mode")
    # set_mode_client1 = rospy.ServiceProxy("/uav1/mavros/set_mode", SetMode)


    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(5)
    rate2 = rospy.Rate(50)

        # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state0.connected):
        rate.sleep()

    pose0 = PoseStamped()
    # pose1 = PoseStamped()

    pose0.pose.position.x = 0
    pose0.pose.position.y = 0
    pose0.pose.position.z = 0.3

    # pose1.pose.position.x = 0
    # pose1.pose.position.y = 0
    # pose1.pose.position.z = 0.5

    # Send a few setpoints before starting
    for i in range(100):
        if(rospy.is_shutdown()):
            break

        local_pos_pub0.publish(pose0)
        # local_pos_pub1.publish(pose1)
        rate2.sleep()

    offb_set_mode0 = SetModeRequest()
    land_set_mode0 = SetModeRequest()
    posi_set_mode0 = SetModeRequest()
    offb_set_mode0.custom_mode = 'OFFBOARD'
    posi_set_mode0.custom_mode = 'POSITION'
    land_set_mode0.custom_mode = 'AUTO.LAND'

    # offb_set_mode1 = SetModeRequest()
    # land_set_mode1 = SetModeRequest()
    # offb_set_mode1.custom_mode = 'OFFBOARD'
    # land_set_mode1.custom_mode = 'AUTO.LAND'

    arm_cmd0 = CommandBoolRequest()
    arm_cmd0.value = True
    # arm_cmd1 = CommandBoolRequest()
    # arm_cmd1.value = True

    last_req = rospy.Time.now()
    rate_2hz = rospy.Rate(2)
    while True:
        set_mode_client0.call(offb_set_mode0)
        # set_mode_client1.call(offb_set_mode1)
        if(set_mode_client0.call(offb_set_mode0).mode_sent == True):
            rospy.loginfo("UAV0 OFFBOARD enabled")
        if(not rospy.is_shutdown() and current_state0.mode == "OFFBOARD" ):
            break
        rate_2hz.sleep()
    
    while True:
        arming_client0.call(arm_cmd0)
        # arming_client1.call(arm_cmd1)
        if(arming_client0.call(arm_cmd0).success == True):
            rospy.loginfo("Vehicle0 and Vehicle1 armed")
        if(not rospy.is_shutdown() and current_state0.armed):
            break
        rate_2hz.sleep()
    
    # hold_on_time = 20.0
    # loof_freq = 10
    # rate_loop = rospy.Rate(loof_freq)
    # rospy.loginfo("Sent Position Start")
    # for i in range(hold_on_time*loof_freq):
    #     local_pos_pub0.publish(pose0)
    #     # local_pos_pub1.publish(pose1)
    #     rate_loop.sleep()
    # rospy.loginfo("Sent Position End")
    cnt = 0
    while(not rospy.is_shutdown()):
        pub_velocity0.publish(set_velocity)
        cnt += 1
        if cnt == 100:
            break
        rate.sleep()
    
    # hold_on_time = 10.0
    # loof_freq = 10
    # rate_loop = rospy.Rate(loof_freq)
    # rospy.loginfo("Sent Position Start")
    # while True:
    #     set_mode_client0.call(posi_set_mode0)
    #     if(set_mode_client0.call(posi_set_mode0).mode_sent == True):
    #         rospy.loginfo("UAVO POSITION enabled")
    #     if(not rospy.is_shutdown() and current_state0.mode =="POSITION"):
    #         break
    # for i in range(hold_on_time*loof_freq):
    #     #local_pos_pub0.publish(pose0)
    #     # local_pos_pub1.publish(pose1)
    #     rate_loop.sleep()
    # rospy.loginfo("Position End")

    rospy.loginfo("Land")
    while True:
        set_mode_client0.call(land_set_mode0)
        # set_mode_client1.call(land_set_mode1)
        rospy.loginfo("Set Mode")
        if(set_mode_client0.call(land_set_mode0).mode_sent == True):
            rospy.loginfo("UAV0 LAND enabled")
        if (current_state0.mode != "AUTO.LAND"):
            break
        rate_2hz.sleep()
    

    # while(not rospy.is_shutdown()):
    #     if(velo_flag == False):
    #         local_pos_pub0.publish(pose0)
    #         # local_pos_pub1.publish(pose1)
    #     else:
    #         pub_velocity0.publish(set_velocity)
    #     velo_flag = False
    #     rate.sleep()

    # while(not rospy.is_shutdown()):
    #     if(current_state0.mode != "OFFBOARD" and current_state1.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0) ):
    #         if(set_mode_client0.call(offb_set_mode0).mode_sent == True):
    #             rospy.loginfo("UAV0 OFFBOARD enabled")
    #         if(set_mode_client1.call(offb_set_mode1).mode_sent == True):
    #             rospy.loginfo("UAV1 OFFBOARD enabled")
    #             rospy.loginfo(current_state0.mode)
    #             rospy.loginfo(current_state1.mode)
    #         last_req = rospy.Time.now()
    #     else:
    #         if(not current_state0.armed and not current_state1.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
    #             if(arming_client0.call(arm_cmd0).success == True):
    #                 rospy.loginfo("Vehicle0 and Vehicle1 armed")
    #             if(arming_client1.call(arm_cmd1).success == True):
    #                 rospy.loginfo("Vehicle1 and Vehicle1 armed")
    #             last_req = rospy.Time.now()
    #     #local_pos_pub.publish(pose)
    #     #pub_velocity.publish(set_velocity)

    #     rate.sleep()