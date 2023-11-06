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

from geometry_msgs.msg import PoseStamped,TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest


current_state0 = State()
current_state1 = State()

def state_cb0(msg):
    global current_state0
    current_state0 = msg

def state_cb1(msg):
    global current_state1
    current_state1 = msg

def velocity_cb0(data):
    set_velocity0.twist.linear.x=data.twist.linear.x
    set_velocity0.twist.linear.y=data.twist.linear.y
    set_velocity0.twist.linear.z=data.twist.linear.z

def velocity_cb1(data):
    set_velocity1.twist.linear.x=data.twist.linear.x
    set_velocity1.twist.linear.y=data.twist.linear.y
    set_velocity1.twist.linear.z=data.twist.linear.z


if __name__ == "__main__":
    rospy.init_node("offb_node_py")
    set_velocity0 = TwistStamped()
    set_velocity1 = TwistStamped()
    state_sub0 = rospy.Subscriber("/uav0/mavros/state", State, callback = state_cb0,queue_size=10)
    pub_velocity0 = rospy.Publisher("uav0/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
    uavVeloSub0 = rospy.Subscriber('/outer_control_0/twist', TwistStamped, velocity_cb0, queue_size=10)
    local_pos_pub0 = rospy.Publisher("uav0/mavros/setpoint_position/local", PoseStamped, queue_size=10)

    state_sub1 = rospy.Subscriber("/uav1/mavros/state", State, callback = state_cb1,queue_size=10)
    pub_velocity1 = rospy.Publisher("uav1/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
    uavVeloSub1 = rospy.Subscriber('/outer_control_1/twist', TwistStamped, velocity_cb1, queue_size=10)
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
    rate = rospy.Rate(5)
    rate_50 = rospy.Rate(50)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state0.connected and not current_state1.connected):
        rate.sleep()

    pose0 = PoseStamped()
    pose1 = PoseStamped()

    pose0.pose.position.x = -0.85
    pose0.pose.position.y = 0.682
    pose0.pose.position.z = 1
    pose1.pose.position.x = -0.85
    pose1.pose.position.y = -0.318
    pose1.pose.position.z = 1

    # Send a few setpoints before starting
    for i in range(100):
        if(rospy.is_shutdown()):
            break

        local_pos_pub0.publish(pose0)
        local_pos_pub1.publish(pose1)
        rate_50.sleep()

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
    
    while True:
        set_mode_client0.call(offb_set_mode0)
        set_mode_client1.call(offb_set_mode1)
        if(set_mode_client0.call(offb_set_mode0).mode_sent == True and set_mode_client1.call(offb_set_mode1).mode_sent == True):
            rospy.loginfo("UAV1 and UAV1 OFFBOARD enabled")
        if(not rospy.is_shutdown() and current_state0.mode == "OFFBOARD" and current_state1.mode == "OFFBOARD"):
            break
        rate_2hz.sleep()
    
    while True:
        arming_client0.call(arm_cmd0)
        arming_client1.call(arm_cmd1)
        if(arming_client0.call(arm_cmd0).success == True and arming_client1.call(arm_cmd1).success == True):
            rospy.loginfo("Vehicle0 and Vehicle1 armed")
        if(not rospy.is_shutdown() and current_state0.armed and current_state1.armed):
            break
        rate_2hz.sleep()
    
    hold_on_time = 20
    loof_freq = 10
    rate_loop = rospy.Rate(loof_freq)
    rospy.loginfo("Sent Position Start")
    for i in range(hold_on_time*loof_freq):
        local_pos_pub0.publish(pose0)
        local_pos_pub1.publish(pose1)
        rate_loop.sleep()
    rospy.loginfo("Sent Position End")
    # cnt = 0
    # while(not rospy.is_shutdown()):
    #     pub_velocity0.publish(set_velocity0)
    #     pub_velocity1.publish(set_velocity1)
    #     cnt += 1
    #     if cnt == 100:
    #         break
    #     rate.sleep()

    # hold_on_time = 10
    # loof_freq = 10
    # rate_loop = rospy.Rate(loof_freq)
    # while True:
    #     set_mode_client0.call(posi_set_mode0)
    #     set_mode_client1.call(posi_set_mode1)
    #     if(set_mode_client0.call(posi_set_mode0).mode_sent == True and set_mode_client1.call(posi_set_mode1).mode_sent == True):
    #         rospy.loginfo("UAVO and UAV1 POSITION enabled")
    #     if(not rospy.is_shutdown() and current_state0.mode =="POSITION" and current_state1.mode == True):
    #         break
    # for i in range(hold_on_time*loof_freq):
    #     local_pos_pub0.publish(pose0)
    #     local_pos_pub1.publish(pose1)
    #     rate_loop.sleep()
    # rospy.loginfo("Position End")

    rospy.loginfo("Land")
    while True:
        set_mode_client0.call(land_set_mode0)
        set_mode_client1.call(land_set_mode1)
        rospy.loginfo("Set Mode")
        if (current_state0.mode != "AUTO.LAND" and current_state1.mode != "AUTO.LAND"):
            break
        rate_2hz.sleep()
    



    # while(not rospy.is_shutdown()):
    #     if(velo_flag == False):
    #         local_pos_pub0.publish(pose0)
    #         local_pos_pub1.publish(pose1)
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