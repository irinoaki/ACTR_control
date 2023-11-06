#! /usr/bin/env python3

#It's the version paraphrased from the cpp version
import queue
import rospy
from geometry_msgs.msg import PoseStamped,TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

current_state = State()
velo_flag = False

def state_cb(msg):
    global current_state
    current_state = msg

def velocity_cb(TwistStamped):
    global velo_flag
    velo_flag = True
    set_velocity.twist.linear.x=TwistStamped.twist.linear.x
    set_velocity.twist.linear.y=TwistStamped.twist.linear.y
    set_velocity.twist.linear.z=TwistStamped.twist.linear.z


def pose_cb(PoseStamped):
    uavpose.pose.position.x = PoseStamped.pose.position.x
    uavpose.pose.position.y = PoseStamped.pose.position.y
    uavpose.pose.position.z = PoseStamped.pose.position.z

if __name__ == "__main__":
    rospy.init_node("offb_node_py")
    set_velocity = TwistStamped()
    uavpose = PoseStamped()

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb,queue_size=10)
    pub_velocity = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
    uavVeloSub = rospy.Subscriber('/outer_control/twist', TwistStamped, velocity_cb, queue_size=10)
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    #local_pos_sub = rospy.Subscriber("/vrpn_client_node/UAV_2_new/pose",PoseStamped,pose_cb,queue_size=10)
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)


    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()

    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 0.3

    # Send a few setpoints before starting
    for i in range(100):
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    land_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'
    land_set_mode.custom_mode = 'AUTO.LAND'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()
    rate_2hz = rospy.Rate(2)
    while True:
        set_mode_client.call(offb_set_mode)
        if(set_mode_client.call(offb_set_mode).mode_sent == True):
            rospy.loginfo("OFFBOARD enabled")
        if(not rospy.is_shutdown() and current_state.mode == "OFFBOARD"):
            break
        rate.sleep()
    
    while True:
        arming_client.call(arm_cmd)
        if(arming_client.call(arm_cmd).success == True):
            rospy.loginfo("Vehicle armed")
        if(not rospy.is_shutdown() and current_state.armed):
            break
        local_pos_pub.publish(pose)
        rate.sleep()

    # while(not rospy.is_shutdown()):
    #     if(velo_flag == False):
    #         local_pos_pub.publish(pose)
    #     else:
    #         pub_velocity.publish(set_velocity)
    #     velo_flag = False
    #     rate.sleep()
           
    

    # while(not rospy.is_shutdown()):
    #     if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
    #         if(set_mode_client.call(offb_set_mode).mode_sent == True):
    #             rospy.loginfo("OFFBOARD enabled")

    #         last_req = rospy.Time.now()
    #     else:
    #         if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
    #             if(arming_client.call(arm_cmd).success == True):
    #                 rospy.loginfo("Vehicle armed")

    #             last_req = rospy.Time.now()

    #     #local_pos_pub.publish(pose)
    #     pub_velocity.publish(set_velocity)

    #     rate.sleep()