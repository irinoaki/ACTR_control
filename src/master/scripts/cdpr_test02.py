#! /usr/bin/env python3

#It's for the situation that two UAVs and some motors.

import queue
import rospy
import time
import numpy as np
import math

from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Int16MultiArray
from slave.srv import SetMotorVelo, GetMotorPos

from transform import quaternion2euler


class CDPR:
    def __init__(self):
        # ros settings
        rospy.init_node('cdpr_control', anonymous=True)

        # offset
        self.xOff = -0.85
        self.yOff = 0.170
        self.zOff = 0
        
        # pose and limit of the end effector
        self._movingPlatformPose = PoseStamped()
        self._uavPose0 = PoseStamped()
        self._uavPose1 = PoseStamped()
        rospy.Subscriber('/vrpn_client_node/BOX_1/pose', PoseStamped, self._poseCallback, queue_size=10)
        rospy.Subscriber('/vrpn_client_node/UAV_01/pose', PoseStamped, self._uavCallback0, queue_size=10)
        rospy.Subscriber('/vrpn_client_node/UAV_11/pose', PoseStamped, self._uavCallback1, queue_size=10)
        self._uavVeloPub0 = rospy.Publisher('/outer_control_0/twist', TwistStamped, queue_size=10)
        self._uavVeloPub1 = rospy.Publisher('/outer_control_1/twist', TwistStamped, queue_size=10)
        self.veloPub = rospy.Publisher('motor_velo', Int16MultiArray, queue_size=10)

        # anchors on the fixed frame (world frame)
        self._anchorA1Pos = [-1.580, 1.353, -0.015]
        self._anchorA2Pos = [0.805, -1.055, 0]
        self.len00=np.linalg.norm(np.array([self._anchorA1Pos[0] - self.xOff, self._anchorA1Pos[1] - self.yOff, self._anchorA1Pos[2] - (self.zOff)]))
        self.len01=np.linalg.norm(np.array([self.len00, 0.44]))
        self.delta0=self.len01-self.len00
        self.len10=np.linalg.norm(np.array([self._anchorA2Pos[0] - self.xOff, self._anchorA2Pos[1] - self.yOff, self._anchorA2Pos[2] - (self.zOff)]))
        self.len11=np.linalg.norm(np.array([self.len10, 0.44]))
        self.delta1=self.len11-self.len10
        #self._anchorA3Pos = [1.216, 3.484, 0.021]
        #self._anchorA4Pos = [-0.818, 3.426, 0.044]
        self._anchorA3Pos = [0, 0, 0]      # uav0
        self._anchorA4Pos = [0, 0, 0]      # uav1
        self.uavCableLen0 = 0.70
        self.uavCableLen1 = 0.70

        # anchors on the moving platform (body frame)
        self._anchorB1Pos = [0, 0, 0]
        self._anchorB2Pos = [0, 0, 0]
        self._anchorB3Pos = [0, 0, 0]
        self._anchorB4Pos = [0, 0, 0]
        #self._anchorB5Pos = [0, 0, 0]


    def _poseCallback(self, data):
        # pose
        self._movingPlatformPose.pose.position.x = data.pose.position.x / 1000
        self._movingPlatformPose.pose.position.y = data.pose.position.y / 1000
        self._movingPlatformPose.pose.position.z = data.pose.position.z / 1000
        self._movingPlatformPose.pose.orientation = data.pose.orientation

        # header
        self._movingPlatformPose.header.frame_id = data.header.frame_id
        self._movingPlatformPose.header.stamp = data.header.stamp


    def _uavCallback0(self, data):
        # pose
        self._uavPose0.pose.position.x = data.pose.position.x / 1000
        self._uavPose0.pose.position.y = data.pose.position.y / 1000
        self._uavPose0.pose.position.z = data.pose.position.z / 1000
        self._uavPose0.pose.orientation = data.pose.orientation

        self._anchorA3Pos = [self._uavPose0.pose.position.x, self._uavPose0.pose.position.y, self._uavPose0.pose.position.z]

        # header
        self._uavPose0.header.frame_id = data.header.frame_id
        self._uavPose0.header.stamp = data.header.stamp
    
    def _uavCallback1(self, data):
        # pose
        self._uavPose1.pose.position.x = data.pose.position.x / 1000
        self._uavPose1.pose.position.y = data.pose.position.y / 1000
        self._uavPose1.pose.position.z = data.pose.position.z / 1000
        self._uavPose1.pose.orientation = data.pose.orientation

        self._anchorA4Pos = [self._uavPose1.pose.position.x, self._uavPose1.pose.position.y, self._uavPose1.pose.position.z]

        # header
        self._uavPose1.header.frame_id = data.header.frame_id
        self._uavPose1.header.stamp = data.header.stamp

    def setMotorVelo(self, motor1Velo, motor2Velo):
        # rospy.wait_for_service('set_motor_velo')
        # try:
        #     set_motor_velo = rospy.ServiceProxy('set_motor_velo', SetMotorVelo)
        #     set_motor_velo(motor1Velo, motor2Velo, motor3Velo, motor4Velo)
        # except rospy.ServiceException as e:
        #     print("Service call failed: {}".format(e))
        motor_velo = Int16MultiArray(data=np.array([motor1Velo, motor2Velo]))
        self.veloPub.publish(motor_velo)



    def setUAVVelo0(self, uavVelo):
        uavTwist = TwistStamped()
        uavTwist.twist.linear.x = uavVelo[0]
        uavTwist.twist.linear.y = uavVelo[1]
        uavTwist.twist.linear.z = uavVelo[2]
        self._uavVeloPub0.publish(uavTwist)
    
    def setUAVVelo1(self, uavVelo):
        uavTwist = TwistStamped()
        uavTwist.twist.linear.x = uavVelo[0]
        uavTwist.twist.linear.y = uavVelo[1]
        uavTwist.twist.linear.z = uavVelo[2]
        self._uavVeloPub1.publish(uavTwist)
    
    # def getMotorPos(self):
    #     rospy.wait_for_service('set_motor_velo')
    #     try:
    #         get_motor_pos = rospy.ServiceProxy('get_motor_pos', GetMotorPos)
    #         # print('1: {}'.format(time.time()))
    #         response = get_motor_pos()
    #         print(response.motor1Pos, response.motor2Pos, response.motor3Pos, response.motor4Pos)
    #         return [response.motor1Pos, response.motor2Pos, response.motor3Pos, response.motor4Pos]
    #         # print('2: {}'.format(time.time()))
    #     except rospy.ServiceException as e:
    #         print("Service call failed: {}".format(e))


    def getMovingPlatformPose(self):
        return self._movingPlatformPose.pose.position.x, self._movingPlatformPose.pose.position.y, self._movingPlatformPose.pose.position.z,\
            [self._movingPlatformPose.pose.orientation.x, self._movingPlatformPose.pose.orientation.y, self._movingPlatformPose.pose.orientation.z, self._movingPlatformPose.pose.orientation.w]


    def getUAVPos0(self):
        return [self._uavPose0.pose.position.x, self._uavPose0.pose.position.y, self._uavPose0.pose.position.z]
    
    def getUAVPos1(self):
        return [self._uavPose1.pose.position.x, self._uavPose1.pose.position.y, self._uavPose1.pose.position.z]

    def pretighten(self):
        time.sleep(0.5)
        # cable1 pre-tightening
        print('cable1 pretightening...')
        self.setMotorVelo(100, 0)
        x0, y0, z0, _ = self.getMovingPlatformPose()
        while True:
            x, y, z, _ = self.getMovingPlatformPose()
            if np.linalg.norm(np.array([x,y,z]) - np.array([x0,y0,z0]), ord=2) > 0.003:
                self.setMotorVelo(0, 0)
                break
            else:
                time.sleep(0.1)
        #print('cable1 pretightened') 
        time.sleep(0.5)

        # cable2 pre-tightening
        print('cable2 pretightening...')
        self.setMotorVelo(0, 100)
        x0, y0, z0, _ = self.getMovingPlatformPose()
        while True:
            x, y, z, _ = self.getMovingPlatformPose()
            if np.linalg.norm(np.array([x,y,z]) - np.array([x0,y0,z0]), ord=2) > 0.003:
                self.setMotorVelo(0, 0)
                break
            else:
                time.sleep(0.1)
        #print('cable2 pretightened') 
        time.sleep(0.5)

        # # cable3 pre-tightening
        # print('cable3 pretightening...')
        # self.setMotorVelo(0, 0, -200, 0)
        # x0, y0, z0, _ = self.getMovingPlatformPose()
        # while True:
        #     x, y, z, _ = self.getMovingPlatformPose()
        #     if np.linalg.norm(np.array([x,y,z]) - np.array([x0,y0,z0]), ord=2) > 0.003:
        #         self.setMotorVelo(0, 0, 0, 0)
        #         break
        #     else:
        #         time.sleep(0.1)
        # #print('cable1 pretightened') 
        # time.sleep(0.5)

        # # cable4 pre-tightening
        # print('cable4 pretightening...')
        # self.setMotorVelo(0, 0, 0, -200)
        # x0, y0, z0, _ = self.getMovingPlatformPose()
        # while True:
        #     x, y, z, _ = self.getMovingPlatformPose()
        #     if np.linalg.norm(np.array([x,y,z]) - np.array([x0,y0,z0]), ord=2) > 0.003:
        #         self.setMotorVelo(0, 0, 0, 0)
        #         break
        #     else:
        #         time.sleep(0.1)
        # #print('cable1 pretightened') 
        # time.sleep(0.5)
        

    def loosen(self):
        print('loosening...')
        print(self.len00)
        print(self.len01)
        print(self.delta0)
        print(self.len10)
        print(self.len11)
        print(self.delta1)
        velo0=self.delta0/3
        velo1=self.delta1/3
        self.setMotorVelo(-int(velo0*60/(0.051*math.pi)*10), -int(velo1*60/(0.051*math.pi)*10))
        time.sleep(3)
        self.setMotorVelo(0, 0)
        time.sleep(0.5)
        

    def getAnchorAPoses(self):
        return [self._anchorA1Pos, self._anchorA2Pos, self._anchorA3Pos,self._anchorA4Pos]


    def getAnchorBPoses(self):
        return [self._anchorB1Pos, self._anchorB2Pos, self._anchorB3Pos,self._anchorB4Pos]


if __name__=="__main__":
    cdpr = CDPR()
    rate = rospy.Rate(2)
    
    # cdpr.setMotorVelo(0, 0, 1000, 1000)
    # cdpr.setMotorVelo(0, 0, 0, 0)
    
    # cnt = 0
    # while cnt < 20:
    #     print(cdpr.getMotorPos())
    #     cnt += 1
    #     rate.sleep()

    # cdpr.setMotorVelo(0, 0, 0, 0)

    
    cdpr.pretighten()
    cdpr.loosen()

    # cdpr.setMotorVelo(0, 0, -300, 0)
    # time.sleep(1)

    #print(cdpr.getMotorPos())
    cdpr.setMotorVelo(0, 0)


    # [-3845038 82537040 6610596 201052]