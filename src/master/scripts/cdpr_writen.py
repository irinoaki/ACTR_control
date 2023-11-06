#! /usr/bin/env python3

#It's the version for the position-controlled motors

import rospy
import time
import numpy as np

from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Int16MultiArray, Int64MultiArray

from slave.srv import SetMotorVelo, GetMotorPos

from transform import quaternion2euler


class CDPR:
    def __init__(self):
        # ros settings
        rospy.init_node('cdpr_control', anonymous=True)

        # origin point offset (coordinates in world frame)
        self.xOff = 0.423
        self.yOff = 0.439
        self.zOff = -0.612

        # subscriber and publisher
        self._movingPlatformPose = PoseStamped()
        rospy.Subscriber('/vrpn_client_node/end_effector/pose', PoseStamped, self._poseCallback, queue_size=1)

        self._motor_pos = np.array([0, 0, 0, 0])
        rospy.Subscriber('motor_pos', Int64MultiArray, self._motorPosCallback, queue_size=1)

        self._veloPub = rospy.Publisher('motor_velo', Int16MultiArray, queue_size=10)

        # anchor positions in the world frame
        self._anchorA1Pos = np.array([0.759, 0.778, -0.229])
        self._anchorA2Pos = np.array([0.082, 0.786, -0.230])
        self._anchorA3Pos = np.array([0.080, 0.097, -0.224])
        self._anchorA4Pos = np.array([0.769, 0.103, -0.226])

        # anchors on the moving platform (body frame)
        self._anchorB1Pos = np.array([0, 0, 0])
        self._anchorB2Pos = np.array([0, 0, 0])
        self._anchorB3Pos = np.array([0, 0, 0])
        self._anchorB4Pos = np.array([0, 0, 0])

        # initial cable lengths and motor positions
        self._cable1_length0 = 0
        self._cable2_length0 = 0
        self._cable3_length0 = 0
        self._cable4_length0 = 0
        self._cable_length0 = np.array([0, 0, 0 ,0])
        self._motor_pos0 = np.array([0, 0, 0 ,0])

    
    def init_cable_length(self, cable1_flag, cable2_flag, cable3_flag, cable4_flag):
        # calculate origin cable lengths
        time.sleep(1)
        x0, y0, z0, _ = self.getMovingPlatformPose()
        pos0 = np.array([x0, y0, z0])
        motor_pos0 = self._motor_pos
        if cable1_flag:
            self._cable1_length0 = np.linalg.norm(pos0 - self._anchorA1Pos)
            self._motor_pos0[0] = motor_pos0[0]
        if cable2_flag:
            self._cable2_length0 = np.linalg.norm(pos0 - self._anchorA2Pos)
            self._motor_pos0[1] = motor_pos0[1]
        if cable3_flag:
            self._cable3_length0 = np.linalg.norm(pos0 - self._anchorA3Pos)
            self._motor_pos0[2] = motor_pos0[2]
        if cable4_flag:
            self._cable4_length0 = np.linalg.norm(pos0 - self._anchorA4Pos)
            self._motor_pos0[3] = motor_pos0[3]
        self._cable_length0 = np.array([self._cable1_length0, self._cable2_length0, self._cable3_length0, self._cable4_length0])


    def _poseCallback(self, data):
        # if motion data is lost(999999), do not update
        if np.abs(data.pose.position.x) > 2000 or np.abs(data.pose.position.y) > 2000 or np.abs(data.pose.position.z) > 2000:
            pass
        else:
            # pose
            self._movingPlatformPose.pose.position.x = data.pose.position.x / 1000
            self._movingPlatformPose.pose.position.y = data.pose.position.y / 1000
            self._movingPlatformPose.pose.position.z = data.pose.position.z / 1000
            self._movingPlatformPose.pose.orientation = data.pose.orientation

            # header
            self._movingPlatformPose.header.frame_id = data.header.frame_id
            self._movingPlatformPose.header.stamp = data.header.stamp


    def _motorPosCallback(self, data):
        diff = np.array(data.data) - self._motor_pos
        if (np.abs(diff[0]) > 100000 or np.abs(diff[1]) > 1000000 or np.abs(diff[2]) > 1000000 or np.abs(diff[3]) > 1000000) \
            and (self._motor_pos[0] != 0 and self._motor_pos[1] != 0 and self._motor_pos[2] != 0 and self._motor_pos[3] != 0):
            pass
        else:
            self._motor_pos = np.array(data.data)#data.data是什么


    def setMotorVelo(self, motor1Velo, motor2Velo, motor3Velo, motor4Velo):
        # rospy.wait_for_service('set_motor_velo')
        # try:
        #     set_motor_velo = rospy.ServiceProxy('set_motor_velo', SetMotorVelo)
        #     set_motor_velo(motor1Velo, motor2Velo, motor3Velo)
        # except rospy.ServiceException as e:
        #     print("Service call failed: {}".format(e))
        motor_velo = Int16MultiArray(data=np.array([motor1Velo, motor2Velo, motor3Velo, motor4Velo]))
        self._veloPub.publish(motor_velo)

    
    ############# to be updated ############
    def getMotorPos(self):
        rospy.wait_for_service('set_motor_velo')
        try:
            get_motor_pos = rospy.ServiceProxy('get_motor_pos', GetMotorPos)
            # print('1: {}'.format(time.time()))
            response = get_motor_pos()
            print(response.motor1Pos, response.motor2Pos, response.motor3Pos)
            return [response.motor1Pos, response.motor2Pos, response.motor3Pos]
            # print('2: {}'.format(time.time()))
        except rospy.ServiceException as e:
            print("Service call failed: {}".format(e))
    ############# to be updated ############


    def getMovingPlatformPose(self):
        return self._movingPlatformPose.pose.position.x, self._movingPlatformPose.pose.position.y, self._movingPlatformPose.pose.position.z,\
            [self._movingPlatformPose.pose.orientation.x, self._movingPlatformPose.pose.orientation.y, self._movingPlatformPose.pose.orientation.z, self._movingPlatformPose.pose.orientation.w]


    def get_cable_length(self):
        r = 0.03 * np.pi / 10000 / 10
        cable_length = (self._motor_pos - self._motor_pos0) * r + self._cable_length0
        return cable_length
    

    def pretighten(self, cable1_flag, cable2_flag, cable3_flag, cable4_flag):

        if cable1_flag:
            time.sleep(0.5)
            # cable1 pre-tightening
            print('cable1 pretightening...')
            self.setMotorVelo(-100, 0, 0, 0)
            x0, y0, z0, _ = self.getMovingPlatformPose()
            while True:
                x, y, z, _ = self.getMovingPlatformPose()
                if np.linalg.norm(np.array([x, y, z]) - np.array([x0, y0, z0]), ord=2) > 0.005:
                    self.setMotorVelo(0, 0, 0, 0)
                    break
                else:
                    time.sleep(0.1)

        if cable2_flag:
            time.sleep(0.5)
            # cable2 pre-tightening
            print('cable2 pretightening...')
            self.setMotorVelo(0, -100, 0, 0)
            x0, y0, z0, _ = self.getMovingPlatformPose()
            while True:
                x, y, z, _ = self.getMovingPlatformPose()
                if np.linalg.norm(np.array([x, y, z]) - np.array([x0, y0, z0]), ord=2) > 0.005:
                    self.setMotorVelo(0, 0, 0, 0)
                    break
                else:
                    time.sleep(0.1)

        if cable3_flag:
            time.sleep(0.5)
            # cable3 pre-tightening
            print('cable3 pretightening...')
            self.setMotorVelo(0, 0, -100, 0)
            x0, y0, z0, _ = self.getMovingPlatformPose()
            while True:
                x, y, z, _ = self.getMovingPlatformPose()
                if np.linalg.norm(np.array([x, y, z]) - np.array([x0, y0, z0]), ord=2) > 0.005:
                    self.setMotorVelo(0, 0, 0, 0)
                    break
                else:
                    time.sleep(0.1)

        if cable4_flag:
            time.sleep(0.5)
            # cable4 pre-tightening
            print('cable4 pretightening...')
            self.setMotorVelo(0, 0, 0, -100)
            x0, y0, z0, _ = self.getMovingPlatformPose()
            while True:
                x, y, z, _ = self.getMovingPlatformPose()
                if np.linalg.norm(np.array([x, y, z]) - np.array([x0, y0, z0]), ord=2) > 0.005:
                    self.setMotorVelo(0, 0, 0, 0)
                    break
                else:
                    time.sleep(0.1)
        

    def loosen(self):
        print('loosening...')
        self.setMotorVelo(600, 600, 600)#为什么只设置三个
        time.sleep(0.2)
        self.setMotorVelo(0, 0, 0)
        time.sleep(0.5)
        

    def getAnchorAPoses(self):
        return [self._anchorA1Pos, self._anchorA2Pos, self._anchorA3Pos, self._anchorA4Pos]


    def getAnchorBPoses(self):
        return [self._anchorB1Pos, self._anchorB2Pos, self._anchorB3Pos, self._anchorB4Pos]



if __name__=="__main__":

    cdpr = CDPR()
    rate = rospy.Rate(10)
    # cdpr.pretighten()
    cdpr.init_cable_length(True, True, True, True)
    print(cdpr._cable_length0)
    time.sleep(1)
    cdpr.setMotorVelo(0, 0, 0, 0)
    start_time = time.time()
    while time.time() - start_time < 1:
        print(cdpr.get_cable_length())
        time.sleep(0.05)
    cdpr.setMotorVelo(0, 0, 0, 0)

