#! /usr/bin/env python3

#It's only for the situation that has motors.

import rospy
import time
import numpy as np

from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Int16MultiArray

from slave.srv import SetMotorVelo, GetMotorPos

from transform import quaternion2euler


class CDPR:
    def __init__(self):
        # ros settings
        rospy.init_node('cdpr_control', anonymous=True)

        # origin point offset (coordinates in world frame)
        self.xOff = 0.039 #-0.005 + 0.2
        self.yOff = 0.035  #2.811 - 0.2
        self.zOff = -0.018  #0.280
        # self.xOff = 0.473 
        # self.yOff = 0.420
        # self.zOff = -0.490 - 0.025
        
        # subscriber and publisher
        self._movingPlatformPose = PoseStamped()
        rospy.Subscriber('/vrpn_client_node/BOX/pose', PoseStamped, self._poseCallback, queue_size=1)
        #rospy.Subscriber('/vrpn_client_node/UAV_2_new/pose', PoseStamped, self._uavCallback, queue_size=1)
        self.veloPub = rospy.Publisher('motor_velo', Int16MultiArray, queue_size=10)

        # anchor positions in the world frame
        self._anchorA1Pos = [-1.053, 0.169, 0.074]#[-0.773, 1.376, 0.073]
        self._anchorA2Pos = [0.458, 1.698, 0.087]#[1.242, 1.408, 0.054]
        # self._anchorA1Pos = [0.717, 0.770, -0.052]
        # self._anchorA2Pos = [0.699, 0.058, -0.050]
        self._anchorA3Pos = [0.064, 0.425, -0.040]
        self._anchorA4Pos = [0.064, 0.425, -0.040]

        # anchors on the moving platform (body frame)
        self._anchorB1Pos = [0, 0, 0]
        self._anchorB2Pos = [0, 0, 0]
        self._anchorB3Pos = [0, 0, 0]
        self._anchorB4Pos = [0, 0, 0]


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


    def setMotorVelo(self, motor1Velo, motor2Velo):
        # rospy.wait_for_service('set_motor_velo')
        # try:
        #     set_motor_velo = rospy.ServiceProxy('set_motor_velo', SetMotorVelo)
        #     set_motor_velo(motor1Velo, motor2Velo, motor3Velo)
        # except rospy.ServiceException as e:
        #     print("Service call failed: {}".format(e))
        motor_velo = Int16MultiArray(data=np.array([motor1Velo, motor2Velo]))
        self.veloPub.publish(motor_velo)


    def getMovingPlatformPose(self):
        return self._movingPlatformPose.pose.position.x, self._movingPlatformPose.pose.position.y, self._movingPlatformPose.pose.position.z,\
            [self._movingPlatformPose.pose.orientation.x, self._movingPlatformPose.pose.orientation.y, self._movingPlatformPose.pose.orientation.z, self._movingPlatformPose.pose.orientation.w]


    def pretighten(self):
        time.sleep(0.5)
        # cable1 pre-tightening
        print('cable1 pretightening...')
        self.setMotorVelo(100, 0)
        # self.setMotorVelo(-100, 0)
        # input('suiyi')
        # self.setMotorVelo(0,0)
        x0, y0, z0, _ = self.getMovingPlatformPose()
        while True:
            x, y, z, _ = self.getMovingPlatformPose()
            if np.linalg.norm(np.array([x, y, z]) - np.array([x0, y0, z0]), ord=2) > 0.004:
                self.setMotorVelo(0, 0)
                break
            else:
                time.sleep(0.1)

        time.sleep(0.5)
        # cable2 pre-tightening
        print('cable2 pretightening...')
        self.setMotorVelo(0, 100)
        # self.setMotorVelo(0, -100)
        # input('suiyi2')
        # self.setMotorVelo(0,0)
        x0, y0, z0, _ = self.getMovingPlatformPose()
        while True:
            x, y, z, _ = self.getMovingPlatformPose()
            if np.linalg.norm(np.array([x, y, z]) - np.array([x0, y0, z0]), ord=2) > 0.004:
                self.setMotorVelo(0, 0)
                break
            else:
                time.sleep(0.1)
        

    def loosen(self):
        print('loosening...')
        self.setMotorVelo(600, 600)
        time.sleep(0.2)
        self.setMotorVelo(0, 0)
        time.sleep(0.5)
        

    def getAnchorAPoses(self):
        return [self._anchorA1Pos, self._anchorA2Pos, self._anchorA3Pos, self._anchorA4Pos]


    def getAnchorBPoses(self):
        return [self._anchorB1Pos, self._anchorB2Pos, self._anchorB3Pos, self._anchorB4Pos]



if __name__=="__main__":

    cdpr = CDPR()
    rate = rospy.Rate(10)
    cdpr.pretighten()

