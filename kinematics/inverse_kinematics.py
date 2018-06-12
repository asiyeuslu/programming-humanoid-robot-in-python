'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity, matrix, linalg
from numpy import random, asarray, zeros, pi, dot, arccos, arcsin, cos, sin, eye
from numpy.linalg import inv
from math import atan2


class InverseKinematicsAgent(ForwardKinematicsAgent):
    
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = []
        # YOUR CODE HERE
        #analytical method from 'B-Human Code Release 2017'
        if (effector_name[-3:] == 'Leg'):
            #45 degree rotation of hip and length from torso to hip
            if effector_name[0] == 'L':
                foot_to_hip_orthogonal = dot(self.local_trans('LRoll', pi/4), transform)
            else:
                foot_to_hip_orthogonal = dot(self.local_trans('RRoll', -pi/4), transform)
            hip_orthogonal_to_foot = inv(foot_to_hip_orthogonal)
            angle_knee = self.cosinus_formula(linalg.norm(self.lengths['LegHipPitchKneePitch']), linalg.norm(self.lengths['LegKneePitchAnklePitch']), linalg.norm(self.from_trans(hip_orthogonal_to_foot)))
            #angle of KneePitch
            angle_knee = pi - angle_knee
            angle_anklepitch_1 = self.cosinus_formula(linalg.norm(self.lengths['LegKneePitchAnklePitch']), linalg.norm(self.from_trans(hip_orthogonal_to_foot)), linalg.norm(self.lengths['LegHipPitchKneePitch']))
            #matrix([X,Y,Z]) of hip_orthogonal_to_foot
            m = self.from_trans(hip_orthogonal_to_foot)
            angle_anklepitch_2 = atan2(m[0,0],linalg.norm(m[0,1:]))
            #angle of AnklePitch
            angle_anklepitch = angle_anklepitch_1 + angle_anklepitch_2
            #angle of AnkleRoll
            angle_ankleroll = atan2(m[0,1], m[0,2])
            thigh_to_foot = dot(self.local_trans('LAnklePitch', angle_anklepitch), self.local_trans('LKneePitch', angle_knee))
            thigh_to_foot = dot(self.local_trans('Roll', angle_ankleroll),thigh_to_foot)
            hip_orthogonal_to_thigh = dot(inv(thigh_to_foot),hip_orthogonal_to_foot)
            #angle of HipRoll, compensate rotation in first step by pi/4
            if effector_name[0] == 'L':
                angle_hiproll = arcsin(hip_orthogonal_to_thigh[2,1]) - pi/4
            else:
                angle_hiproll = arcsin(hip_orthogonal_to_thigh[2,1]) + pi/4
            #angle of HipYawPitch
            angle_hipyaw = atan2(-hip_orthogonal_to_thigh[0,1], hip_orthogonal_to_thigh[1,1])
            #angle of HipPitch
            angle_hippitch = atan2(-hip_orthogonal_to_thigh[2,0], hip_orthogonal_to_thigh[2,2])
            joint_angles = [angle_hipyaw, angle_hiproll, angle_hippitch, angle_knee, angle_anklepitch, angle_ankleroll]
            
        #analytical method from 'Complete Analytical Inverse for NAO' by Kofinas,N. et al. and 'Forward and Inverse Kinematics for the NAO Humanoid Robot' PhD Thesis by Kofinas,N.
        elif (effector_name[-3:] == 'Arm'):
            #length from torso to shoulder for left and right arm
            if effector_name[0] == 'L':
                shoulder_to_wrist = dot(self.local_trans('LYaw', pi/2), transform)
            else:
                shoulder_to_wrist = dot(self.local_trans('RYaw', -pi/2), transform)
            m = self.from_trans(shoulder_to_wrist)
            l = self.lengths['ArmShoulderRollElbowYaw']
            print('-1..1: ', m[0,2]/l[0,0])
            if effector_name[0] == 'L':
                angle_elbow_yaw = arcsin(m[0,2] / l[0,0])
                angle_elbow_roll = -(pi - self.cosinus_formula(linalg.norm(l), (self.lengths['ArmElbowRollWristYaw'])[0,0] + linalg.norm(self.lengths['ArmElbowRollWristYaw']), linalg.norm(m)))
            else:
                angle_elbow_yaw = arcsin(m[0,2] / -l[0,0])
                angle_elbow_roll = -(pi - self.cosinus_formula(linalg.norm(l), (self.lengths['ArmElbowRollWristYaw'])[0,0] + linalg.norm(self.lengths['ArmElbowRollWristYaw']), linalg.norm(m)))
            upperarm_to_elbow = inv(dot(self.local_trans('LElbowYaw', angle_elbow_yaw), self.local_trans('LElbowRoll', angle_elbow_roll)))
            shoulder_to_upperarm = dot(inv(shoulder_to_wrist), upperarm_to_elbow)
            if effector_name[0] == 'L':
                angle_shoulder_roll = atan2(shoulder_to_upperarm[2,1], shoulder_to_upperarm[2,2]) - pi/2
            else:
                angle_shoulder_roll = atan2(shoulder_to_upperarm[2,1], shoulder_to_upperarm[2,2]) + pi/2
            angle_shoulder_pitch = atan2(shoulder_to_upperarm[1,3], shoulder_to_upperarm[3,3])
            joint_angles = [angle_shoulder_pitch, angle_shoulder_roll, angle_elbow_yaw, angle_elbow_roll]
            
        elif (effector_name == 'Head'):
            l = self.from_trans(transform)
            if (transform[0:3,0:3] == identity(3)).all():
                angle_head_yaw = l[0,2]
                angle_head_pitch = l[0,1]
            else:
                l_3 = 126.5
                b = atan2(l[0,0], l[0,1])
                angle_head_pitch = arcsin((-l[0,2] + l_3) / linalg.norm(l[0,:1])) - b + pi/2
                angle_head_yaw = arccos(l[0,0] / (l[0,2] * cos(angle_head_pitch - pi/2) + l[0,1] * sin(angle_head_pitch - pi/2)))
            joint_angles = [angle_head_yaw, angle_head_pitch]
           
        return joint_angles

    def cosinus_formula(self, a, b, c):
        print('-1, 1:' , (a**2 + b**2 - c**2)/(2 * a * b))
        return arccos((a**2 + b**2 - c**2)/(2 * a * b))
    
    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        names = []
        times = []
        keys = []
        for joint in self.chains[effector_name]:
            names.append(joint)
            times.append(1.0)
          
        times = [times]
        keys.extend(self.inverse_kinematics(effector_name, transform))
        akey = ([[3, 0.0, 0.0]]*2)
        for i in range(len(keys)):
            keys[i] = [keys[i]]
            keys[i].extend(akey)
        keys = [keys]
        for i in range(len(names)):
            times.extend(times)
            keys.extend(keys)

        self.keyframes = (names, times, keys)  # the result joint angles have to fill in

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[0,-1] = -1
    T[1,-1] = -1
    T[2,-1] = -1
    agent.set_transforms('LArm', T)
    agent.run()
