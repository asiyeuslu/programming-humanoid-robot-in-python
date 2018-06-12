'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    the local_trans has to consider different joint axes and link parameters for different joints
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from numpy.matlib import matrix, identity
from numpy import sin, cos, dot

from angle_interpolation import AngleInterpolationAgent


class ForwardKinematicsAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        self.chains = {'Head': ['HeadYaw', 'HeadPitch'],
                       'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll'],
                       'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
                       'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll'],
                       'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll']
                       }
        #lengths bodypart from.. to.. [X, Y, Z]
        self.lengths = {'ArmShoulderRollElbowYaw': matrix([105.0, 15.0, 0.0]),
                        'ArmElbowRollWristYaw': matrix([55.95, 0.0, 0.0]),
                        'LegHipPitchKneePitch': matrix([0.0, 0.0, -100.0]),
                        'LegKneePitchAnklePitch': matrix([0.0, 0.0, -102.9]),
                        'LLegTorsoHip': matrix([0.0, 50.0, -85.0]),
                        'RLegTorsoHip': matrix([0.0, -50.0, -85.0]),
                        'LArmTorsoShoulder': matrix([0.0, 98.0, 100.0]),
                        'RArmTorsoShoulder': matrix([0.0, -98.0, 100.0])
                        }
        self.start_time =self.perception.time

    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''

        # YOUR CODE HERE
        s = sin(joint_angle)
        c = cos(joint_angle)
        #set rotation 
        #Z-axis
        if (joint_name[-3:] == 'Yaw'):
            T = matrix([[c, -s, 0, 0],
                       [s, c, 0, 0],
                       [0, 0, 1, 0],
                       [0, 0, 0, 1]])
        #X-axis
        elif (joint_name[-4:] == 'Roll'):
            T = matrix([[1, 0 , 0, 0],
                       [0, c, -s, 0],
                       [0, s, c, 0],
                       [0, 0, 0, 1]])
        #Y-axis
        elif (joint_name[-5:] == 'Pitch'):
            T = matrix([[c, 0 , -s, 0],
                       [0, 1, 0, 0],
                       [s, 0, c, 0],
                       [0, 0, 0, 1]])
        #set X,Y,Z for joints that have a different coordinate difference to the joint before than 0,0,0
        if (joint_name[0] == 'L' or joint_name[0] == 'R'):
            if (joint_name == 'LElbowYaw' or joint_name == 'RElbowYaw'):
                T[:3, -1] = self.lengths['ArmShoulderRollElbowYaw'].T
            
            elif (joint_name == 'LWristYaw' or joint_name == 'RWristYaw'):
                T[:3, -1] = self.lengths['ArmElbowRollWristYaw'].T
                
            elif (joint_name == 'LKneePitch' or joint_name == 'RKneePitch'):
                T[:3, -1] = self.lengths['LegHipPitchKneePitch'].T
                
            elif (joint_name == 'LAnklePitch' or joint_name == 'RAnklePitch'):
                T[:3, -1] = self.lengths['LegKneePitchAnklePitch'].T
                #for transformation from torso to hip (left leg)
            elif (joint_name == 'LRoll'):
                T[:3, -1] = self.lengths['LLegTorsoHip'].T
                #for transformation from torso to hip (right leg)
            elif (joint_name == 'RRoll'):
                T[:3, -1] = self.lengths['RLegTorsoHip'].T
                #for transformation from torso to shoulder (left arm)
            elif (joint_name == 'LYaw'):
                T[:3, -1] = self.lengths['LArmTorsoShoulder'].T
                #for transformation from torso to shoulder (right arm)
            elif (joint_name == 'RYaw'):
                T[:3, -1] = self.lengths['RArmTorsoShoulder'].T

        return T
        
    #get X,Y,Z from transform
    def from_trans(self, transform):
        return matrix([transform[0, -1], transform[1 ,-1], transform[2,-1]])

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        for chain_joints in self.chains.values():
            T = identity(4)
            for joint in chain_joints:
                if not(joint in joints):
                    continue
                angle = joints[joint]
                Tl = self.local_trans(joint, angle)
                # YOUR CODE HERE
                T = dot(T, Tl)

                self.transforms[joint] = T

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
