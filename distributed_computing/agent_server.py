'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import os
import sys
from SimpleXMLRPCServer import SimpleXMLRPCServer
import pickle
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from inverse_kinematics import InverseKinematicsAgent
from recognize_posture import PostureRecognitionAgent


class ServerAgent(InverseKinematicsAgent, PostureRecognitionAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        return self.perception.joint[joint_name]
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        self.perception.joint[joint_name] = float(angle)
        return "angle of " + joint_name + " was set to " + angle

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return pickle.dumps(self.recognize_posture(self.perception))

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        self.keyframes = pickle.loads(keyframes)
        return "robot should do something"

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        return pickle.dumps(self.local_trans(name, self.perception.joint[name]))

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        trans = pickle.loads(transform)
        self.set_transforms(effector_name, trans)
        return effector_name + " should move"
        
server = SimpleXMLRPCServer(('localhost', 9009))
print('Server listening on port 9009...')
server.register_introspection_functions()
#    server.register_function(get_angle, 'get_angle')
#    server.register_function(set_angle, 'set_angle')
#    server.register_function(get_posture, 'get_posture')
#    server.register_function(execute_keyframes, 'execute_keyframes')
#    server.register_function(get_transform, 'get_transform')
#    server.register_function(set_transform, 'set_transform')
server.register_instance(ServerAgent())
server.serve_forever()

if __name__ == '__main__':
    agent = ServerAgent()
    agent.run()

