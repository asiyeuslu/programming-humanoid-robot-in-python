'''In this file you need to implement remote procedure call (RPC) client

* The agent_server.py has to be implemented first (at least one function is implemented and exported)
* Please implement functions in ClientAgent first, which should request remote call directly
* The PostHandler can be implement in the last step, it provides non-blocking functions, e.g. agent.post.execute_keyframes
 * Hints: [threading](https://docs.python.org/2/library/threading.html) may be needed for monitoring if the task is done
'''

import weakref
import xmlrpclib
import pickle

class PostHandler(object):
    '''the post hander wraps function to be excuted in paralle
    '''
    def __init__(self, obj):
        self.proxy = weakref.proxy(obj)

    def execute_keyframes(self, keyframes):
        '''non-blocking call of ClientAgent.execute_keyframes'''
        # YOUR CODE HERE

    def set_transform(self, effector_name, transform):
        '''non-blocking call of ClientAgent.set_transform'''
        # YOUR CODE HERE


class ClientAgent(object):
    '''ClientAgent request RPC service from remote server
    '''
    # YOUR CODE HERE
    def __init__(self):
        self.post = PostHandler(self)
        self.server = xmlrpclib.ServerProxy('http://localhost:9009')
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        print(self.server.get_angle(joint_name))
        return self.server.get_angle(joint_name)
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        print(self.server.set_angle(joint_name, str(angle)))

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        print(pickle.loads(self.server.get_posture()))
        return pickle.loads(self.server.get_posture())

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        key = pickle.dumps(keyframes)
        print(self.server.execute_keyframes(key))

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        print(pickle.loads(self.server.get_transform(name)))
        return pickle.loads(self.server.get_transform(name))

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        trans = pickle.dumps(transform)
        print(self.server.set_transform(effector_name, trans))

if __name__ == '__main__':
    agent = ClientAgent()
    # TEST CODE HERE
    agent.set_angle('LShoulderPitch', 1.56)


