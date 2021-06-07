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
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from inverse_kinematics import InverseKinematicsAgent
from recognize_posture import PostureRecognitionAgent

import xmlrpclib
from SimpleXMLRPCServer import SimpleXMLRPCServer
import time
import numpy

class ServerAgent(InverseKinematicsAgent, PostureRecognitionAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE
    def __init__(self):
        super(ServerAgent, self).__init__()
        server = SimpleXMLRPCServer(("localhost", 8000))
        print("Listening on port 8000...")
        server.register_function(self.get_angle, "get_angle")
        server.register_function(self.set_angle, "set_angle")
        server.register_function(self.get_posture, "get_posture")
        server.register_function(self.execute_keyframes, "execute_keyframes")
        server.register_function(self.get_transform, "get_transform")
        server.register_function(self.set_transform, "set_transform")
        server.serve_forever()
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        return self.perception.joint[joint_name]
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        self.perception.joint[joint_name]=angle
        return ('angle set')

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.recognize_posture(self.perception)

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        self.keyframes=keyframes
        time.sleep(max([item for sublist in keyframes[1] for item in sublist]))
        return ('keyframes executed')

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        self.forward_kinematics(self.perception.joint)
        return self.transforms[name].tolist()

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.set_transforms(effector_name, numpy.matrix(transform))
        return ('transform set')

if __name__ == '__main__':
    agent = ServerAgent()
    agent.run()

