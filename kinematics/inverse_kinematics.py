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
from numpy.matlib import identity

from numpy import sin, cos, pi, matrix, random, linalg, asarray
from scipy.linalg import pinv
from math import atan2

def from_trans(m):
    if m[0, 0] == 1:
        return [m[3, 0], m[3, 1], m[3, 2], atan2(m[2, 1], m[1, 1])]
    elif m[1, 1] == 1:
        return [m[3, 0], m[3, 1], m[3, 2], atan2(m[0, 2], m[0, 0])]
    elif m[2, 2] == 1:
        return [m[3, 0], m[3, 1], m[3, 2], atan2(m[1, 0], m[0, 0])]
    else:
        return [m[3, 0], m[3, 1], m[3, 2], 0]


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def fKinematics(self, joints):
        transform = []
        T = identity(4)
        for joint in joints:
            angle = joints[joint]
            Tl = self.local_trans(joint, angle)
            T = T * Tl
            self.transforms[joint] = T
            transform.append(T)
        return transform

    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = []
        # YOUR CODE HERE
        theta = random.random(len(self.chains[effector_name])) * 1e5
        lambda_ = 1
        max_step = 0.1
        angles = {}
        for joint in (self.chains[effector_name]):
            angles[joint] = random.random() * 1e5
        target = matrix([from_trans(transform)]).T
        for i in range(1000):
            Ts = self.fKinematics(angles)
            Te = matrix([from_trans(Ts[-1])]).T
            e = target - Te
            e[e > max_step] = max_step
            e[e < -max_step] = -max_step
            T = matrix([from_trans(i) for i in Ts[0:]]).T
            J = Te - T
            dT = Te - T
            J[0, :] = -dT[1, :]
            J[1, :] = dT[0, :]
            J[-1, :] = 1
            d_theta = lambda_ * pinv(J) * e
            theta += asarray(d_theta.T)[0]
            if  linalg.norm(d_theta) < 1e-4:
                break
        
        joint_angles=theta

        return joint_angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.keyframes = ([], [], [])  # the result joint angles have to fill in
        angles = self.inverse_kinematics(effector_name, transform)
        for i, joint in enumerate(self.chains[effector_name]):
            self.keyframes[0].append(joint)
            self.keyframes[1].append([0., 5.])
            self.keyframes[2].append([[self.perception.joint[joint], [5., 0., 0.], [5., 0., 0.]], [angles[i], [5., 0., 0.], [5., 0., 0.]]])
           
if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = 0.26
    agent.set_transforms('LLeg', T)
    agent.run()
