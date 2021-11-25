from math import *
import numpy as np

class RoboticArm:
    def __init__(self):
        self.__l1 = 68
        self.__l2 = 80
        self.__l3 = 120
        self.__l4 = 57.94
        self.__l5 = 70

        self.__joint_1_l = -2.6
        self.__joint_1_u = 2.6
        self.__joint_2_l = 0
        self.__joint_2_u = 2.61
        self.__joint_3_l = 0.57-pi
        self.__joint_3_u = 5.23-pi
        self.__joint_4_l = -1.57
        self.__joint_4_u = 1.57
        self.__joint_5_l = -2.60
        self.__joint_5_u = 2.60

        self.__convertionAngleFor1 = 0
        self.__convertionAngleFor2 = 0
        self.__convertionAngleFor3 = -pi
        self.__convertionAngleFor4 = 0
        self.__convertionAngleFor5 = 0

    def DirectProblem(self,q1,q2,q3,q4,q5):

        l1 = self.__l1
        l2 = self.__l2
        l3 = self.__l3
        l4 = self.__l4
        l5 = self.__l5
        q1 = q1 - self.__convertionAngleFor1
        q2 = q2 - self.__convertionAngleFor2
        q3 = q3 - self.__convertionAngleFor3
        q4 = -q4 - self.__convertionAngleFor4
        q5 = q5 - self.__convertionAngleFor5

        # a1 = np.array([[cos(q1),0,sin(q1),0],
        #                 [sin(q1),0,-cos(q1),0],
        #                 [0,1,0,l1],
        #                 [0,0,0,1]])
        # a2 = np.array([[cos(q2),-sin(q2),0,l2*cos(q2)],
        #                 [sin(q2),cos(q2),0,l2*sin(q2)],
        #                 [0,0,1,0],
        #                 [0,0,0,1]])
        # a3 = np.array([[cos(q3),sin(q3),0,-l3*cos(q3)],
        #                 [sin(q3),-cos(q3),0,-l3*sin(q3)],
        #                 [0,0,-1,0],
        #                 [0,0,0,1]])
        # a4 = np.array([[cos(q4),0,sin(q4),0],
        #                 [sin(q4),0,-cos(q4),0],
        #                 [0,1,0,0],
        #                 [0,0,0,1]])
        # a5 = np.array([[cos(q5),-sin(q5),0,0],
        #                 [sin(q5),cos(q5),0,0],
        #                 [0,0,1,l4+l5],
        #                 [0,0,0,1]])

        # a1a2 = np.dot(a1,a2)
        # a1a2a3 = np.dot(a1a2,a3)
        # a1a2a3a4 = np.dot(a1a2a3,a4)
        # tT = np.dot(a1a2a3a4,a5)

        tT = np.array([[cos(q5)*(cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) + sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2))) - sin(q1)*sin(q5), -cos(q5)*sin(q1) - sin(q5)*(cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) + sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2))), -cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)), -cos(q1)*cos(q2)*cos(q3)*l3 + cos(q1)*cos(q2)*l2 + cos(q1)*l3*sin(q2)*sin(q3) + (l4 + l5)*(-cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)))],
                       [cos(q1)*sin(q5) + cos(q5)*(cos(q4)*(cos(q2)*cos(q3)*sin(q1) - sin(q1)*sin(q2)*sin(q3)) + sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))), cos(q1)*cos(q5) - sin(q5)*(cos(q4)*(cos(q2)*cos(q3)*sin(q1) - sin(q1)*sin(q2)*sin(q3)) + sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))), -cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3)*sin(q1) - sin(q1)*sin(q2)*sin(q3)), -cos(q2)*cos(q3)*l3*sin(q1) + cos(q2)*l2*sin(q1) + l3*sin(q1)*sin(q2)*sin(q3) + (l4 + l5)*(-cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3)*sin(q1) - sin(q1)*sin(q2)*sin(q3)))],
                       [cos(q5)*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(-cos(q2)*cos(q3) + sin(q2)*sin(q3))), -sin(q5)*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(-cos(q2)*cos(q3) + sin(q2)*sin(q3))), -cos(q4)*(-cos(q2)*cos(q3) + sin(q2)*sin(q3)) + sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)), -cos(q2)*l3*sin(q3) - cos(q3)*l3*sin(q2) + l1 + l2*sin(q2) + (l4 + l5)*(-cos(q4)*(-cos(q2)*cos(q3) + sin(q2)*sin(q3)) + sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)))],
                       [0, 0, 0, 1]])

        return (np.dot(tT,np.array([[0],[0],[0],[1]])))

    def __ConvertionToOurSK(self,q1,q2_1,q2_2,q3_1,q3_2,q4_1,q4_2,q5):
        q1 = round(q1 + self.__convertionAngleFor1,2)
        q2_1 = round(q2_1 + self.__convertionAngleFor2,2)
        q2_2 = round(q2_2 + self.__convertionAngleFor2,2)
        q3_1 = round(q3_1 + self.__convertionAngleFor3,2)
        q3_2 = round(q3_2 + self.__convertionAngleFor3,2)
        q4_1 = round(q4_1 + self.__convertionAngleFor4,2)
        q4_2 = round(q4_2 + self.__convertionAngleFor4,2)
        q5 = round(q5 + self.__convertionAngleFor5,2)
        return q1,q2_1,q2_2,q3_1,q3_2,q4_1,q4_2,q5

    def __VadatingOfJointAngle(self,q1,q2_1,q2_2,q3_1,q3_2,q4_1,q4_2,q5):
        countAvailJointState = 0
        if (q1 >= self.__joint_1_l or q1 <= self.__joint_1_u):
            countAvailJointState += 1
        if (self.__Validatingq2q3q4(q2_1,q3_1,q4_1)):
            q2 = q2_1
            q3 = q3_1
            q4 = q4_1
            countAvailJointState += 3
        if(countAvailJointState < 2):
            if(self.__Validatingq2q3q4(q2_2,q3_2,q4_2)):
                q2 = q2_2
                q3 = q3_2
                q4 = q4_2
                countAvailJointState += 3

        if (q5 >= self.__joint_5_l and q5 <= self.__joint_5_u):
            countAvailJointState +=1

        if (countAvailJointState == 5):
            availConfig = True
            return availConfig,q1,q2,q3,q4,q5
        else:
            availConfig = False
            return availConfig,0,0,0,0,0

    def __Validatingq2q3q4(self,q2,q3,q4):
        if (q2 >= self.__joint_2_l and q2 <= self.__joint_2_u):
            if (q3 >= self.__joint_3_l and q3 <= self.__joint_3_u):
                if (q4 >= self.__joint_4_l and q4 <= self.__joint_4_u):
                    return True
        return False

    def InversProblem(self,x,y,z,pitch = 0,roll = 0):
        q5 = roll
        
        q1 = -atan2(y,x)

        w = sqrt(x**2+y**2)

        z1 = (z - self.__l1) - (self.__l4 + self.__l5)*sin(pitch)
        w1 = w - (self.__l4 + self.__l5)*cos(pitch)

        c = (self.__l3**2-w1**2-z1**2-self.__l2**2)/(-2)
        a = z1**2 + w1**2
        b = -2*z1*c
        e = c**2 - self.__l2**2*w1**2

        D = b**2 - 4*a*e

        if (D < 0 ):
            return False,[0,0,0,0,0]

        z2_1 = (-b + sqrt(D))/(2*a)
        z2_2 = (-b - sqrt(D))/(2*a)

        w2_1 = (c-z2_1*z1)/w1
        w2_2 = (c-z2_2*z1)/w1

        q2_1 = atan2(z2_1,w2_1)
        q2_2 = atan2(z2_2,w2_2)

        joint3Angle_1 = atan2(z1 - z2_1,w1-w2_1)
        joint3Angle_2 = atan2(z1 - z2_2,w1-w2_2)

        q3_1 = pi - q2_1 + joint3Angle_1
        q3_2 = pi - q2_2 + joint3Angle_2

        q4_1 = -(joint3Angle_1 - pitch)
        q4_2 = -(joint3Angle_2 - pitch)

        q1,q2_1,q2_2,q3_1,q3_2,q4_1,q4_2,q5 = self.__ConvertionToOurSK(q1,q2_1,q2_2,q3_1,q3_2,q4_1,q4_2,q5)
        availConfig,q1,q2,q3,q4,q5 = self.__VadatingOfJointAngle(q1,q2_1,q2_2,q3_1,q3_2,q4_1,q4_2,q5)

        return availConfig,[q1,-q2,-q3,-q4,q5]

# robot = RoboticArm()
# print(robot.InversProblem(200,200,50))
