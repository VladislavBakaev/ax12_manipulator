from math import *
import numpy as np

class RoboticArm:
    def __init__(self):
        self.__l1 = 70
        self.__l2 = 0
        self.__l3 = 140
        self.__l4 = 140
        self.__l5 = 50
        self.__l6 = 50
        self.__l7 = 6.0

    def InversProblem(self,X,Y,Z,pitch = 0):
        l1 = self.__l1
        l2 = self.__l2
        l3 = self.__l3
        l4 = self.__l4
        l5 = self.__l5
        l6 = self.__l6
        l7 = self.__l7
        try:
            alpha_temp = atan2(Y,X)
            tetta = asin(l7/sqrt(X**2+Y**2))
            alpha1 = alpha_temp+tetta
            l = sqrt(X**2+Y**2-l7**2)

            X = l*cos(alpha1)
            Y = l*sin(alpha1)

            z = Z+l6-l1

            x = X/cos(alpha1)
            x = x - l2 - l5 

            d = sqrt(x*x+z*z)
            gamma = acos((l3*l3+d*d-l4*l4)/(2*l3*d))
            beta = gamma + atan(z/x)
            alpha2 = pi/2 - beta

            gamma1 = acos((l3*l3+l4*l4-d*d)/(2*l3*l4))
            
            alpha3 = gamma1 - alpha2

            q = (alpha1,alpha2,-(alpha3-pi/2),pitch)

            q_max = max(list(map(abs, q)))
            if q_max > 2.6:
                raise Exception("Constrain")
            return True,q

        except Exception as e:
            print (e)
            return False, (0,0,0,0)

# robot = RoboticArm()
# print(robot.InversProblem(150,0,50))
