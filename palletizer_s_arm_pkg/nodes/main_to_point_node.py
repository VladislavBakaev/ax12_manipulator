#!/usr/bin/env python
import rospy  
from RoboticArmPalletizerClass import RoboticArm
from inverse_problem_srv.srv import point_cmd,point_cmdResponse
from sensor_msgs.msg import JointState
import numpy as np
from math import sqrt


class RobotControll():
    def __init__(self):
        rospy.Subscriber('/joint_states', JointState, self.jointStateCb)
        rospy.sleep(3.0)
        rospy.Service('/cmd_point', point_cmd, self.moveToPointCallback)
        self.publish_joint_state = rospy.Publisher('/cmd_joints', JointState, queue_size=10)
        self.current_joint_state = JointState()
        self.k_vel = 1.5
        
    def parseMsg(self, msg):
        try:
            coord_list = msg.point.split()
            x = float(coord_list[0])
            y = float(coord_list[1])
            z = float(coord_list[2])
            return x,y,z
        except ValueError:
            rospy.logerr('Input Error')

    def moveToPointCallback(self, msg):
        x,y,z = self.parseMsg(msg)
        roboticArm = RoboticArm()
        avail_joints_state, goal_joint_state = roboticArm.InversProblem(x,y,z, 0.0)

        if (not avail_joints_state):
            return point_cmdResponse(False)
        else:
            new_msg = JointState()
            new_msg.velocity = self.getExecutionVelocity(goal_joint_state)
            new_msg.position = goal_joint_state
            while(max(self.getCurrentError(goal_joint_state))>0.1):
                new_msg.header.stamp = rospy.Time.now()
                self.publish_joint_state.publish(new_msg)
                rospy.sleep(0.1)

            return point_cmdResponse(True)

    def jointStateCb(self, msg):
        self.current_joint_state = msg.position

    def getCurrentError(self, target):
        current = np.array(self.current_joint_state)
        error_ = np.array(target) - current
        abs_error = list(map(abs, error_.tolist()))
        return abs_error
    
    def getExecutionVelocity(self, target_position):
        way_list = self.getCurrentError(target_position)
        vel_list = []
        max_way = max(way_list)
        if not max_way == 0:
            max_vel = sqrt(max_way/self.k_vel)
            time = 3*max_way/(2*max_vel)
        else:
            time = 1

        for i,el in enumerate(way_list):
            vel_list.append(el*3/(2*time))
        
        return vel_list
        

if __name__=='__main__':
    rospy.init_node('main_to_point_node', disable_signals=True)
    robot_control = RobotControll()
    rospy.spin()
