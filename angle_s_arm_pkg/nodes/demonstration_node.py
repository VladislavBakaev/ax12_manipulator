#!/usr/bin/env python
import rospy
from inverse_problem_srv.srv import point_cmd,point_cmdResponse
from std_srvs.srv import SetBool

srv_ang_point = rospy.ServiceProxy('/cmd_point', point_cmd)
srv_ang_end_eff = rospy.ServiceProxy('/gripper', SetBool)
rospy.wait_for_service('/cmd_point')

while True:
    srv_ang_point("170 0 80 -1.4 0")
    srv_ang_end_eff(False)
    srv_ang_point("150 150 20 -1.4 0")
    srv_ang_end_eff(True)

