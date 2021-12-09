#!/usr/bin/env python
import rospy
from std_srvs.srv import SetBool
from std_msgs.msg import Bool

pub = rospy.Publisher('/end_eff_cmd', Bool, queue_size=10)

def set_gripper(msg):
    for i in range(20):
        data = Bool()
        data.data = msg.data
        pub.publish(data)
        rospy.sleep(0.1)
    return True, ''

if __name__=="__main__":
    rospy.init_node('gripper_node')
    rospy.Service('/gripper', SetBool, set_gripper)
    rospy.spin()
