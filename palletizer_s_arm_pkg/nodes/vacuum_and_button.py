#!/usr/bin/env python
import rospy
from std_srvs.srv import SetBool
from std_msgs.msg import Bool
import serial
import thread

def read_button():
    global port
    pub = rospy.Publisher('/stop_robot', Bool, queue_size=10)
    robot_stop = Bool()
    robot_stop.data = False
    while True:
        msg = port.readline()
        if msg == 'u':
            robot_stop.data = False
        if msg == 'd':
            robot_stop.data = True
        pub.publish(robot_stop)
        rospy.sleep(0.01)


def set_vacuum(msg):
    if(msg.data):
        port.write(b'2')
    else:
        port.write(b'1')
    return True,'success'

if __name__=="__main__":
    global port
    port = serial.Serial("/dev/ttyS1", baudrate=115200)
    rospy.init_node('vacuum_and_button')
    thread.start_new_thread(read_button,())
    rospy.Service('/vacuum',SetBool,set_vacuum)
    rospy.spin()
