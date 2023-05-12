#!/usr/bin/env python3

import time
import serial
import rospy
from geometry_msgs.msg import Point


baud_rate = 57600
serial_port = '/dev/ttyUSB1'
cuavp9_ser = serial.Serial(serial_port, baud_rate, timeout=1)


ugv_msg: Point
def ugv_callback(msg):
    Point = msg
    p9_msg = str(Point.x)  + "," + str(Point.y) + ","+ str(Point.z)+"\n"
    cuavp9_ser.write(p9_msg)


rospy.init_node('p9_ground_node', anonymous=True)
sub_ugv_pose = rospy.Subscriber('/ugv/pose', Point, ugv_callback)

# 本地测试代码
# rosrate = rospy.Rate(20)
# while not rospy.is_shutdown():
#     p9_msg = str(3.1415926)  + "," + str(1.41421356) + ","+ str(2.71828)+"\n"
#     n = cuavp9_ser.write(p9_msg.encode('utf-8'))
#     if n != len(p9_msg):
#         print("Failed to send data!")
#     rosrate.sleep()

rospy.spin()