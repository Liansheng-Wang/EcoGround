#! /usr/bin/env python3

import serial
import rospy
from geometry_msgs.msg import Point

baud_rate = 57600
serial_port = '/dev/ttyUSB0'

rospy.init_node('p9_light_node', anonymous=True)
pub_uav_pose = rospy.Publisher('/uav/pose', Point, queue_size=1)
rosrate = rospy.Rate(20)

cuavp9_ser = serial.Serial(serial_port, baud_rate, timeout=1)

while not rospy.is_shutdown():
    buffer = cuavp9_ser.readline().decode().rstrip()
    if buffer:
        data_list = buffer.split(',')
        ugv_msg = Point()
        uav_msg = Point()

        uav_msg.x = float (data_list[3])
        uav_msg.y = float (data_list[4])
        uav_msg.z = float (data_list[5])

        pub_uav_pose.publish(uav_msg)
    rosrate.sleep()