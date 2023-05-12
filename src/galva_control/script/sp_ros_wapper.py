#!/usr/bin/env python3

import rospy
import threading
from geometry_msgs.msg import Point
from SlidePlatform import *


rospy.init_node('sp_ros_node', anonymous=True)
pub_uav_pose = rospy.Publisher('/uav/pose', Point, queue_size=1)
# client1 = MyModbusClient(1, serial_port1, 1, kp=1.5, kd=1.0)
# client2 = MyModbusClient(2, serial_port2, 1, direction = -1)
client3 = MyModbusClient(3, serial_port3, 1, direction = -1, kp=1.5)
client4 = MyModbusClient(4, serial_port4, 1)

# slide_platform_down = SlidePlatform(client1, client2)
slide_platform_up   = SlidePlatform(client3, client4)

# execute_thread1 = threading.Thread(target=slide_platform_down.circle, args=(1,30))
execute_thread2 = threading.Thread(target=slide_platform_up.circle, args=(1,30))

# slide_platform_down.enable_motor()
slide_platform_up.enable_motor()

# execute_thread1.start()
execute_thread2.start()

rate = rospy.Rate(20)

# while execute_thread1.is_alive() or execute_thread2.is_alive():
while execute_thread2.is_alive():
    position = slide_platform_up.get_position()
    uav_point = Point(position[0]+0.25, position[1]-0.55, 0.8)
    pub_uav_pose.publish(uav_point)
    rate.sleep()
    
# slide_platform_down.disable_motor()
slide_platform_up.disable_motor()