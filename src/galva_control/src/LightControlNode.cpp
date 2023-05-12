#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <vector>
#include <string>

#include <LightCal.hpp>


std::string serial_port1 = "/dev/ttyUSB4";
std::string serial_port2 = "/dev/ttyUSB5";


int main(int argc, char** argv)
{
    ros::init(argc, argv ,"lightcontrol");
    ros::NodeHandle nh;
    serial::Serial ser_1(serial_port1, 115200, serial::Timeout::simpleTimeout(1000));
    serial::Serial ser_2(serial_port2, 115200, serial::Timeout::simpleTimeout(1000));
    LightControl lightcontrol(nh, ser_1, ser_2);
    lightcontrol.resetZero();
    ros::Duration(2.0).sleep();
    ros::spin();
    return 0;
}




