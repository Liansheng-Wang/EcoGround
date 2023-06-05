#include <ros/ros.h>
#include <serial/serial.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>


using namespace  std;

std::string serial_port = "/dev/ttyUSB0";

bool new_msg = false;
geometry_msgs::PoseStamped g_uav_pose;
void uavPose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
  g_uav_pose = *msg;
  new_msg = true;
}

int main(int argc, char** argv){
  ros::init(argc, argv ,"p9_ground_node");
  ros::NodeHandle nh;
  serial::Serial serial(serial_port, 57600, serial::Timeout::simpleTimeout(1000));
  ros::Subscriber sub_uav_pose = nh.subscribe<geometry_msgs::PoseStamped>(
                                  "/mavros/local_position/pose", 
                                  1, &uavPose_cb);
  ros::Rate loop(30);
  while(ros::ok()){
    ros::spinOnce();
    if (new_msg){
      std::string ser_msg;
      ser_msg = std::to_string(g_uav_pose.pose.position.x) + "," +
                std::to_string(g_uav_pose.pose.position.y) + "," +
                std::to_string(g_uav_pose.pose.position.z) + "\n";
      if (serial.isOpen()) {
        serial.write(ser_msg);
      } else {
        ROS_ERROR("Failed to open serial port.");
      }
      new_msg = false;
    }
    loop.sleep();
  }

  return 0;
}