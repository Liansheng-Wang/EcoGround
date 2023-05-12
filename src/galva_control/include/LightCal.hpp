#include <vector>
#include <string>
#include <cmath>
#include <memory>
#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/transport_hints.h>
#include <geometry_msgs/Point.h>
#include <serial/serial.h>
#include <Eigen/Dense>
#include <control_cmd.h>

class LightCal{
private:
    std::vector<double> x_cmd_list_;
    std::vector<double> z_cmd_list_;
    double last_anglez_;
    Eigen::Vector2d anglez_cmd;

public:
    LightCal(){
        last_anglez_ = 0;
        anglez_cmd[0] = 0;
    }
    ~LightCal(){}
    Eigen::Vector2d GetMotorAngle(Eigen::Vector3d light_point, Eigen::Vector3d uav_point);
    int GetSpinDiection(double angle_z);
};

Eigen::Vector2d LightCal::GetMotorAngle(Eigen::Vector3d light_point,Eigen::Vector3d uav_point){

    Eigen::Vector2d motor_angle;

    //计算镜面法向量
    Eigen::Vector3d light2uavnorm = (uav_point - light_point).normalized();
    Eigen::Vector3d normal = light2uavnorm;

    Eigen::Vector3d z_normal(0,0,1);
    Eigen::Vector3d normal2xoy(normal.x(),normal.y(),0);
    normal2xoy = normal2xoy.normalized();

    double angle_x = std::acos(z_normal.dot(normal) / (z_normal.norm() * normal.norm()));
    angle_x = angle_x * 180 /M_PI;
    Eigen::Vector3d x_normal(1,0,0);

    double normal2xoy_x = normal2xoy.x();
    double normal2xoy_y = normal2xoy.y();
    double angle_z = std::atan2(normal2xoy_y,normal2xoy_x);
    angle_z = angle_z * 180 /M_PI;    //算出来的angle_z为0-180或0--180

    if(angle_z < 0 ){
        angle_z = angle_z + 360.0;
    }

    if(isnan(angle_x)){angle_x = 0;}
    if(isnan(angle_z)){angle_z = 0;}
    motor_angle[0] = angle_x; 
    motor_angle[1] = angle_z;
    
    return motor_angle;
}

int LightCal::GetSpinDiection(double angle_z){
    int spin_dircetion;
    double d_anglez = angle_z - last_anglez_;
    last_anglez_ = angle_z;
    std::cout<< "angle_z  " << angle_z << std::endl;
    std::cout<< "last_anglez_  " << last_anglez_ << std::endl;
    std::cout<< "d_anglez  " << d_anglez << std::endl;
    if(d_anglez >= 0){
        spin_dircetion = 0;
        if(d_anglez > 180){
            spin_dircetion = 1;
        }
    }
    else if(d_anglez < 0){
        spin_dircetion = 1;
        if(d_anglez < -180){
            spin_dircetion = 0;
        } 
    }
    return spin_dircetion;
}
// ============================================================
// class LightCal End


class LightControl {
private:
    ros::NodeHandle nh_;
    ros::TransportHints hints_;
    ros::Subscriber subUAVPose_;
    serial::Serial *serial1_, *serial2_;  // 1是Z，2是X 
    LightCal operators_;
    Eigen::Vector3d self_pose_, uav_pose_;
    int direction1_, direction2_;

    void uav_pose_cb(const geometry_msgs::Point::ConstPtr& msg){
        uav_pose_[0] = msg->x;
        uav_pose_[1] = msg->y;
        uav_pose_[2] = msg->z;
        auto result = operators_.GetMotorAngle(self_pose_, uav_pose_);
        setCmd(result);
    }

public:
    LightControl(ros::NodeHandle& nh, serial::Serial& serial1, serial::Serial& serial2,
    int direction1 = 1, int direction2 = 1):
    nh_(nh),
    hints_(ros::TransportHints().tcpNoDelay(true)),
    direction1_(direction1), direction2_(direction2)
    {
        serial1_ = &serial1;
        serial2_ = &serial2;
        try {
            serial1_->open();
            std::cout << "serial1_ 打开成功" << std::endl;
        }
        catch (std::exception& e) {
            std::cerr << "Error: " << e.what() << std::endl;
        }
        try {
            serial2_->open();
            std::cout << "serial2_ 打开成功" << std::endl;
        }
        catch (std::exception& e) {
            std::cerr << "Error: " << e.what() << std::endl;
        }
        self_pose_ = Eigen::Vector3d::Zero();
        subUAVPose_ = nh_.subscribe("/uav/pose", 1, &LightControl::uav_pose_cb, this, hints_);
    }

    ~LightControl(){
        if(serial1_ != nullptr){
            serial1_->close();
        }
        if(serial2_ != nullptr){
            serial2_->close();
        }
    }

    void setCmd(Eigen::Vector2d cmd){
        int spin_direction = operators_.GetSpinDiection(cmd[1]);
        PutAngleCmd_z(cmd[1] , spin_direction);
        PutAngleCmd_x(cmd[0]);
    }

    void resetZero(){
        PutAngleCmd_z(0, 1);
        PutAngleCmd_x(0);
    }

    void PutAngleCmd_z(double angle, int spin_direction){
        ControlCmd controlcmd_1(1);
        controlcmd_1.SingleLoopAngleWithSpeed(spin_direction, angle, 60);
        std::vector<uint8_t> cmd_list =  controlcmd_1.OutputCmdlist();
        int lenght = cmd_list.size();
        uint8_t msg[lenght];
        for(int i = 0; i < lenght ; i++){
            msg[i] = cmd_list[i];
        }
        // 
        serial1_->write(msg, lenght);
    }

    void PutAngleCmd_x(double angle){
        ControlCmd controlcmd_1(1);
        controlcmd_1.MultiLoopAngleWithSpeed(angle, 60);
        std::vector<uint8_t> cmd_list =  controlcmd_1.OutputCmdlist();
        int lenght = cmd_list.size();
        uint8_t msg[lenght];
        for(int i = 0; i < lenght ; i++){
            msg[i] = cmd_list[i];
        }
        serial2_->write(msg, lenght);
    }
};
