#include <serial/serial.h>
#include <iostream>
#include <std_msgs/Int32.h>
#include <std_msgs/Int64.h>
#include <vector>
#include <iomanip>
#include <cstdint>
#include <string>
#include <Eigen/Eigen>
#include <Eigen/StdVector>


class ControlCmd{
private:
    std::vector<uint8_t> cmd_list_;
    int id_;
    uint8_t GetChecksum(std::vector<uint8_t> &cmd_list, int left);
    void ShowCmdlist();
public:
    ControlCmd(int id);
    void MultiLoopAngle(double angle );
    void Speed(double angle);
    void SingleLoopAngle(int spin_direction , double angle);
    void MultiLoopAngleWithSpeed(double angle,double speed);
    void SingleLoopAngleWithSpeed(int spin_direction_in ,double angle_in, double speed_in);
    std::vector<uint8_t> OutputCmdlist();
};

ControlCmd::ControlCmd(int id){
    id_ = id;
}

void ControlCmd::MultiLoopAngle(double angle_in){
    int64_t angle = std::llround(angle_in* 100);
    std::vector<uint8_t> cmd_list;
    cmd_list.push_back(0x3E);                                        //0          
    cmd_list.push_back(0xA3);                                        //1
    cmd_list.push_back(static_cast<uint8_t>(id_));                    //2   
    cmd_list.push_back(0x08);                                        //3
    uint8_t checksum_head = GetChecksum(cmd_list,0);                       
    cmd_list.push_back(checksum_head);                               //4
    cmd_list.push_back(*(uint8_t *)(&angle));                        //5
    cmd_list.push_back(*((uint8_t *)(&angle)+1));                      //6 
    cmd_list.push_back(*((uint8_t *)(&angle)+2));                      //7  
    cmd_list.push_back(*((uint8_t *)(&angle)+3));                      //8 
    cmd_list.push_back(*((uint8_t *)(&angle)+4));                      //9 
    cmd_list.push_back(*((uint8_t *)(&angle)+5));                      //10 
    cmd_list.push_back(*((uint8_t *)(&angle)+6));                      //11
    cmd_list.push_back(*((uint8_t *)(&angle)+7));                      //12
    uint8_t checksum_data = GetChecksum(cmd_list,5);
    cmd_list.push_back(checksum_data);                               //13                        
    cmd_list_ = cmd_list;
}

void ControlCmd::MultiLoopAngleWithSpeed(double angle_in, double speed_in){
    int64_t angle = std::llround(angle_in* 100);
    int32_t speed = std::llround(speed_in*100);
    std::vector<uint8_t> cmd_list;

    cmd_list.push_back(0x3E);                                        //0          
    cmd_list.push_back(0xA4);                                        //1
    cmd_list.push_back(static_cast<uint8_t>(id_));                    //2   
    cmd_list.push_back(0x0C);                                        //3
    uint8_t checksum_head = GetChecksum(cmd_list,0);                       
    cmd_list.push_back(checksum_head);                               //4
    cmd_list.push_back(*(uint8_t *)(&angle));                        //5
    cmd_list.push_back(*((uint8_t *)(&angle)+1));                      //6 
    cmd_list.push_back(*((uint8_t *)(&angle)+2));                      //7  
    cmd_list.push_back(*((uint8_t *)(&angle)+3));                      //8 
    cmd_list.push_back(*((uint8_t *)(&angle)+4));                      //9 
    cmd_list.push_back(*((uint8_t *)(&angle)+5));                      //10 
    cmd_list.push_back(*((uint8_t *)(&angle)+6));                      //11
    cmd_list.push_back(*((uint8_t *)(&angle)+7));                      //12
    cmd_list.push_back(*(uint8_t *)(&speed));                          //13
    cmd_list.push_back(*((uint8_t *)(&speed)+1));                      //14
    cmd_list.push_back(*((uint8_t *)(&speed)+2));                      //15 
    cmd_list.push_back(*((uint8_t *)(&speed)+3));                      //16
    uint8_t checksum_data = GetChecksum(cmd_list,5);                   
    cmd_list.push_back(checksum_data);                               //17                     
    
    cmd_list_ = cmd_list;



}

void ControlCmd::Speed(double speed_in){
    int32_t speed = std::llround(speed_in*100);
    
    std::vector<uint8_t> cmd_list;

    cmd_list.push_back(0x3E);                                        //0          
    cmd_list.push_back(0xA2);                                        //1
    cmd_list.push_back(static_cast<uint8_t>(id_));                    //2   
    cmd_list.push_back(0x04);                                        //3
    uint8_t checksum_head = GetChecksum(cmd_list,0);                       
    cmd_list.push_back(checksum_head);                               //4
    cmd_list.push_back(*(uint8_t *)(&speed));                        //5
    cmd_list.push_back(*((uint8_t *)(&speed)+1));                      //6 
    cmd_list.push_back(*((uint8_t *)(&speed)+2));                      //7  
    cmd_list.push_back(*((uint8_t *)(&speed)+3));                      //8 
    uint8_t checksum_data = GetChecksum(cmd_list,5);
    cmd_list.push_back(checksum_data);                               //13                        
    cmd_list_ = cmd_list;

}

void ControlCmd::SingleLoopAngle(int spin_direction_in ,double angle_in){
    uint8_t spin_direction = std::llround(spin_direction_in);
    uint16_t angle = std::llround(angle_in*100);
    

    std::vector<uint8_t> cmd_list;

    cmd_list.push_back(0x3E);                                        //0          
    cmd_list.push_back(0xA5);                                        //1
    cmd_list.push_back(static_cast<uint8_t>(id_));                    //2   
    cmd_list.push_back(0x04);                                        //3
    uint8_t checksum_head = GetChecksum(cmd_list,0);                       
    cmd_list.push_back(checksum_head);                               //4
    cmd_list.push_back(spin_direction);
    cmd_list.push_back(*(uint8_t *)(&angle));                        //5
    cmd_list.push_back(*((uint8_t *)(&angle)+1));                      //6 
    cmd_list.push_back(0x00);                      //8 
    uint8_t checksum_data = GetChecksum(cmd_list,5);
    cmd_list.push_back(checksum_data);                               //13                        
    cmd_list_ = cmd_list;
}


void ControlCmd::SingleLoopAngleWithSpeed(int spin_direction_in ,double angle_in, double speed_in){
    uint8_t spin_direction = std::llround(spin_direction_in);
    uint16_t angle = std::llround(angle_in*100);
    uint32_t speed = std::llround(speed_in*100);

    std::vector<uint8_t> cmd_list;


    cmd_list.push_back(0x3E);                                        //0          
    cmd_list.push_back(0xA6);                                        //1
    cmd_list.push_back(static_cast<uint8_t>(id_));                    //2   
    cmd_list.push_back(0x08);                                        //3
    uint8_t checksum_head = GetChecksum(cmd_list,0);                       
    cmd_list.push_back(checksum_head);                               //4
    cmd_list.push_back(spin_direction);
    cmd_list.push_back(*(uint8_t *)(&angle));                        //5
    cmd_list.push_back(*((uint8_t *)(&angle)+1));                      //6 
    cmd_list.push_back(0x00);                      //8 
    cmd_list.push_back(*(uint8_t *)(&speed));                        //5
    cmd_list.push_back(*((uint8_t *)(&speed)+1));                      //6 
    cmd_list.push_back(*((uint8_t *)(&speed)+2));                      //7  
    cmd_list.push_back(*((uint8_t *)(&speed)+3));     
    uint8_t checksum_data = GetChecksum(cmd_list,5);
    cmd_list.push_back(checksum_data);                               //13                        
    cmd_list_ = cmd_list;
}




uint8_t ControlCmd::GetChecksum(std::vector<uint8_t> &cmd_list, int left){
    uint8_t checksum = 0x00;
    for(int i = left; i < cmd_list.size(); i ++){
        checksum = checksum + cmd_list[i];
    }
    return checksum;
}

void ControlCmd::ShowCmdlist(){
        for(int i = 0; i < cmd_list_.size(); i++){
        std::cout << "nums " << i << "  " << std::setw(2) << std::setfill('0') << 
        std::hex << static_cast<int>(cmd_list_[i]) << std::endl;
    }
}

std::vector<uint8_t> ControlCmd::OutputCmdlist(){
    // ShowCmdlist();
    return cmd_list_;
}