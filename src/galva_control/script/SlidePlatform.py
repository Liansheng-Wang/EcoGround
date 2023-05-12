#!/usr/bin/env python3

import colorama as cama
import datetime
import time
import math
import threading
from pymodbus.client.sync import ModbusSerialClient

cama.init()

# USB转RS485设备的串口名称和波特率
serial_port1 = '/dev/ttyUSB0'
serial_port2 = '/dev/ttyUSB1'
serial_port3 = '/dev/ttyUSB2'
serial_port4 = '/dev/ttyUSB3'

# 定义Modbus RTU通信参数
baud_rate = 19200
parity = 'N'           # 'N'、'E'、'O'、'M' 和 'S'，分别表示无校验、偶校验、奇校验、标记校验和空格校验
stop_bits = 1
data_bits = 8

# 寄存器地址
# 写入的寄存器地址
enable_reg = 98
write_vel = 137
# 读状态的寄存器地址
speed_add = 0x0000
motor_pos_low = 0x0005
motor_pos_high = 0x0006
pos_err_low = 0x0007
pos_err_high = 0x0008

class MyModbusClient:
    def __init__(self, motor_id, serial_port, slave_address, lower = 0, upper = 5020, distance = 1.0,
                 kp = 1.0, ki = 0.0, kd = 0.1, vel_limit_max = 300, vel_limit_min = 12, direction = 1):
        self.id = motor_id
        self.serial_port = serial_port
        self.slave_address = slave_address
        self.distance = distance
        self.lower = lower
        self.upper = upper
        self.revolution = (upper - lower) / self.distance  # 分辨率定义为 degree/m
        self.target_degree = 0.0                           # 目标度数
        self.current_pose_meter = 0.0                      # 当前所在行程   (m)
        self.current_pose_degree = 0.0                     # 当前所在的度数 (du)
        self.direction = direction                         # 电机转向是否求反
        self.last_err = 0.0
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.vel_limit_max = vel_limit_max                 # 最大速度不能超过 60r/min = 360 d/s
        self.vel_limit_min = vel_limit_min                 # 最小速度不能超过 5r/min  = 30 d/s
        self.flag_target_change = False                    # 目标值更新的Flag
        self.flag_complete = False                         # 当前线程结束
        self.execute_thread = threading.Thread(target=self.__execute)
        self.mutex = threading.Lock()
        self.client = ModbusSerialClient(method='rtu', port=serial_port, baudrate=baud_rate, timeout=1,
                                         parity=parity, stopbits=stop_bits, bytesize=data_bits)

        if self.client.connect():
            print(cama.Fore.GREEN + '电机：'+ str(self.id) + ', 连接成功')
        else:
            print(cama.Fore.RED + '电机：' + str(self.id) + ', 连接失败')

    def __del__(self):
        self.flag_target_change = True
        # self.disable_motor()

    # 测试通过
    def enable_motor(self):
        # 向使能寄存器写入 1，使电机使能
        result = self.client.write_register(address=98, value=1, unit=self.slave_address)
        if not result.isError():
            print(cama.Fore.GREEN + '电机：'+ str(self.id) + ', 已使能')
        else:
            print(cama.Fore.RED + '电机：'+ str(self.id) + ', 使能失败')

    # 测试通过
    def disable_motor(self):
        # 向使能寄存器写入 0，使电机不使能
        result = self.client.write_register(address=98, value=0, unit=self.slave_address)
        if not result.isError():
            print(cama.Fore.GREEN + '电机：' + str(self.id) + ', 已停止使能')
        else:
            print(cama.Fore.RED + '电机：'+ str(self.id) + ', 停止使能失败')

    # 测试通过
    def set_vel(self, vel: float):
        # 设置速度值
        vel_reg = int(vel / 6.0)                     # 寄存器接收的是 r/min, 接收范围为 -5000~5000
        vel_reg_to_show = vel_reg
        vel_reg = self.direction * vel_reg           # 给电机的真实速度
        if vel_reg < 0:
            vel_reg += 65536
        result = self.client.write_register(address=write_vel, value=vel_reg, unit=self.slave_address)
        if result.isError():
            print(cama.Fore.RED + '电机：' + str(self.id) + ', 速度设置失败')
        # else:
            # print(cama.Fore.GREEN + '电机：' + str(self.id) + ', 速度已设置为： {:.2f}'.format(vel) + \
            #       ' degree/s,  ' + str(vel_reg_to_show) + ' r/min.')
            

    def get_position(self):
        with self.mutex:
            return self.current_pose_meter

    # 测试通过
    def __get_position(self):
        # 读取位置值
        result1 = self.client.read_input_registers(address=motor_pos_low, count=1, unit=self.slave_address)
        result2 = self.client.read_input_registers(address=motor_pos_high, count=1, unit=self.slave_address)
        if not result1.isError() and not result2.isError():
            pos_reg_low = result1.registers[0]
            pos_reg_high = result2.registers[0]
            # 判断高位是否为负数
            if pos_reg_high >= 0x8000:
                pos_reg_high = pos_reg_high - 0x10000  # 转换成负数
            pos_reg = (pos_reg_high << 16) | pos_reg_low
            with self.mutex:
                self.current_pose_degree = float(pos_reg) / 10000 * 360 * self.direction  # 将Modbus寄存器的值除以10000并转换为浮点数，作为位置值
                self.current_pose_meter  = self.current_pose_degree / self.revolution
            # print(cama.Fore.GREEN + '电机：' + str(self.id) + ', 位置： {:.2f} 度, {:.2f} 米'.\
                  # format(self.current_pose_degree, self.current_pose_meter))
            # return self.current_pose_degree, self.current_pose_meter
        else:
            print(cama.Fore.RED + '电机：' + str(self.id) + ', 位置读取失败')
            # return None

    def __execute(self):
        self.flag_target_change = False       # 领取任务，切换标志
        self.flag_complete = False
        self.last_err = 0.0
        err_for_kd = 0.0
        while not self.flag_target_change:
            self.__get_position()
            self.mutex.acquire()
            if math.fabs(self.target_degree - self.current_pose_degree) < 2:
                self.set_vel(0)
                self.mutex.release()
                break
            vel = self.kp * (self.target_degree - self.current_pose_degree) + self.kd * err_for_kd
            err_for_kd = self.target_degree - self.current_pose_degree - self.last_err
            self.last_err = self.target_degree - self.current_pose_degree
            self.mutex.release()
            if vel > self.vel_limit_max:
                vel = self.vel_limit_max
            elif vel < -self.vel_limit_max:
                vel = -self.vel_limit_max
            if 0 <= vel < self.vel_limit_min:
                vel = self.vel_limit_min
            elif -self.vel_limit_min < vel < 0:
                vel = - self.vel_limit_min
            self.set_vel(vel)
            time.sleep(0.1)
        self.flag_complete = True

    # 设置距离 target 是 0~1.0的米制单位
    def set_distance(self, target: float):
        self.target_degree = self.revolution * target
        self.flag_target_change = True
        # print(cama.Fore.LIGHTGREEN_EX + '电机：' + str(self.id) + "设置任务： {:.2f} 米， {:.2f} 度".format(target, self.target_degree))
        # 等待旧线程的执行完毕
        if self.execute_thread.is_alive():
            self.execute_thread.join()
        # 需要重置线程函数
        self.execute_thread = threading.Thread(target=self.__execute)
        self.execute_thread.start()

    # 未测试
    def get_position_error(self):
        # 读取位置偏差值
        result1 = self.client.read_input_registers(address=pos_err_low, count=1, unit=self.slave_address)
        result2 = self.client.read_input_registers(address=pos_err_high, count=1, unit=self.slave_address)
        if not result1.isError() and not result2.isError():
            err_reg_low = result1.registers[0]
            err_reg_high = result2.registers[0]
            err_reg = (err_reg_high << 16) + err_reg_low  # 将高低字节的值合并为一个整数
            if err_reg > 0x7FFF:
                err_reg = err_reg - 0x10000  # 将值转换为有符号整数
            err = float(err_reg) / 1000  # 将Modbus寄存器的值除以1000并转换为浮点数，作为位置偏差值
            return err
        else:
            print(cama.Fore.RED + 'Read error:', result1, result2)

    # 未测试
    def get_speed(self):
        # 读取位置偏差值
        result = self.client.read_input_registers(address=speed_add, count=1, unit=self.slave_address)
        if not result.isError():
            speed = result.registers[0]
            # r/min 转化为 度/s
            speed_degree_per_second = speed * 6.0
            return speed_degree_per_second
        else:
            print(cama.Fore.RED + 'Read error:', result)


class SlidePlatform:
    def __init__(self, client1 : MyModbusClient, client2 : MyModbusClient):
        # 默认 client1 为x, client2 为y
        self.client1 = client1
        self.client2 = client2
        self.pose = (0,0)

    def setXY(self, x: float, y: float):
        self.client1.set_distance(x)
        self.client2.set_distance(y)

    def enable_motor(self):
        self.client1.enable_motor()
        self.client2.enable_motor()

    def disable_motor(self):
        self.client1.disable_motor()
        self.client2.disable_motor()

    def get_position(self):
        x_meter = self.client1.get_position()
        y_meter = self.client2.get_position()
        self.pose = (x_meter, y_meter)
        return self.pose

    def circle(self, numbers: int, time_per_circle: float):
        print(cama.Fore.LIGHTBLUE_EX + "Circle: under execution...")
        print(cama.Fore.LIGHTBLUE_EX + "Circle: Go to the starting point.")
        self.setXY(0.2, 0.5)
        # 等待到达起始点
        if self.client1.execute_thread.is_alive():
            self.client1.execute_thread.join()
        if self.client2.execute_thread.is_alive():
            self.client2.execute_thread.join()
        start = datetime.datetime.now()
        end = datetime.datetime.now()
        during = (end - start).total_seconds()
        w_v = 2 * math.pi / time_per_circle
        while during < numbers * time_per_circle:
            end = datetime.datetime.now()
            during = (end - start).total_seconds()
            pose_x = 0.5 - 0.3 * math.cos(w_v * during)
            pose_y = 0.5 + 0.3 * math.sin(w_v * during)
            self.setXY(pose_x, pose_y)
            time.sleep(0.1)

        if self.client1.execute_thread.is_alive():
            self.client1.execute_thread.join()
        if self.client2.execute_thread.is_alive():
            self.client2.execute_thread.join()

        print(cama.Fore.LIGHTBLUE_EX + "Circle: complete.")


# Test
def main():
    client1 = MyModbusClient(1, serial_port1, 1, kp=1.5, kd=1.0)
    client2 = MyModbusClient(2, serial_port2, 1, direction = -1)
    client3 = MyModbusClient(3, serial_port3, 1, direction = -1, kp=1.5)
    client4 = MyModbusClient(4, serial_port4, 1)
    slide_platform_down = SlidePlatform(client1, client2)
    slide_platform_up   = SlidePlatform(client3, client4)
    slide_platform_down.enable_motor()
    slide_platform_up.enable_motor()
    slide_platform_down.circle(1, 30)
    slide_platform_up.circle(1, 30)
    slide_platform_up.disable_motor()
    slide_platform_down.disable_motor()

if __name__ == '__main__':
    main()
