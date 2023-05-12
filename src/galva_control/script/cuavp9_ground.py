#!/usr/bin/env python3
import time
import serial

serial_port = '/dev/ttyUSB0'
baud_rate = 57600

cuavp9_ser = serial.Serial(serial_port, baud_rate, timeout=1)

# 设置参数
send_data = 'AT&F7\n' + 'ATS102=2\n' + 'ATS103=4\n' + 'ATS104=1234567890\n' + \
            'ATS108=30\n' + 'ATS105=1\n' + 'ATS140=65535\n' + 'AT&V\n' + 'AT&W\n'
print(send_data)

# 发送参数
cuavp9_ser.write(send_data.encode('ascii'))

# 等待回复
while True:
    buffer = cuavp9_ser.read(1024)
    if b'OK' in buffer:
        print(buffer.decode('ascii'))
        break
    time.sleep(0.05)
