#! /usr/bin/env python3

import serial
import time

serial_port = '/dev/ttyUSB1'
baud_rate = 9600

cuavp9_ser = serial.Serial(serial_port, baud_rate, timeout=1)

send_data = 'AT&F8\n' + 'ATS102=2\n' + 'ATS103=4\n' + 'ATS104=1234567890\n' + \
            'ATS108=30\n' + 'ATS105=2\n' + 'ATS140=1\n' + 'AT&V\n' + 'AT&W\n'
print(send_data)

# cuavp9_ser.write(send_data.encode())

while True:
    buffer = cuavp9_ser.read(1024)
    print(buffer)
    time.sleep(0.05)