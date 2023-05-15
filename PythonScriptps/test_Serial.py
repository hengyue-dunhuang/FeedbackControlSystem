import serial 
import struct
# import matplotlib.pyplot as plt
# import matplotlib.animation as animation
from numpy import mean
from time import sleep




ser = serial.Serial('COM3', 115200)  # 替换为正确的串口号和波特率
if ser.is_open: ser.close()
ser.open()
ser.write(b'\x00\x30')

bufffer_len = 10
speed_buffer = [0 for i in range(bufffer_len)]

 
set_speed = 10

# while(1):
    
#     data = ser.read(5) # 读取4个字节
#     float_value = struct.unpack('f', data[1:])[0]
#     speed_buffer.insert(-1,float_value) 
#     speed_buffer.pop(0)
#     print(speed_buffer)
#     sleep(0.03)
#     # print(float_value)

