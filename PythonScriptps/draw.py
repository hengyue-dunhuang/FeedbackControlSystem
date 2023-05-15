import serial 
import struct
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from numpy import mean
from time import sleep


fig, ax = plt.subplots()


ser = serial.Serial('COM3', 115200)  # 替换为正确的串口号和波特率
if ser.is_open: ser.close()
ser.open()
ser.set_buffer_size(200)
bufffer_len = 10
speed_buffer = [0 for i in range(bufffer_len)]

line, = ax.plot(speed_buffer)
ax.set_ylim(0, 300)  # 设置纵坐标轴范围为0到100

count = 0
# set_speed = 10
# ser.write(b'\x00\x55')
# ser.write(b'\x01\x03')
# ser.write(b'\x01')
# ser.write(b'\xab')
# sleep(1)

def send_data(speed):  #0-200级调速
    if speed >= 0:
        string = hex(speed)[2:]
        ser.write(b'\x00'+bytes.fromhex(string))
    else:
        string = hex(-speed)[2:]
        ser.write(b'\x01'+bytes.fromhex(string))
    # ser.read(2)
    ser.flush()
    
def update(frame):
    send_data(20)
    data = ser.read(10) # 读取10个字节
    index = data.find(b'\xaa')
    data = data[index:index+5]
    try:
        float_value = struct.unpack('f', data[1:])[0]
        speed_buffer.append(float_value) 
        speed_buffer.pop(0)
    except:
        float_value = speed_buffer[-1]

    print(min(speed_buffer))
    line.set_ydata(speed_buffer)
    
    return line
    # print(speed_buffer)
    # sleep(0.03)
    # print(float_value)
# 创建动画
ani = animation.FuncAnimation(fig, update, interval=50)

# 显示图形
plt.show()


