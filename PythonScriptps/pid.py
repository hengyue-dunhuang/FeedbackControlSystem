import serial
import struct
from time import sleep
class pid_control():
    def __init__(self,com='COM3'):
        ser = serial.Serial(com, 115200)
        if ser.is_open: ser.close()
        ser.open()
        self.ser = ser
        self.P,self.I,self.D = [0.1,0.1,0.01]
        self.error = []
        self.ser.set_buffer_size(10)
        self.speed_buffer = [0 for i in range(5)]
        self.error_buffer = [0 for i in range(3)]
        self.speed_goal = 50
        self.position_goal = 0
        self.speed_control = 0
        self.speed = 0

    def read(self):
        data = self.ser.read(10) # 读取10个字节
        index = data.find(b'\xaa')
        data = data[index:index+5]
        try:
            float_value = struct.unpack('f', data[1:])[0]
            self.speed_buffer.append(float_value) 
            self.speed_buffer.pop(0)
        except:
            float_value = self.speed_buffer[-1]
        return float_value
    
    def send_data(self,speed):  #0-200级调速
        speed = int(speed)
        if speed >= 0:
            string = hex(speed)[2:]
            if len(string)==1:
                string = '0'+string
            self.ser.write(b'\x00'+bytes.fromhex(string))
        else:
            
            string = hex(-speed)[2:]
            if len(string)==1:
                string = '0'+string
            self.ser.write(b'\x01'+bytes.fromhex(string))
        # ser.read(2)
            self.ser.flush()
    def error_update(self):
        current_speed = self.read()
        self.speed = current_speed
        error =   self.speed_goal - current_speed
        self.error_buffer.append(error)
        self.error_buffer.pop(0)
    def pid_speed_control(self):
        self.error_update()
        delta = (self.P*(self.error_buffer[-1]-self.error_buffer[-2])
                +self.I*self.error_buffer[-1]
                +self.D*(self.error_buffer[-1]-2*self.error_buffer[-2]+self.error_buffer[-3])
        )
        self.speed_control = self.speed_control + delta
        if self.speed_control>200:
            self.speed_control = 200
        elif self.speed_control<-200:
            self.speed_control = -200

        self.send_data(self.speed_control)
        



if __name__ == '__main__':
    test = pid_control()
    test.send_data(56)
    while(1):
        test.pid_speed_control()
        # print(test.speed_control)
        print(test.speed)
        sleep(0.03)
