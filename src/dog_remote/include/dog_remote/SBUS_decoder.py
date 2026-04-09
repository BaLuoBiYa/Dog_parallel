from encodings.utf_8 import encode
import logging
import serial
from sympy import true
import rclpy
from rclpy.node import Node
from msgs_pkg.msg import SerialData
import numpy

SBUS_FRAME_SIZE = 25
SBUS_FRAME_HEADER = 0x0F
SBUS_FRAME_END = 0x00
SBUS_CHANNEL_SIZE = 16
FILTER_INDEX_SIZE = 5


class SBUSDecoder(Node):
    def __init__(self, port = '/dev/ttyUSB0', baudrate = 100000, timeout = 1,
                 ):
      super().__init__('SBUS_decoder')
      #变量定义
      self.databuffer = bytearray()
      self.channel_data = numpy.zeros(SBUS_CHANNEL_SIZE, dtype=numpy.int16)
      self.filter_buffer = numpy.zeros((SBUS_CHANNEL_SIZE, FILTER_INDEX_SIZE))
      self.buffer_index = numpy.zeros(SBUS_CHANNEL_SIZE, dtype=numpy.int16)
      self.last_receive_time = 0
      self.filter_enable = True
      self.mad_k = 3
      
      
      #创建订阅者
      self.ser = serial.Serial(port, baudrate, timeout=timeout)
      self.publishers_ = self.create_publisher(SerialData, 'SBUS_data', 10)
      self.timers_ = self.create_timer(0.01, self.timercallback)
      
      print('SBUS节点初始化完成')
            
    def timercallback(self):
      #接收串口消息
      if self.ser.in_waiting > 0:
        new_msg = self.receive_msgs(self.ser.in_waiting)
        if new_msg:
          self.databuffer.extend(new_msg)
        #保证包头和包尾匹配
        while len(self.databuffer) >= SBUS_FRAME_SIZE:
          if self.databuffer[0] != SBUS_FRAME_HEADER:
            del self.databuffer[0]
            continue
          
          if self.databuffer[SBUS_FRAME_SIZE - 1] == SBUS_FRAME_END:
            self.extract_data()
            del self.databuffer[:SBUS_FRAME_SIZE]
          else:
            del self.databuffer[0]
            
    def receive_msgs(self, waiting_msgs):
      return self.ser.read(waiting_msgs)
      
        
        #提取串口中数据,并解码为可使用数据
    def extract_data(self):
      self.channel_data[0] = ((self.databuffer[1] | self.databuffer[2] << 8) & 0x07FF)
      self.channel_data[1] = ((self.databuffer[2] >> 3 | self.databuffer[3] << 5) & 0x07FF)
      self.channel_data[2] = ((self.databuffer[3] >> 6 | self.databuffer[4] << 2 | self.databuffer[5] << 10) & 0x07FF)
      self.channel_data[3] = ((self.databuffer[5] >> 1 | self.databuffer[6] << 7) & 0x07FF)
      self.channel_data[4] = ((self.databuffer[6] >> 4 | self.databuffer[7] << 4) & 0x07FF)
      self.channel_data[5] = ((self.databuffer[7] >> 7 | self.databuffer[8] << 1 | self.databuffer[9] << 9) & 0x07FF)
      self.channel_data[6] = ((self.databuffer[9] >> 2 | self.databuffer[10] << 6) & 0x07FF)
      self.channel_data[7] = ((self.databuffer[10] >> 5 | self.databuffer[11] << 3) & 0x07FF)
      self.channel_data[8] = ((self.databuffer[12] | self.databuffer[13] << 8) & 0x07FF)
      self.channel_data[9] = ((self.databuffer[13] >> 3 | self.databuffer[14] << 5) & 0x07FF)
      self.channel_data[10] = ((self.databuffer[14] >> 6 | self.databuffer[15] << 2 | self.databuffer[16] << 10) & 0x07FF)
      self.channel_data[11] = ((self.databuffer[16] >> 1 | self.databuffer[17] << 7) & 0x07FF)
      self.channel_data[12] = ((self.databuffer[17] >> 4 | self.databuffer[18] << 4) & 0x07FF)
      self.channel_data[13] = ((self.databuffer[18] >> 7 | self.databuffer[19] << 1 | self.databuffer[20] << 9) & 0x07FF)
      self.channel_data[14] = ((self.databuffer[20] >> 2 | self.databuffer[21] << 6) & 0x07FF)
      self.channel_data[15] = ((self.databuffer[21] >> 5 | self.databuffer[22] << 3) & 0x07FF)
      
      self.last_receive_time = self.get_clock().now()
      
      if self.filter_enable == true:
        self.Apply_Filter()
      
      #启用滤波
    def Apply_Filter(self):
      for ch in range(SBUS_CHANNEL_SIZE):
        self.raw_val = self.channel_data[ch]
        #中值滤波
        median_val = self.Median_Filter(ch, self.raw_val)
        #Hampel检测
        self.channel_data[ch] = self.Hampel_Filter(ch, median_val)
        
        
    def Median_Filter(self,ch ,new_val):
      #更新缓冲区
      self.filter_buffer[ch][self.buffer_index[ch]] = new_val
      self.buffer_index[ch] = (self.buffer_index[ch] + 1) % FILTER_INDEX_SIZE
      
      #排序取中值
      #           复制的对吗？
      temp = numpy.copy(self.filter_buffer[ch])
      #按照大小重新排序
      temp.sort()
      
      return temp[int(round(FILTER_INDEX_SIZE/2, 1))]
      
      
    def Hampel_Filter(self, ch, new_vel):
      
      #有必要重复定义吗
      window = numpy.copy(self.filter_buffer[ch])
      temp = numpy.copy(window)
      temp.sort()
      #计算中值
      my_median = temp[int(round(FILTER_INDEX_SIZE/2, 1))]
      
      deviations = numpy.zeros(FILTER_INDEX_SIZE)
      for i in range(FILTER_INDEX_SIZE):
        deviations[i] = abs(window[i] - my_median)
      deviations.sort()
      
      mad = deviations[int(round(FILTER_INDEX_SIZE/2, 1))]
      
      return my_median if abs(new_vel - my_median) > self.mad_k * mad else new_vel
      
def main():
  rclpy.init()
  node = SBUSDecoder()
  rclpy.spin(node)
  rclpy.shutdown()
        
if __name__ == '__main__':
    main()
