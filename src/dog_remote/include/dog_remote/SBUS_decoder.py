#!/usr/bin/env python3
import serial
import numpy as np
import time
import sys

SBUS_FRAME_SIZE = 25
SBUS_FRAME_HEADER = 0x0F
SBUS_FRAME_END = 0x00
SBUS_CHANNEL_SIZE = 16
FILTER_INDEX_SIZE = 5

class SBUSDecoder:
    def __init__(self, port='/dev/ttyS1', baudrate=100000, timeout=0.1):
        self.ser = serial.Serial(port, baudrate, 
                                 timeout=timeout, 
                                 parity=serial.PARITY_EVEN,
                                stopbits=serial.STOPBITS_TWO,
                                bytesize=serial.EIGHTBITS)
        self.databuffer = bytearray()
        self.channel_data = np.zeros(SBUS_CHANNEL_SIZE, dtype=np.int16)
        self.filter_buffer = np.zeros((SBUS_CHANNEL_SIZE, FILTER_INDEX_SIZE), dtype=float)
        self.buffer_index = np.zeros(SBUS_CHANNEL_SIZE, dtype=int)
        self.filter_enable = True
        self.mad_k = 3

    def read_into_buffer(self):
        n = self.ser.in_waiting
        if n:
            self.databuffer.extend(self.ser.read(n))

    def feed_and_parse(self):
        while len(self.databuffer) >= SBUS_FRAME_SIZE:
            if self.databuffer[0] != SBUS_FRAME_HEADER:
                del self.databuffer[0]
                continue
            if self.databuffer[SBUS_FRAME_SIZE - 1] == SBUS_FRAME_END:
                frame = self.databuffer[:SBUS_FRAME_SIZE]
                del self.databuffer[:SBUS_FRAME_SIZE]
                self.extract_data(frame)
                if self.filter_enable:
                    self.apply_filter()
                self.print_channels()
            else:
                del self.databuffer[0]

    def extract_data(self, buf):
        b = buf  # alias
        self.channel_data[0]  = ((b[1]  | (b[2]  << 8))                       & 0x07FF)
        self.channel_data[1]  = (((b[2] >> 3) | (b[3]  << 5))                 & 0x07FF)
        self.channel_data[2]  = (((b[3] >> 6) | (b[4]  << 2) | (b[5] << 10))  & 0x07FF)
        self.channel_data[3]  = (((b[5] >> 1) | (b[6]  << 7))                 & 0x07FF)
        self.channel_data[4]  = (((b[6] >> 4) | (b[7]  << 4))                 & 0x07FF)
        self.channel_data[5]  = (((b[7] >> 7) | (b[8]  << 1) | (b[9] << 9))   & 0x07FF)
        self.channel_data[6]  = (((b[9] >> 2) | (b[10] << 6))                 & 0x07FF)
        self.channel_data[7]  = (((b[10]>> 5) | (b[11] << 3))                 & 0x07FF)
        self.channel_data[8]  = ((b[12] | (b[13] << 8))                       & 0x07FF)
        self.channel_data[9]  = (((b[13]>> 3) | (b[14] << 5))                 & 0x07FF)
        self.channel_data[10] = (((b[14]>> 6) | (b[15] << 2) | (b[16] << 10)) & 0x07FF)
        self.channel_data[11] = (((b[16]>> 1) | (b[17] << 7))                 & 0x07FF)
        self.channel_data[12] = (((b[17]>> 4) | (b[18] << 4))                 & 0x07FF)
        self.channel_data[13] = (((b[18]>> 7) | (b[19] << 1) | (b[20] << 9))  & 0x07FF)
        self.channel_data[14] = (((b[20]>> 2) | (b[21] << 6))                 & 0x07FF)
        self.channel_data[15] = (((b[21]>> 5) | (b[22] << 3))                 & 0x07FF)
        print(self.channel_data[0])
        print(self.channel_data[1])
      
    def apply_filter(self):
        for ch in range(SBUS_CHANNEL_SIZE):
            raw_val = float(self.channel_data[ch])
            median_val = self.median_filter(ch, raw_val)
            self.channel_data[ch] = int(self.hampel_filter(ch, median_val))

    def median_filter(self, ch, new_val):
        idx = int(self.buffer_index[ch])
        self.filter_buffer[ch, idx] = new_val
        self.buffer_index[ch] = (idx + 1) % FILTER_INDEX_SIZE
        temp = np.copy(self.filter_buffer[ch])
        temp.sort()
        median_idx = FILTER_INDEX_SIZE // 2
        return float(temp[median_idx])

    def hampel_filter(self, ch, new_vel):
        window = np.copy(self.filter_buffer[ch])
        temp = np.copy(window)
        temp.sort()
        median_idx = FILTER_INDEX_SIZE // 2
        my_median = float(temp[median_idx])
        deviations = np.abs(window - my_median)
        deviations.sort()
        mad = float(deviations[median_idx])
        if mad == 0:
            return new_vel
        return my_median if abs(new_vel - my_median) > self.mad_k * mad else new_vel

    def print_channels(self):
        # 打印原始 0..2047 范围的通道值；若需标准化到 [-1,1]，可按需修改
        print("Channels:", ", ".join(str(int(x)) for x in self.channel_data))

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyS1'
    decoder = SBUSDecoder(port=port)
    try:
        while True:
            decoder.read_into_buffer()
            decoder.feed_and_parse()
            time.sleep(0.001)
    except KeyboardInterrupt:
        print("Exiting.")
    finally:
        decoder.ser.close()

if __name__ == '__main__':
    main()