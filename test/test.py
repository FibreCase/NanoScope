import serial
import time
import struct
import random

# 修改为你的串口号和波特率
SERIAL_PORT = 'COM9'
BAUD_RATE = 921600

data = b''.join([struct.pack('<H', random.randint(0, 4095)) for _ in range(4000)])
# 打印data的大小


def send_test_packet(ser):
    ser.write(bytes([0xFF]))
    ser.write(bytes([0xFE]))
    ser.write(data)
    ser.flush()
    print("Test packet sent." , len(data) , "bytes")

def read_serial_output(ser, timeout=5):
    line = ser.readline()
    if line:
        print("ESP32:", line.decode(errors='ignore').strip())

def main():
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1) as ser:
        next_send = time.time()
        while True:
            now = time.time()
            if now >= next_send:
                send_test_packet(ser)
                next_send = now + 0.25  # 1Hz
            read_serial_output(ser)

if __name__ == "__main__":
    main()