#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import sys, re, signal, time
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

PORT = "/dev/ttyUSB0"
BAUD = 921600
TIMEOUT_S = 0.3
HEX_FRAME_REGEX = re.compile(rb'([0-9A-Fa-f]{24})')

def parse_hex24_to_i16s(hex_bytes24: bytes):
    vals = []
    for i in range(0, 24, 4):
        w = int(hex_bytes24[i:i+4], 16)
        if w >= 0x8000:
            w -= 0x10000
        vals.append(w)
    return vals

def send(ser, cmd: bytes):
    ser.write(cmd + b"\r\n")

class FTSensorNode(Node):
    def __init__(self):
        super().__init__('ft_sensor_publisher')

        # ROS2 Publisher
        self.publisher_ = self.create_publisher(Float32MultiArray, 'ft_data', 10)

        # Serial 初始化
        '''try:
            self.ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT_S,
                                     bytesize=serial.EIGHTBITS,
                                     parity=serial.PARITY_NONE,
                                     stopbits=serial.STOPBITS_ONE)
            self.get_logger().info(f"Connected to {PORT} at {BAUD} baud.")
        except Exception as e:
            self.get_logger().error(f"Serial open failed: {e}")
            sys.exit(1)'''
        try:
            ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT_S)
        except Exception as e:
            print(f"Serial open failed: {e}")
            ser = None  # 先不 crash，程式繼續跑


        # 定時器：週期讀取資料
        self.timer = self.create_timer(0.05, self.read_and_publish)  # 20Hz 更新頻率
        self.stop = False

        # Ctrl+C 停止
        signal.signal(signal.SIGINT, self.on_sigint)

    def on_sigint(self, sig, frame):
        self.get_logger().info("Stopping...")
        self.stop = True
        rclpy.shutdown()

    def read_and_publish(self):
        if self.stop:
            return
        try:
            send(self.ser, b"READ?")
            line = self.ser.read_until(b"\r\n")
            if not line:
                return
            m = HEX_FRAME_REGEX.search(line)
            if not m:
                return
            Fx, Fy, Fz, Mx, My, Mz = parse_hex24_to_i16s(m.group(1))

            # 終端機印出
            Fx += 0
            Fy += 0
            Fz += 0
            
            print(f"Fx={Fx:+6d}  Fy={Fy:+6d}  Fz={Fz:+6d}   Mx={Mx:+6d}  My={My:+6d}  Mz={Mz:+6d}")

            # 發布 ROS2 topic
            msg = Float32MultiArray()
            msg.data = [Fx, Fy, Fz, Mx, My, Mz]
            self.publisher_.publish(msg)

        except Exception as e:
            self.get_logger().warn(f"Error reading serial: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = FTSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "_main_":
    main()