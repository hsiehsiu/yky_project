#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool  # ❗ 訊息類型更改為 Bool

import serial  # 繼續使用 pyserial
import time
import sys

# --- 您需要修改的設定 ---
ARDUINO_PORT = '/dev/ttyACM0' # 或是 /dev/ttyUSB0 (請改成您正確的)
BAUD_RATE = 9600           # ❗ 必須和 Arduino 程式中的 Serial.begin() 一致
# --- 設定結束 ---


class SuckerNode(Node):
    def __init__(self):
        super().__init__('sucker')
        self.get_logger().info("Sucker 節點已啟動 (pyserial on/off 模式)。")

        # --- 1. 建立序列埠連線 ---
        try:
            self.ser = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
            self.get_logger().info(f"成功連線到 {ARDUINO_PORT}")
            time.sleep(2) # 等待 Arduino 重啟
            self.get_logger().info("Arduino 已準備就緒。")
            
        except Exception as e:
            self.get_logger().error(f"連線 Arduino 失敗 (位於 {ARDUINO_PORT}): {e}")
            rclpy.shutdown()
            sys.exit(1)

        # --- 2. 建立 ROS 2 訂閱者 (❗ 已修改) ---
        self.subscription = self.create_subscription(
            Bool,                   # ❗ 訂閱 Bool 類型
            'sucker_command',       # ❗ 主題名稱更改為 'sucker_command'
            self.listener_callback,
            10)
        
        self.get_logger().info("節點正在訂閱 /sucker_command 主題 (true/false)...")


    def listener_callback(self, msg):
        
        # --- 3. 發送 "on" 或 "off" (❗ 已修改) ---
        try:
            if msg.data == True:
                # 收到 'true'，發送 "on"
                self.get_logger().info('收到 ON 指令 (true)，發送 "on" ...')
                self.ser.write(b"on\n")
            else:
                # 收到 'false'，發送 "off"
                self.get_logger().info('收到 OFF 指令 (false)，發送 "off" ...')
                self.ser.write(b"off\n")
            
        except Exception as e:
            self.get_logger().error(f"!!! WRITE FAILED: {e}")
            self.get_logger().warn("硬體連線可能已中斷。")
            
    def cleanup(self):
        self.get_logger().info("節點關閉中...")
        try:
            # 關閉前，發送 "off" 指令
            self.ser.write(b"off\n") 
            time.sleep(0.1) # 給 Arduino 一點時間反應
            self.ser.close() 
            self.get_logger().info("Arduino 連線已關閉。")
        except Exception as e:
            self.get_logger().warn(f"關閉時寫入 Arduino 失敗: {e}")


def main(args=None):
    # (這部分與上次相同)
    rclpy.init(args=args)
    sucker_node = SuckerNode()
    try:
        rclpy.spin(sucker_node)
    except KeyboardInterrupt:
        pass 
    finally:
        sucker_node.cleanup()
        sucker_node.destroy_node()
        rclpy.shutdown()
        print("ROS 2 節點已完全關閉。")

if __name__ == '__main__':
    main()