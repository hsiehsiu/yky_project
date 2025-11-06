#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import json
from vision_msgs.msg import Detection2DArray
from rclpy.qos import QoSProfile

class KeycapTransferNode(Node):
    def __init__(self):
        super().__init__('keycap_transfer_node')

        # --- 讀取手臂末端座標 ---
        try:
            with open('/home/hudenxiao/tmdriver_ws/src/tmr_ros2/json/feedback_pose.json', 'r') as f:
                data = json.load(f)
                tool_pose = data['tool_pose']
                self.arm_x = tool_pose['x']
                self.arm_y = tool_pose['y']
                self.arm_z = tool_pose['z']
        except Exception as e:
            self.get_logger().error(f'無法讀取 feedback_pose.json: {e}')
            return

        # --- 相機在手臂座標系下的位置 ---
        self.camera_x = self.arm_x + 30
        self.camera_y = self.arm_y
        self.camera_z = self.arm_z - 85

        self.get_logger().info(
            f'相機在手臂座標系的位置: x={self.camera_x:.2f}, y={self.camera_y:.2f}, z={self.camera_z:.2f}'
        )

        # --- 儲存轉換後結果 ---
        self.keycap_coords = {}

        # --- 建立訂閱者 ---
        qos_profile = QoSProfile(depth=10)
        self.subscription = self.create_subscription(
            Detection2DArray,
            '/yolo/detections',
            self.detection_callback,
            qos_profile
        )
        self.subscription  # 避免 lint 警告

    def detection_callback(self, msg: Detection2DArray):
        if not msg.detections:
            self.get_logger().warn("未收到任何偵測結果")
            return

        for det in msg.detections:
            if not det.results:
                continue

            # 嘗試讀取 class_id（不同版本可能是 hypothesis.class_id 或 hypothesis.id）
            hypothesis = det.results[0].hypothesis
            if hasattr(hypothesis, 'class_id'):
                class_id = hypothesis.class_id
            elif hasattr(hypothesis, 'id'):
                class_id = str(int(hypothesis.id))
            else:
                self.get_logger().warn("找不到 class_id 或 id 欄位，略過此偵測")
                continue

            # 取得位置資訊
            if not hasattr(det.results[0], 'pose') or not hasattr(det.results[0].pose, 'pose'):
                self.get_logger().warn(f"鍵帽 {class_id} 缺少 pose 資料，略過")
                continue

            position = det.results[0].pose.pose.position

            # --- 相機座標系 → 手臂座標系 ---
            arm_x = self.camera_x + position.y * 1000
            arm_y = self.camera_y + position.x * 1000
            arm_z = self.camera_z - position.z * 1000 + 50 # for safety

            self.keycap_coords[class_id] = {
                "x": round(arm_x, 3),
                "y": round(arm_y, 3),
                "z": round(arm_z, 3)
            }

            self.get_logger().info(
                f"鍵帽 {class_id}: Arm frame -> x={arm_x:.2f}, y={arm_y:.2f}, z={arm_z:.2f}"
            )

        # --- 寫入 keycap_coordinate.json ---
        try:
            with open('/home/hudenxiao/tmdriver_ws/src/tmr_ros2/json/keycap_coordinate.json', 'w') as f:
                json.dump(self.keycap_coords, f, indent=4)
            self.get_logger().info("已更新 keycap_coordinate.json")
        except Exception as e:
            self.get_logger().error(f'寫入 keycap_coordinate.json 失敗: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = KeycapTransferNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('transfer node 結束運行')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
