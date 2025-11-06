#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import json
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile


class KeycapTransferNode(Node):
    def __init__(self):
        super().__init__('keycap_transfer_node')

        # 初始化變數
        self.arm_x = 0.0
        self.arm_y = 0.0
        self.arm_z = 0.0
        self.camera_x = 0.0
        self.camera_y = 0.0
        self.camera_z = 0.0
        self.keycap_coords = {}

        qos_profile = QoSProfile(depth=10)

        # --- 訂閱手臂末端姿態 (tool_pose) ---
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/tool_pose',
            self.pose_callback,
            qos_profile
        )

        # --- 訂閱 YOLO 偵測結果 ---
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/yolo/detections',
            self.detection_callback,
            qos_profile
        )

        self.get_logger().info("Keycap Transfer Node 啟動完成，等待 /tool_pose 與 /yolo/detections 資料...")

    # ========== 手臂座標回呼 ==========
    def pose_callback(self, msg: PoseStamped):
        # 將單位轉成 mm
        self.arm_x = msg.pose.position.x * 1000
        self.arm_y = msg.pose.position.y * 1000
        self.arm_z = msg.pose.position.z * 1000

        # 相機相對於手臂末端的位置（根據實際量測）
        self.camera_x = self.arm_x + 30   # +X 方向 30 mm
        self.camera_y = self.arm_y        # 相同 Y
        self.camera_z = self.arm_z - 85   # 向下偏 85 mm

        self.get_logger().debug(
            f"更新手臂位置: x={self.arm_x:.1f}, y={self.arm_y:.1f}, z={self.arm_z:.1f}"
        )

    # ========== YOLO 偵測回呼 ==========
    def detection_callback(self, msg: Detection2DArray):
        if not msg.detections:
            self.get_logger().warn("未收到任何偵測結果")
            return

        for det in msg.detections:
            if not det.results:
                continue

            hypothesis = det.results[0].hypothesis
            # 嘗試取 class_id
            if hasattr(hypothesis, 'class_id'):
                class_id = hypothesis.class_id
            elif hasattr(hypothesis, 'id'):
                class_id = str(int(hypothesis.id))
            else:
                self.get_logger().warn("偵測缺少 class_id/id，略過")
                continue

            # 取得位置資訊
            if not hasattr(det.results[0], 'pose') or not hasattr(det.results[0].pose, 'pose'):
                self.get_logger().warn(f"鍵帽 {class_id} 缺少 pose 資料，略過")
                continue

            position = det.results[0].pose.pose.position

            # --- 相機座標系 → 手臂座標系 ---
            arm_x = self.camera_x + position.y * 1000
            arm_y = self.camera_y + position.x * 1000
            arm_z = self.arm_z  # 直接使用手臂當前 z 座標

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
            with open('/home/hsiu/tmrdriver_ws/resource/json/keycap_coordinate.json', 'w') as f:
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
