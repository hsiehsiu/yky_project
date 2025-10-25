#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
import tf2_ros
import numpy as np
import json
from cv_bridge import CvBridge

class DetectionToBase(Node):
    def __init__(self):
        super().__init__('detection_to_base')

        # ---- TF buffer & listener ----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- 影像與深度 ----
        self.bridge = CvBridge()
        self.depth_image = None
        self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.cb_depth, 10)

        # ---- YOLO 偵測結果 ----
        self.create_subscription(Detection2DArray, '/yolo/detections', self.cb_detections, 10)

        # ---- 相機內參 (請改成你的 camera_info 值) ----
        self.fx = 612.0
        self.fy = 612.0
        self.cx = 323.0
        self.cy = 238.0

        # ---- JSON 檔案儲存 ----
        self.objects_json = {}  # 儲存所有辨識物件的座標
        self.json_path = '/home/hsiu/tmrdriver_ws/resource/json/keycap_coordinate.json'

    # ============================================================
    def cb_depth(self, msg: Image):
        """接收深度影像"""
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    # ============================================================
    def quaternion_to_matrix(self, qx, qy, qz, qw):
        """四元數轉換成 4x4 變換矩陣"""
        x, y, z, w = qx, qy, qz, qw
        T = np.array([
            [1 - 2 * (y**2 + z**2), 2 * (x*y - z*w),     2 * (x*z + y*w),     0],
            [2 * (x*y + z*w),       1 - 2 * (x**2 + z**2), 2 * (y*z - x*w),   0],
            [2 * (x*z - y*w),       2 * (y*z + x*w),     1 - 2 * (x**2 + y**2), 0],
            [0, 0, 0, 1]
        ])
        return T

    # ============================================================
    def cb_detections(self, msg: Detection2DArray):
        """接收 YOLO 偵測並轉換成 Base Link 座標"""
        if self.depth_image is None:
            self.get_logger().warn('等待深度影像...')
            return

        updated_objects = {}

        for det in msg.detections:
            if not det.results:
                continue

            # --- 取出 class id ---
            hyp = det.results[0].hypothesis
            cls_name = getattr(hyp, 'class_id', 'unknown')

            # --- 取得 bbox 中心像素座標 ---
            bbox = det.bbox
            cx = int(bbox.center.position.x)
            cy = int(bbox.center.position.y)

            # --- 檢查像素範圍 ---
            if (self.depth_image is None or
                cy < 0 or cy >= self.depth_image.shape[0] or
                cx < 0 or cx >= self.depth_image.shape[1]):
                continue

            # --- 取得深度 (公尺) ---
            depth = float(self.depth_image[cy, cx]) / 1000.0
            if depth <= 0.0:
                self.get_logger().warn(f'{cls_name} 深度無效 (0)')
                continue

            # --- 像素 → 相機座標 (右手座標系) ---
            X = (cx - self.cx) * depth / self.fx
            Y = (cy - self.cy) * depth / self.fy
            Z = depth
            pt_cam = np.array([X, Y, Z, 1.0])

            # --- 轉換到 base_link ---
            try:
                trans = self.tf_buffer.lookup_transform(
                    'base_link',
                    'camera_color_optical_frame',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5)
                )

                q = trans.transform.rotation
                t = trans.transform.translation
                T = self.quaternion_to_matrix(q.x, q.y, q.z, q.w)
                T[:3, 3] = [t.x, t.y, t.z]

                pt_base = T @ pt_cam

                updated_objects[cls_name] = {
                    "x": round(float(pt_base[0]), 3),
                    "y": round(float(pt_base[1]), 3),
                    "z": round(float(pt_base[2]), 3)
                }

                self.get_logger().info(
                    f"🔹 {cls_name}: base_link = ({pt_base[0]:.3f}, {pt_base[1]:.3f}, {pt_base[2]:.3f})"
                )

            except Exception as e:
                self.get_logger().error(f'TF lookup failed: {e}')
                continue

        # ---- 更新 JSON ----
        if updated_objects:
            self.objects_json.update(updated_objects)
            try:
                with open(self.json_path, 'w') as f:
                    json.dump(self.objects_json, f, indent=2)
                self.get_logger().info(
                    f"已更新 JSON：{len(self.objects_json)} 物件，路徑：{self.json_path}"
                )
            except Exception as e:
                self.get_logger().error(f"無法寫入 JSON：{e}")

# ============================================================
def main():
    rclpy.init()
    node = DetectionToBase()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
