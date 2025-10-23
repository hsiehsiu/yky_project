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

        # ---- 訂閱 YOLO 偵測 ----
        self.create_subscription(Detection2DArray, '/yolo/detections', self.cb_detections, 10)

        # ---- 相機內參 (請改成你的 camera_info) ----
        self.fx = 612.0
        self.fy = 612.0
        self.cx = 323.0
        self.cy = 238.0

        # 儲存物件座標，保留上一次辨識的座標
        self.objects_json = {}

        # JSON 路徑
        self.json_path = '/home/hsiu/tmrdriver_ws/resource/json/keycap_coordinate.json'

    def cb_depth(self, msg: Image):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def quaternion_to_matrix(self, qx, qy, qz, qw):
        """ 將四元數轉換為 4x4 變換矩陣 """
        x, y, z, w = qx, qy, qz, qw
        T = np.array([
            [1-2*y*y-2*z*z, 2*x*y-2*z*w, 2*x*z+2*y*w, 0],
            [2*x*y+2*z*w, 1-2*x*x-2*z*z, 2*y*z-2*x*w, 0],
            [2*x*z-2*y*w, 2*y*z+2*x*w, 1-2*x*x-2*y*y, 0],
            [0,0,0,1]
        ])
        return T

    def cb_detections(self, msg: Detection2DArray):
        if self.depth_image is None:
            self.get_logger().warn('等待深度影像...')
            return

        # 暫存本次偵測更新的物件
        updated_objects = {}

        for det in msg.detections:
            if not det.results:
                continue
            hyp = det.results[0]
            cls_name = getattr(hyp, 'id', 'unknown')

            # bbox 中心點
            bbox = det.bbox
            cx = int(bbox.center.x)
            cy = int(bbox.center.y)

            # 深度 (meter)
            depth = float(self.depth_image[cy, cx]) / 1000.0
            if depth <= 0.0:
                continue

            # ---- 像素座標 → 相機座標 ----
            X = (cx - self.cx) * depth / self.fx
            Y = (cy - self.cy) * depth / self.fy
            Z = depth
            pt_cam = np.array([X, Y, Z, 1.0])

            # ---- 轉換到 base_link ----
            try:
                trans = self.tf_buffer.lookup_transform('base_link', 'camera_color_optical_frame', rclpy.time.Time())
                q = trans.transform.rotation
                t = trans.transform.translation
                T = self.quaternion_to_matrix(q.x, q.y, q.z, q.w)
                T[:3, 3] = [t.x, t.y, t.z]
                pt_base = T @ pt_cam

                # 更新本次偵測到的物件
                updated_objects[cls_name] = {
                    "x": round(float(pt_base[0]), 3),
                    "y": round(float(pt_base[1]), 3),
                    "z": round(float(pt_base[2]), 3)
                }

            except Exception as e:
                self.get_logger().error(f'TF lookup failed: {e}')
                continue

        # ---- 合併上一次的座標 (保留沒偵測到的物件) ----
        self.objects_json.update(updated_objects)

        # ---- 寫入 JSON ----
        if self.objects_json:
            with open(self.json_path, 'w') as f:
                json.dump(self.objects_json, f, indent=2)
            self.get_logger().info(f"已更新 JSON：{len(self.objects_json)} 物件，路徑：{self.json_path}")

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
