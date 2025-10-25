#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
import numpy as np
import tf2_ros
from geometry_msgs.msg import PointStamped

class DetectionToBase(Node):
    def __init__(self):
        super().__init__('transfer_node')

        # ---- YOLO detections ----
        self.subscription = self.create_subscription(
            Detection2DArray,
            '/yolo/detections',
            self.detection_callback,
            10)

        # ---- Depth image ----
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10)

        # ---- TF ----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.bridge = CvBridge()
        self.depth_image = None

        # RealSense intrinsics (根據你的相機改成實際值)
        self.fx = 616.0
        self.fy = 616.0
        self.cx = 320.0
        self.cy = 240.0

    def depth_callback(self, msg):
        """保存深度影像"""
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def detection_callback(self, msg):
        if self.depth_image is None:
            self.get_logger().info("等待深度影像中...")
            return

        for det in msg.detections:
            # --- 取出物件類別 ---
            class_id = "unknown"
            if len(det.results) > 0:
                class_id = det.results[0].hypothesis.class_id

            # --- 取出中心像素座標 ---
            u = int(det.bbox.center.position.x)
            v = int(det.bbox.center.position.y)

            # --- 取得該像素深度 ---
            if v < 0 or v >= self.depth_image.shape[0] or u < 0 or u >= self.depth_image.shape[1]:
                continue
            depth = self.depth_image[v, u] / 1000.0  # 轉成公尺 (若原為mm)

            if depth == 0:
                self.get_logger().warn(f"{class_id} 的深度無效 (0)")
                continue

            # --- 像素轉相機座標系 (Xc, Yc, Zc) ---
            Xc = (u - self.cx) * depth / self.fx
            Yc = (v - self.cy) * depth / self.fy
            Zc = depth

            # --- 建立 PointStamped (相機座標) ---
            point_cam = PointStamped()
            point_cam.header = msg.header
            point_cam.header.frame_id = "camera_color_optical_frame"
            point_cam.point.x = Xc
            point_cam.point.y = Yc
            point_cam.point.z = Zc

            try:
                # --- 轉換成 base_link ---
                transform = self.tf_buffer.lookup_transform(
                    'base_link',  # 目標座標系
                    point_cam.header.frame_id,  # 來源座標系
                    rclpy.time.Time())

                from tf2_geometry_msgs import do_transform_point
                point_base = do_transform_point(point_cam, transform)

                self.get_logger().info(
                    f"🔹 {class_id}: base_link座標 = "
                    f"({point_base.point.x:.3f}, {point_base.point.y:.3f}, {point_base.point.z:.3f})"
                )

            except Exception as e:
                self.get_logger().warn(f"TF 轉換失敗: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DetectionToBase()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
