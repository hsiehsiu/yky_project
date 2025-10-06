#!/usr/bin/env python3
import math
import traceback
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from builtin_interfaces.msg import Time as RosTime

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, ObjectHypothesis

from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2


# ---------- helpers: vision_msgs 相容處理 ----------
def _get_center_xy(det: Detection2D) -> Tuple[float, float]:
    """
    取得 bbox.center 的 x,y，兼容 Pose2D(.x/.y) 或 Pose(.position.x/.y)。
    """
    c = det.bbox.center
    if hasattr(c, 'x') and hasattr(c, 'y'):
        return float(c.x), float(c.y)
    if hasattr(c, 'position'):
        return float(c.position.x), float(c.position.y)
    # fallback
    return 0.0, 0.0

def _get_label_and_score(hyp: ObjectHypothesisWithPose) -> Tuple[str, float]:
    """
    兼容新版 hypothesis.class_id/score 與舊版 id/score。
    """
    if hasattr(hyp, 'hypothesis') and isinstance(hyp.hypothesis, ObjectHypothesis):
        label = str(hyp.hypothesis.class_id)
        score = float(hyp.hypothesis.score) if hasattr(hyp.hypothesis, 'score') else 0.0
        return label, score
    # 舊版
    label = str(hyp.id) if hasattr(hyp, 'id') else ''
    score = float(hyp.score) if hasattr(hyp, 'score') else 0.0
    return label, score

def _ros_time_to_float(t: RosTime) -> float:
    return float(t.sec) + float(t.nanosec) * 1e-9


class YoloVisualization(Node):
    """
    訂閱：彩色影像 + /yolo/detections
    發佈：/yolo/visualization/image (已疊框/標籤)
    目的：在單一節點內，同時顯示 RealSense 畫面與 YOLO 的分析結果（以疊圖方式）。
    """
    def __init__(self):
        super().__init__('yolo_visualization')

        # ---------- 參數 ----------
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('detections_topic', '/yolo/detections')
        self.declare_parameter('out_image_topic', '/yolo/visualization/image')
        self.declare_parameter('sync_tolerance_ms', 120.0)   # 影像與偵測允許的時間差
        self.declare_parameter('draw_thickness', 2)          # 方框邊線粗細
        self.declare_parameter('font_scale', 0.5)            # 文字大小
        self.declare_parameter('min_score_to_draw', 0.0)     # 分數門檻（低於不畫）
        self.declare_parameter('box_color_bgr', [0, 255, 0]) # BGR

        self.image_topic = self.get_parameter('image_topic').value
        self.det_topic   = self.get_parameter('detections_topic').value
        self.out_topic   = self.get_parameter('out_image_topic').value

        self.sync_tol_s  = float(self.get_parameter('sync_tolerance_ms').value) / 1000.0
        self.thickness   = int(self.get_parameter('draw_thickness').value)
        self.font_scale  = float(self.get_parameter('font_scale').value)
        self.min_score   = float(self.get_parameter('min_score_to_draw').value)
        self.color_bgr   = self.get_parameter('box_color_bgr').value
        if not (isinstance(self.color_bgr, (list, tuple)) and len(self.color_bgr) == 3):
            self.color_bgr = [0, 255, 0]
        self.color_bgr = tuple(int(c) for c in self.color_bgr)

        # ---------- QoS ----------
        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        # ---------- bridge / buffers ----------
        self.bridge = CvBridge()

        self.last_img: Optional[np.ndarray] = None
        self.last_img_header = None  # 包含 stamp, frame_id
        self.last_img_time: Optional[float] = None

        self.last_det: Optional[Detection2DArray] = None
        self.last_det_time: Optional[float] = None

        # ---------- pubs & subs ----------
        try:
            self.sub_img = self.create_subscription(Image, self.image_topic, self.cb_image, sensor_qos)
            self.sub_det = self.create_subscription(Detection2DArray, self.det_topic, self.cb_dets, 10)
        except Exception as e:
            self.get_logger().fatal(f'Failed to create subscriptions: {e}\n{traceback.format_exc()}')
            raise

        try:
            self.pub_img = self.create_publisher(Image, self.out_topic, 10)
        except Exception as e:
            self.get_logger().fatal(f'Failed to create publisher: {e}\n{traceback.format_exc()}')
            raise

        self.get_logger().info(
            f'Listening image="{self.image_topic}", detections="{self.det_topic}" '
            f'-> out image="{self.out_topic}" (sync_tol={self.sync_tol_s*1000:.0f} ms)'
        )

    # ---------- Callbacks ----------
    def cb_image(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge color error: {e}')
            return
        except Exception as e:
            self.get_logger().error(f'cb_image convert error: {e}\n{traceback.format_exc()}')
            return

        self.last_img = img
        self.last_img_header = msg.header
        self.last_img_time = _ros_time_to_float(msg.header.stamp)
        self._try_render_and_publish()

    def cb_dets(self, msg: Detection2DArray):
        self.last_det = msg
        self.last_det_time = _ros_time_to_float(msg.header.stamp) if msg and msg.header else None
        self._try_render_and_publish()

    # ---------- Core ----------
    def _try_render_and_publish(self):
        """
        當影像與偵測都到齊，且時間差在 sync_tolerance 內，就疊圖並發佈。
        """
        if self.last_img is None or self.last_det is None:
            return
        if self.last_img_time is None or self.last_det_time is None:
            return

        dt = abs(self.last_img_time - self.last_det_time)
        if dt > self.sync_tol_s:
            # 時間差太大，等待更接近的資料
            return

        try:
            annotated = self._draw(self.last_img.copy(), self.last_det)
        except Exception as e:
            self.get_logger().error(f'draw error: {e}\n{traceback.format_exc()}')
            return

        try:
            out = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            # header 以影像為主（保留時間與 frame_id）
            if self.last_img_header:
                out.header = self.last_img_header
            self.pub_img.publish(out)
        except Exception as e:
            self.get_logger().error(f'publish annotated image failed: {e}\n{traceback.format_exc()}')

    def _draw(self, img: np.ndarray, det_arr: Detection2DArray) -> np.ndarray:
        H, W = img.shape[:2]
        color = self.color_bgr
        th = self.thickness
        fs = self.font_scale

        for det in det_arr.detections:
            cx, cy = _get_center_xy(det)
            w, h = float(det.bbox.size_x), float(det.bbox.size_y)
            x1, y1 = int(round(cx - w/2.0)), int(round(cy - h/2.0))
            x2, y2 = int(round(cx + w/2.0)), int(round(cy + h/2.0))

            # 邊界裁切，避免 OpenCV 畫圖報錯
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(W-1, x2), min(H-1, y2)
            if x2 <= x1 or y2 <= y1:
                continue

            # 文字（取第一個 hypothesis）
            label_txt = ''
            score = 0.0
            if det.results:
                label_txt, score = _get_label_and_score(det.results[0])

            if score < self.min_score:
                # 低於門檻不畫（可選）
                continue

            cv2.rectangle(img, (x1, y1), (x2, y2), color, th)
            text = f'{label_txt}:{score:.2f}' if label_txt != '' else f'{score:.2f}'
            ty = max(0, y1 - 5)
            cv2.putText(img, text, (x1, ty), cv2.FONT_HERSHEY_SIMPLEX, fs, color, max(1, th-1), cv2.LINE_AA)

        return img


def main():
    rclpy.init()
    node = YoloVisualization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().fatal(f'Unhandled exception: {e}\n{traceback.format_exc()}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
