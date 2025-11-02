#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import json
from datetime import datetime, timezone, timedelta

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
import tf2_ros
from builtin_interfaces.msg import Time as RosTime
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, ObjectHypothesis
# (在 transfer.py 的最上方)

def _get_hypothesis_id(hyp: "ObjectHypothesisWithPose") -> str: # noqa
    """從 hyp 中取出 class id（兼容新/舊版）。"""
    if hasattr(hyp, 'hypothesis') and hasattr(hyp.hypothesis, 'class_id'):
        return str(hyp.hypothesis.class_id)
    if hasattr(hyp, 'id'):
        return str(hyp.id)
    return ""

def _get_hypothesis_score(hyp: "ObjectHypothesisWithPose") -> float: # noqa
    """從 hyp 中取出 score（兼容新/舊版）。"""
    if hasattr(hyp, 'hypothesis') and hasattr(hyp.hypothesis, 'score'):
        return float(hyp.hypothesis.score)
    if hasattr(hyp, 'score'):
        return float(hyp.score)
    return 0.0
def _tz_taipei_iso(t: RosTime) -> str:
    tz = timezone(timedelta(hours=8))
    return datetime.fromtimestamp(t.sec + t.nanosec * 1e-9, tz=tz).isoformat(timespec='milliseconds')


def _ensure_parent(file_path: str):
    os.makedirs(os.path.dirname(file_path), exist_ok=True)


def _depth_at(depth_msg: Image, u: int, v: int, bridge: CvBridge):
    if u < 0 or v < 0 or u >= depth_msg.width or v >= depth_msg.height:
        return None
    try:
        dimg = bridge.imgmsg_to_cv2(depth_msg)
        val = dimg[int(v), int(u)]
    except Exception:
        return None
    enc = (depth_msg.encoding or "").upper()
    if enc in ("16UC1", "16UC"):
        if val == 0:
            return None
        return float(val) / 1000.0  # mm -> m
    elif enc in ("32FC1", "TYPE_32FC1"):
        if np.isnan(val) or val <= 0.0:
            return None
        return float(val)
    else:
        try:
            return float(val) if val > 0 else None
        except Exception:
            return None


def _deproject(K, u, v, z):
    fx, cx = K[0], K[2]
    fy, cy = K[4], K[5]
    X = (u - cx) * z / fx
    Y = (v - cy) * z / fy
    return np.array([X, Y, z], dtype=np.float64)


def _quat_to_R(qx, qy, qz, qw):
    qx2, qy2, qz2 = qx*qx, qy*qy, qz*qz
    return np.array([
        [1-2*(qy2+qz2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw), 1-2*(qx2+qz2), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1-2*(qx2+qy2)]
    ], dtype=np.float64)


def _transform_point(T, p_cam):
    q = T.transform.rotation
    t = T.transform.translation
    R = _quat_to_R(q.x, q.y, q.z, q.w)
    return R @ p_cam + np.array([t.x, t.y, t.z], dtype=np.float64)


class TransferNode(Node):
    def __init__(self):
        super().__init__('transfer')

        # -------- 參數 --------
        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('detections_topic', '/yolo/detections')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('keymap_file', '/home/hsiu/tmrdriver_ws/resource/json/keycap_coordinate.json')
        self.declare_parameter('units', 'mm')           # 'mm' or 'm'
        self.declare_parameter('lowercase_labels', True)
        self.declare_parameter('tf_timeout_sec', 0.08)

        # 讀參數
        self.rgb_topic = self.get_parameter('rgb_topic').get_parameter_value().string_value
        self.depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        self.info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.det_topic = self.get_parameter('detections_topic').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self.keymap_file = self.get_parameter('keymap_file').get_parameter_value().string_value
        self.units = self.get_parameter('units').get_parameter_value().string_value.lower()
        self.lowercase = bool(self.get_parameter('lowercase_labels').get_parameter_value().bool_value)
        self.tf_timeout = float(self.get_parameter('tf_timeout_sec').get_parameter_value().double_value)

        _ensure_parent(self.keymap_file)

        # -------- QoS 同步 --------
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=5
        )
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=10
        )

        self.bridge = CvBridge()
        self.K = None

        self.sub_rgb = Subscriber(self, Image, self.rgb_topic, qos_profile=sensor_qos)
        self.sub_depth = Subscriber(self, Image, self.depth_topic, qos_profile=sensor_qos)
        self.sub_info = Subscriber(self, CameraInfo, self.info_topic, qos_profile=reliable_qos)
        self.sub_det = Subscriber(self, Detection2DArray, self.det_topic, qos_profile=reliable_qos)

        self.sync = ApproximateTimeSynchronizer(
            [self.sub_rgb, self.sub_depth, self.sub_info, self.sub_det],
            queue_size=20, slop=0.12, allow_headerless=False
        )
        self.sync.registerCallback(self._cb)

        # -------- TF2 --------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info(
            f"[transfer] Subscribing:\n"
            f"  {self.rgb_topic}\n  {self.depth_topic}\n  {self.info_topic}\n  {self.det_topic}\n"
            f"TF: {self.base_frame} <- {self.camera_frame}\n"
            f"Keymap: {self.keymap_file} (units={self.units})"
        )

    # 讀/寫 keymap 檔（dict 結構）
    def _load_keymap(self):
        if not os.path.exists(self.keymap_file):
            return {}
        try:
            with open(self.keymap_file, 'r', encoding='utf-8') as f:
                obj = json.load(f)
                return obj if isinstance(obj, dict) else {}
        except Exception:
            return {}

    def _save_keymap(self, data: dict):
        tmp = self.keymap_file + ".tmp"
        with open(tmp, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        os.replace(tmp, self.keymap_file)

    def _label_of(self, det) -> str:
        label = ""
        if det.results:
            # 使用正確的輔助函數來找最高分
            best = max(det.results, key=_get_hypothesis_score)
            # 使用正確的輔助函數來提取 ID
            label = _get_hypothesis_id(best)

        # Fallback (這個 fallback 很好，analysis.py 也有設定 det.id，請保留)
        if not label and hasattr(det, 'id') and det.id:
            label = str(det.id)
        
        return label.lower() if self.lowercase else label

    def _cb(self, rgb_msg: Image, depth_msg: Image, info_msg: CameraInfo, det_msg: Detection2DArray):
        if info_msg.k and len(info_msg.k) == 9:
            self.K = info_msg.k
        else:
            self.get_logger().warn("CameraInfo.K invalid; skip frame.")
            return

        stamp = rgb_msg.header.stamp
        try:
            T = self.tf_buffer.lookup_transform(
                self.base_frame, self.camera_frame, stamp,
                rclpy.duration.Duration(seconds=self.tf_timeout)
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return

        keymap = self._load_keymap()
        updated = 0

        for det in det_msg.detections:
            # 取 bbox 中心像素
            try:
                cx = det.bbox.center.position.x
                cy = det.bbox.center.position.y
            except Exception:
                continue
            u, v = int(round(cx)), int(round(cy))

            # 深度（m）
            z = _depth_at(depth_msg, u, v, self.bridge)
            if z is None or z <= 0:
                continue

            # 反投影到相機座標 → 轉 base_link
            p_cam = _deproject(self.K, u, v, z)
            try:
                p_base = _transform_point(T, p_cam)  # m
            except Exception:
                continue

            label = self._label_of(det)
            if not label:
                continue

            if self.units == 'mm':
                x, y, z_out = (p_base * 1000.0).tolist()
            else:
                x, y, z_out = p_base.tolist()

            keymap[label] = {
                "x": round(float(x), 3),
                "y": round(float(y), 3),
                "z": round(float(z_out), 3)
            }
            updated += 1

        if updated > 0:
            keymap["_meta"] = {
                "last_update": _tz_taipei_iso(stamp),
                "frame": self.base_frame,
                "source": "transfer",
                "units": self.units
            }
            try:
                self._save_keymap(keymap)
                self.get_logger().info(f"Updated {updated} keys → {self.keymap_file}")
            except Exception as e:
                self.get_logger().error(f"Write JSON failed: {e}")


def main():
    rclpy.init()
    node = TransferNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
