#!/usr/bin/env python3
import os
import math
import traceback
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, ObjectHypothesis
from geometry_msgs.msg import Pose2D as GeoPose2D
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
import numpy as np
import cv2

# --------------------- helpers ---------------------
def _as_list_xyxy(x):
    """將 1x4 tensor/ndarray/list 安全轉為 [x1,y1,x2,y2] 的 list[float]（支援 CUDA tensor）。"""
    if x is None:
        return None
    if hasattr(x, 'detach'):  # torch tensor（含 CUDA）
        x = x.detach().cpu().numpy()
    if hasattr(x, 'tolist'):
        x = x.tolist()
    if isinstance(x, (list, tuple)) and len(x) == 1 and isinstance(x[0], (list, tuple)):
        x = x[0]
    return list(map(float, x)) if isinstance(x, (list, tuple)) and len(x) == 4 else None

def _as_float0(t):
    """將單元素張量/數值安全轉成 float（支援 CUDA tensor）。"""
    if t is None:
        return 0.0
    if hasattr(t, 'item'):
        try:
            return float(t.item())
        except Exception:
            pass
    try:
        return float(t)
    except Exception:
        return 0.0

def _as_int0(t, default=-1):
    """將單元素張量/數值安全轉成 int（支援 CUDA tensor）。"""
    if t is None:
        return default
    if hasattr(t, 'item'):
        try:
            return int(t.item())
        except Exception:
            pass
    try:
        return int(t)
    except Exception:
        return default

def _set_bbox_center(bbox, x, y):
    """
    多版本相容地設定 BoundingBox2D.center：
      - geometry_msgs/Pose2D: center.x, center.y, center.theta
      - geometry_msgs/Pose:   center.position.x, center.position.y
    若現有 center 型別不符，會回填一個 Pose2D。
    """
    try:
        c = bbox.center
    except Exception:
        bbox.center = GeoPose2D()
        bbox.center.x = float(x)
        bbox.center.y = float(y)
        if hasattr(bbox.center, 'theta'):
            bbox.center.theta = 0.0
        return

    if hasattr(c, 'x') and hasattr(c, 'y'):
        try:
            c.x = float(x)
            c.y = float(y)
            if hasattr(c, 'theta') and c.theta is None:
                c.theta = 0.0
            return
        except Exception:
            pass

    if hasattr(c, 'position') and hasattr(c.position, 'x') and hasattr(c.position, 'y'):
        try:
            c.position.x = float(x)
            c.position.y = float(y)
            return
        except Exception:
            pass

    try:
        bbox.center = GeoPose2D()
        bbox.center.x = float(x)
        bbox.center.y = float(y)
        if hasattr(bbox.center, 'theta'):
            bbox.center.theta = 0.0
    except Exception:
        pass

def _set_hypothesis(hyp: ObjectHypothesisWithPose, cls_text: str, score: float):
    """
    兼容兩種版型：
      舊版：hyp.id, hyp.score
      新版：hyp.hypothesis.class_id, hyp.hypothesis.score
    """
    # 新版：有 hypothesis 欄位
    if hasattr(hyp, 'hypothesis') and isinstance(hyp.hypothesis, ObjectHypothesis):
        hyp.hypothesis.class_id = str(cls_text)
        hyp.hypothesis.score = float(score)
        return
    # 舊版：頂層 id/score
    if hasattr(hyp, 'id'):
        hyp.id = str(cls_text)
    if hasattr(hyp, 'score'):
        hyp.score = float(score)

def _get_hypothesis_id(hyp: ObjectHypothesisWithPose) -> str:
    """從 hyp 中取出 class id（兼容新/舊版）。"""
    if hasattr(hyp, 'hypothesis') and hasattr(hyp.hypothesis, 'class_id'):
        return str(hyp.hypothesis.class_id)
    if hasattr(hyp, 'id'):
        return str(hyp.id)
    return ""

def _get_hypothesis_score(hyp: ObjectHypothesisWithPose) -> float:
    """從 hyp 中取出 score（兼容新/舊版）。"""
    if hasattr(hyp, 'hypothesis') and hasattr(hyp.hypothesis, 'score'):
        return float(hyp.hypothesis.score)
    if hasattr(hyp, 'score'):
        return float(hyp.score)
    return 0.0

# --------------------- node ---------------------
class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        # ---------- params ----------
        self.declare_parameter('model_path', '/home/hsiu/tmrdriver_ws/src/yolo/resource/yolo_model.pt')
        self.declare_parameter('conf', 0.5)
        self.declare_parameter('estimate_3d', True)
        self.declare_parameter('depth_in_meters', False)  # True: 深度單位 m(32FC1)；False: mm(16UC1)
        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')

        p = self.get_parameter
        self.model_path = p('model_path').value
        self.conf = float(p('conf').value)
        self.estimate_3d = bool(p('estimate_3d').value)
        self.depth_in_meters = bool(p('depth_in_meters').value)
        self.color_topic = p('color_topic').value
        self.depth_topic = p('depth_topic').value
        self.camera_info_topic = p('camera_info_topic').value

        # ---------- sanity checks ----------
        if not isinstance(self.model_path, str) or not self.model_path:
            self.get_logger().fatal('Parameter "model_path" is empty.')
            raise ValueError('model_path is empty')
        if not os.path.exists(self.model_path):
            self.get_logger().fatal(f'Model file not found: {self.model_path}')
            raise FileNotFoundError(self.model_path)
        if not (0.0 <= self.conf <= 1.0):
            self.get_logger().warning(f'conf={self.conf} out of [0,1], clamping.')
            self.conf = max(0.0, min(1.0, self.conf))

        # ---------- YOLO ----------
        try:
            self.model = YOLO(self.model_path)
        except Exception as e:
            self.get_logger().fatal(f'Failed to load YOLO model: {e}\n{traceback.format_exc()}')
            raise

        # 取得類別名稱表（統一成 {int: str}），兼容多版本/多型別
        def _get_names(m):
            cand = None
            if hasattr(m, 'names'):
                cand = m.names
            elif hasattr(m, 'model') and hasattr(m.model, 'names'):
                cand = m.model.names
            if isinstance(cand, dict):
                return {int(k): str(v) for k, v in cand.items()}
            if isinstance(cand, (list, tuple)):
                return {i: str(v) for i, v in enumerate(cand)}
            return None

        self.class_names = _get_names(self.model)
        if not self.class_names:
            self.get_logger().warning('Could not read class names from YOLO model; will publish numeric IDs as text.')

        self.get_logger().info(f'Loaded YOLO: {self.model_path} (conf={self.conf})')

        # ---------- QoS ----------
        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        self.bridge = CvBridge()
        self.last_depth = None       # np.ndarray (H,W)
        self.last_depth_shape = None
        self.intrinsics = None       # (fx, fy, cx, cy)

        # one-time warn flags
        self.warned_no_intr = False
        self.warned_no_depth = False
        self.warned_depth_shape = False

        # ---------- subs & pubs ----------
        try:
            self.sub_color = self.create_subscription(Image, self.color_topic, self.cb_color, sensor_qos)
            self.sub_depth = self.create_subscription(Image, self.depth_topic, self.cb_depth, sensor_qos)
            self.sub_info  = self.create_subscription(CameraInfo, self.camera_info_topic, self.cb_info, 10)
        except Exception as e:
            self.get_logger().fatal(f'Failed to create subscriptions: {e}\n{traceback.format_exc()}')
            raise

        try:
            self.pub_det = self.create_publisher(Detection2DArray, '/yolo/detections', 10)
            self.pub_annot = self.create_publisher(Image, '/yolo/image_annotated', 10)
        except Exception as e:
            self.get_logger().fatal(f'Failed to create publishers: {e}\n{traceback.format_exc()}')
            raise

    # -------------------- Callbacks --------------------
    def cb_info(self, msg: CameraInfo):
        try:
            # K 應有 9 個元素（3x3）
            if len(msg.k) < 9:
                if not self.warned_no_intr:
                    self.get_logger().warning('CameraInfo.k has insufficient length; skip setting intrinsics.')
                    self.warned_no_intr = True
                return
            fx, fy, cx, cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]
            if any((not math.isfinite(v)) for v in (fx, fy, cx, cy)) or fx <= 0 or fy <= 0:
                if not self.warned_no_intr:
                    self.get_logger().warning(f'Camera intrinsics invalid: fx={fx}, fy={fy}, cx={cx}, cy={cy}')
                    self.warned_no_intr = True
                return
            self.intrinsics = (float(fx), float(fy), float(cx), float(cy))
            self.warned_no_intr = False  # reset if good info later
        except Exception as e:
            self.get_logger().error(f'cb_info error: {e}\n{traceback.format_exc()}')

    def cb_depth(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if depth is None:
                if not self.warned_no_depth:
                    self.get_logger().warning('Depth conversion returned None.')
                    self.warned_no_depth = True
                return
            if depth.ndim != 2:
                if not self.warned_depth_shape:
                    self.get_logger().warning(f'Unexpected depth shape {depth.shape}, expect (H,W).')
                    self.warned_depth_shape = True
                return
            self.last_depth = depth
            self.last_depth_shape = depth.shape
            self.warned_no_depth = False
            self.warned_depth_shape = False
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error converting depth: {e}\n{traceback.format_exc()}')
        except Exception as e:
            self.get_logger().error(f'cb_depth error: {e}\n{traceback.format_exc()}')

    def cb_color(self, msg: Image):
        # 只有彩色影像觸發推論
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error converting color: {e}\n{traceback.format_exc()}')
            return
        except Exception as e:
            self.get_logger().error(f'cb_color convert error: {e}\n{traceback.format_exc()}')
            return

        # YOLO 推論
        try:
            results = self.model.predict(frame, conf=self.conf, verbose=False)
        except Exception as e:
            self.get_logger().error(f'YOLO inference failed: {e}\n{traceback.format_exc()}')
            return

        det_arr = Detection2DArray()
        det_arr.header = msg.header

        annot = frame.copy()

        # 檢查 3D 需求是否具備
        can_3d = self.estimate_3d and (self.last_depth is not None) and (self.intrinsics is not None)
        if self.estimate_3d and not can_3d:
            if self.intrinsics is None and not self.warned_no_intr:
                self.get_logger().warning('estimate_3d=True but no CameraInfo received yet.')
                self.warned_no_intr = True
            if self.last_depth is None and not self.warned_no_depth:
                self.get_logger().warning('estimate_3d=True but no depth frames received yet.')
                self.warned_no_depth = True

        fx = fy = cx = cy = None
        if self.intrinsics is not None:
            fx, fy, cx, cy = self.intrinsics

        # 解析 YOLO 結果
        try:
            for r in results:
                if not hasattr(r, 'boxes') or r.boxes is None:
                    continue
                for b in r.boxes:
                    try:
                        # ---- 取值（兼容 CUDA/CPU tensor, numpy, list）----
                        xyxy = _as_list_xyxy(b.xyxy[0] if hasattr(b, 'xyxy') else None)
                        confv = _as_float0(b.conf[0] if hasattr(b, 'conf') else None)
                        clsv = _as_int0(b.cls[0] if hasattr(b, 'cls') else None, default=-1)

                        if xyxy is None or len(xyxy) != 4:
                            self.get_logger().warning('Invalid bbox format from YOLO; skipping one box.')
                            continue

                        x1, y1, x2, y2 = map(float, xyxy)
                        # 保證正確順序與非負寬高
                        x1, x2 = sorted((x1, x2))
                        y1, y2 = sorted((y1, y2))
                        w = max(0.0, x2 - x1)
                        h = max(0.0, y2 - y1)
                        cx2d = (x1 + x2) * 0.5
                        cy2d = (y1 + y2) * 0.5

                        # ---- 填 Detection2D 與 Hypothesis ----
                        det = Detection2D()
                        _set_bbox_center(det.bbox, cx2d, cy2d)
                        det.bbox.size_x = w
                        det.bbox.size_y = h

                        # 取得類別文字（名稱優先，取不到時用數字字串）
                        if self.class_names and clsv in self.class_names:
                            cls_text = self.class_names[clsv]
                        else:
                            cls_text = str(clsv)

                        hyp = ObjectHypothesisWithPose()
                        _set_hypothesis(hyp, cls_text, confv)  # -> hypothesis.class_id = 名稱（或數字字串）

                        # 也把名稱放到 det.id（若該欄位存在，方便某些下游直接讀 det.id）
                        if hasattr(det, 'id'):
                            det.id = cls_text

                        # ---- 3D 反投影（需要 fx,fy,cx,cy 與 depth）----
                        if can_3d and fx and fy:
                            try:
                                px, py = int(round(cx2d)), int(round(cy2d))
                                H, W = (self.last_depth_shape if self.last_depth_shape
                                        else self.last_depth.shape[:2])

                                if 0 <= px < W and 0 <= py < H:
                                    d_raw = float(self.last_depth[py, px])
                                    if self.depth_in_meters:
                                        depth_m = d_raw if math.isfinite(d_raw) and d_raw > 0 else 0.0
                                    else:
                                        # 16UC1（mm）-> m
                                        depth_m = (d_raw / 1000.0) if math.isfinite(d_raw) and d_raw > 0 else 0.0

                                    if depth_m > 0.0:
                                        Z = depth_m
                                        X = (px - cx) * Z / fx
                                        Y = (py - cy) * Z / fy
                                        hyp.pose.pose.position.x = float(X)
                                        hyp.pose.pose.position.y = float(Y)
                                        hyp.pose.pose.position.z = float(Z)
                                else:
                                    self.get_logger().debug(
                                        f'Center pixel ({px},{py}) out of depth bounds ({W}x{H}).'
                                    )
                            except Exception:
                                self.get_logger().debug('Depth back-projection failed:\n' + traceback.format_exc())

                        det.results.append(hyp)
                        det_arr.detections.append(det)

                        # ---- 視覺標註（若有人訂閱就畫；沒人訂閱也無妨）----
                        if annot is not None:
                            label = _get_hypothesis_id(hyp)   # 現在會是「類別名稱」
                            score_show = _get_hypothesis_score(hyp)
                            cv2.rectangle(annot, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                            ty = max(0, int(y1) - 5)
                            cv2.putText(annot, f'{label}:{score_show:.2f}', (int(x1), ty),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

                    except Exception as ibox:
                        self.get_logger().error(
                            f'Error handling one detection: {ibox}\n{traceback.format_exc()}'
                        )
                        continue
        except Exception as e:
            self.get_logger().error(f'Failed to parse YOLO results: {e}\n{traceback.format_exc()}')
            return

        # 發布
        try:
            self.pub_det.publish(det_arr)
        except Exception as e:
            self.get_logger().error(f'Failed to publish /yolo/detections: {e}\n{traceback.format_exc()}')

        try:
            if self.pub_annot.get_subscription_count() > 0:
                out = self.bridge.cv2_to_imgmsg(annot, encoding='bgr8')
                out.header = msg.header
                self.pub_annot.publish(out)
        except Exception as e:
            self.get_logger().error(f'Failed to publish /yolo/image_annotated: {e}\n{traceback.format_exc()}')

def main():
    rclpy.init()
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().fatal(f'Unhandled exception in node: {e}\n{traceback.format_exc()}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

'''模型載入前檢查：model_path 是否存在，不存在直接 fatal + raise。

conf 範圍檢查：自動 clamp 到 [0,1]，並 warn。

建立 sub/pub 失敗：fatal。

CameraInfo：K 長度與數值有效性檢查；無效只 warn 並略過。

Depth 影像：CvBridge 轉換錯誤、shape 非 (H,W) 皆會 warn/error。

YOLO 推論：任何例外會 error 並略過該幀。

結果解析：每個 box 做 try/except，單箱出錯不影響其它。

3D 估測：缺少 intrinsics/depth 只 warn 一次（避免洗版）；像素越界、深度無效則略過不報錯。

發布失敗：針對兩個 publisher 各自 try/except。'''
