import os
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
import numpy as np
import cv2

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        # ---------- params ----------
        self.declare_parameter('model_path', '/home/hsiu/tmrdriver_ws/src/yolo/resource/yolo_model.pt')
        self.declare_parameter('conf', 0.5)
        self.declare_parameter('estimate_3d', True)
        self.declare_parameter('color_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')

        p = self.get_parameter
        self.model_path = p('model_path').value
        self.conf = float(p('conf').value)
        self.estimate_3d = bool(p('estimate_3d').value)
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
            self.get_logger().warn(f'conf={self.conf} out of [0,1], clamping.')
            self.conf = max(0.0, min(1.0, self.conf))

        # ---------- YOLO ----------
        try:
            self.model = YOLO(self.model_path)
        except Exception as e:
            self.get_logger().fatal(f'Failed to load YOLO model: {e}')
            raise
        self.class_names = None
        try:
            # ultralytics 模型通常有 names 屬性
            self.class_names = self.model.model.names if hasattr(self.model, 'model') else None
        except Exception:
            self.class_names = None
        self.get_logger().info(f'Loaded YOLO: {self.model_path} (conf={self.conf})')

        # ---------- QoS ----------
        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        self.bridge = CvBridge()
        self.last_depth = None       # np.ndarray (H,W) 16UC1
        self.last_depth_shape = None
        self.intrinsics = None       # (fx, fy, cx, cy)

        # one-time warn flags to避免洗版
        self.warned_no_intr = False
        self.warned_no_depth = False
        self.warned_depth_shape = False

        # ---------- subs & pubs ----------
        try:
            self.sub_color = self.create_subscription(Image, self.color_topic, self.cb_color, sensor_qos)
            self.sub_depth = self.create_subscription(Image, self.depth_topic, self.cb_depth, sensor_qos)
            self.sub_info  = self.create_subscription(CameraInfo, self.camera_info_topic, self.cb_info, 10)
        except Exception as e:
            self.get_logger().fatal(f'Failed to create subscriptions: {e}')
            raise

        try:
            self.pub_det = self.create_publisher(Detection2DArray, '/yolo/detections', 10)
            self.pub_annot = self.create_publisher(Image, '/yolo/image_annotated', 10)
        except Exception as e:
            self.get_logger().fatal(f'Failed to create publishers: {e}')
            raise

    # -------------------- Callbacks --------------------
    def cb_info(self, msg: CameraInfo):
        try:
            if len(msg.k) < 6:
                if not self.warned_no_intr:
                    self.get_logger().warn('CameraInfo.k has insufficient length; skip setting intrinsics.')
                    self.warned_no_intr = True
                return
            fx, fy, cx, cy = msg.k[0], msg.k[4], msg.k[2], msg.k[5]
            if any(not math.isfinite(v) for v in (fx, fy, cx, cy)) or fx <= 0 or fy <= 0:
                if not self.warned_no_intr:
                    self.get_logger().warn(f'Camera intrinsics invalid: fx={fx}, fy={fy}, cx={cx}, cy={cy}')
                    self.warned_no_intr = True
                return
            self.intrinsics = (float(fx), float(fy), float(cx), float(cy))
            self.warned_no_intr = False  # reset if good info later
        except Exception as e:
            self.get_logger().error(f'cb_info error: {e}')

    def cb_depth(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if depth is None:
                if not self.warned_no_depth:
                    self.get_logger().warn('Depth conversion returned None.')
                    self.warned_no_depth = True
                return
            if depth.ndim != 2:
                if not self.warned_depth_shape:
                    self.get_logger().warn(f'Unexpected depth shape {depth.shape}, expect (H,W).')
                    self.warned_depth_shape = True
                return
            self.last_depth = depth
            self.last_depth_shape = depth.shape
            self.warned_no_depth = False
            self.warned_depth_shape = False
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error converting depth: {e}')
        except Exception as e:
            self.get_logger().error(f'cb_depth error: {e}')

    def cb_color(self, msg: Image):
        # 只有彩色影像觸發推論
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error converting color: {e}')
            return
        except Exception as e:
            self.get_logger().error(f'cb_color convert error: {e}')
            return

        # YOLO 推論
        try:
            results = self.model.predict(frame, conf=self.conf, verbose=False)
        except Exception as e:
            self.get_logger().error(f'YOLO inference failed: {e}')
            return

        det_arr = Detection2DArray()
        det_arr.header = msg.header

        annot = frame.copy()
        # 檢查 3D 需求是否具備
        can_3d = self.estimate_3d and (self.last_depth is not None) and (self.intrinsics is not None)
        if self.estimate_3d and not can_3d:
            if self.intrinsics is None and not self.warned_no_intr:
                self.get_logger().warn('estimate_3d=True but no CameraInfo received yet.')
                self.warned_no_intr = True
            if self.last_depth is None and not self.warned_no_depth:
                self.get_logger().warn('estimate_3d=True but no depth frames received yet.')
                self.warned_no_depth = True

        fx = fy = cx = cy = None
        if self.intrinsics is not None:
            fx, fy, cx, cy = self.intrinsics

        try:
            for r in results:
                if not hasattr(r, 'boxes') or r.boxes is None:
                    continue
                for b in r.boxes:
                    try:
                        xyxy = b.xyxy[0].tolist() if hasattr(b, 'xyxy') else None
                        confv = float(b.conf[0]) if hasattr(b, 'conf') else 0.0
                        clsv = int(b.cls[0]) if hasattr(b, 'cls') else -1
                        if xyxy is None or len(xyxy) != 4:
                            self.get_logger().warn('Invalid bbox format from YOLO; skipping one box.')
                            continue
                        x1, y1, x2, y2 = map(float, xyxy)
                        cx2d = (x1 + x2) / 2.0
                        cy2d = (y1 + y2) / 2.0
                        w = max(0.0, x2 - x1)
                        h = max(0.0, y2 - y1)

                        det = Detection2D()
                        det.bbox.center.x = cx2d
                        det.bbox.center.y = cy2d
                        det.bbox.size_x = w
                        det.bbox.size_y = h

                        hyp = ObjectHypothesisWithPose()
                        # id 用類別索引或名稱
                        if self.class_names and 0 <= clsv < len(self.class_names):
                            hyp.id = str(self.class_names[clsv])
                        else:
                            hyp.id = str(clsv)
                        hyp.score = confv

                        if can_3d and fx and fy:
                            # 取 bbox 中心的深度值（你也可以改成 bbox 區域中位數）
                            px, py = int(round(cx2d)), int(round(cy2d))
                            H, W = self.last_depth_shape if self.last_depth_shape else self.last_depth.shape
                            if 0 <= px < W and 0 <= py < H:
                                d_raw = float(self.last_depth[py, px])
                                # 假設是毫米；若你的 RealSense depth 單位非毫米，請改這裡
                                depth_m = d_raw / 1000.0 if d_raw > 0 else 0.0
                                if depth_m > 0.0 and math.isfinite(depth_m):
                                    X = (px - cx) / fx * depth_m
                                    Y = (py - cy) / fy * depth_m
                                    Z = depth_m
                                    hyp.pose.pose.position.x = X
                                    hyp.pose.pose.position.y = Y
                                    hyp.pose.pose.position.z = Z
                                else:
                                    # 不報錯，只是不填 pose
                                    pass
                            else:
                                self.get_logger().debug(
                                    f'Center pixel ({px},{py}) out of depth bounds ({W}x{H}).'
                                )

                        det.results.append(hyp)
                        det_arr.detections.append(det)

                        # 標註圖（若沒人訂閱也無妨）
                        label = hyp.id if isinstance(hyp.id, str) else str(hyp.id)
                        cv2.rectangle(annot, (int(x1), int(y1)), (int(x2), int(y2)), (0,255,0), 2)
                        cv2.putText(annot, f'{label}:{confv:.2f}', (int(x1), int(y1)-5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1, cv2.LINE_AA)
                    except Exception as ibox:
                        self.get_logger().error(f'Error handling one detection: {ibox}')
                        continue
        except Exception as e:
            self.get_logger().error(f'Failed to parse YOLO results: {e}')
            return

        # 發布
        try:
            self.pub_det.publish(det_arr)
        except Exception as e:
            self.get_logger().error(f'Failed to publish /yolo/detections: {e}')

        try:
            if self.pub_annot.get_subscription_count() > 0:
                out = self.bridge.cv2_to_imgmsg(annot, encoding='bgr8')
                out.header = msg.header
                self.pub_annot.publish(out)
        except Exception as e:
            self.get_logger().error(f'Failed to publish /yolo/image_annotated: {e}')

def main():
    rclpy.init()
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().fatal(f'Unhandled exception in node: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()
'''模型載入前檢查：model_path 是否存在，不存在直接 fatal + raise。

conf 範圍檢查：自動 clamp 到 [0,1]，並 warn。

建立 sub/pub 失敗：fatal。

CameraInfo：K 長度與數值有效性檢查；無效只 warn 並略過。

Depth 影像：CvBridge 轉換錯誤、shape 非 (H,W) 皆會 warn/error。

YOLO 推論：任何例外會 error 並略過該幀。

結果解析：每個 box 做 try/except，單箱出錯不影響其它。

3D 估測：缺少 intrinsics/depth 只 warn 一次（避免洗版）；像素越界、深度無效則略過不報錯。

發布失敗：針對兩個 publisher 各自 try/except。'''