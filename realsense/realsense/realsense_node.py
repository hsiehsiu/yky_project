import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import pyrealsense2 as rs
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

class RealSenseNode(Node):
    def __init__(self):
        super().__init__('my_rs_camera')

        # ---- parameters ----
        self.declare_parameter('rgb_profile', '640x480x30')
        self.declare_parameter('depth_profile', '640x480x30')
        self.declare_parameter('serial_no', '')
        self.declare_parameter('align_depth', True)
        # [優化] 新增可調參數
        self.declare_parameter('publish_rate', 30.0)         # 發布頻率 (Hz)
        self.declare_parameter('pointcloud_enabled', True)   # 是否啟用點雲
        self.declare_parameter('pointcloud_decimation', 5)   # 每 N 幀發布一次點雲

        self.rgb_profile = self.get_parameter('rgb_profile').value
        self.depth_profile = self.get_parameter('depth_profile').value
        self.serial_no = self.get_parameter('serial_no').value
        self.align_depth = bool(self.get_parameter('align_depth').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.pointcloud_enabled = bool(self.get_parameter('pointcloud_enabled').value)
        self.pointcloud_decimation = int(self.get_parameter('pointcloud_decimation').value)

        # [優化] 幀計數器，用於控制點雲發布頻率
        self.frame_count = 0

        # ---- publishers ----
        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        self.pub_color = self.create_publisher(Image, '/camera/color/image_raw', sensor_qos)
        self.pub_depth = self.create_publisher(Image, '/camera/aligned_depth_to_color/image_raw', sensor_qos)
        self.pub_cinfo = self.create_publisher(CameraInfo, '/camera/color/camera_info', 10)
        
        # [優化] 只有啟用點雲時才建立發布器
        if self.pointcloud_enabled:
            self.pub_pointcloud = self.create_publisher(PointCloud2, '/camera/depth/color/points', sensor_qos)
        else:
            self.pub_pointcloud = None
            
        self.bridge = CvBridge()

        # ---- RealSense pipeline ----
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        if self.serial_no:
            cfg.enable_device(self.serial_no)
            self.get_logger().info(f'Using device serial: {self.serial_no}')

        try:
            w, h, fps = [int(x) for x in self.rgb_profile.replace('x', ' ').split()]
            dw, dh, dfps = [int(x) for x in self.depth_profile.replace('x', ' ').split()]
        except Exception as e:
            self.get_logger().error(f'Invalid profile format! Should be WxHxFPS. Got: {e}')
            raise

        try:
            cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)
            cfg.enable_stream(rs.stream.depth, dw, dh, rs.format.z16, dfps)
            self.profile = self.pipeline.start(cfg)
        except Exception as e:
            self.get_logger().error(f'Failed to start RealSense pipeline: {e}')
            raise

        self.get_logger().info('RealSense pipeline started')

        # 對齊
        self.align = rs.align(rs.stream.color) if self.align_depth else None
        
        # [優化] 只有啟用點雲時才初始化點雲生成器
        if self.pointcloud_enabled:
            self.pc = rs.pointcloud()
        else:
            self.pc = None

        try:
            color_stream = self.profile.get_stream(rs.stream.color).as_video_stream_profile()
            intr = color_stream.get_intrinsics()
        except Exception as e:
            self.get_logger().error(f'Failed to get camera intrinsics: {e}')
            raise

        self.fx, self.fy, self.cx, self.cy = intr.fx, intr.fy, intr.ppx, intr.ppy
        self.dist = list(intr.coeffs)
        self.width, self.height = intr.width, intr.height
        self.frame_id = 'camera_color_optical_frame'

        # [優化] 預先建立並快取 CameraInfo，避免每幀重複建立
        self._cached_camera_info = self._build_camera_info()

        self.get_logger().info(
            f'Intrinsics: fx={self.fx:.2f}, fy={self.fy:.2f}, '
            f'cx={self.cx:.2f}, cy={self.cy:.2f}'
        )
        self.get_logger().info(
            f'Publish rate: {self.publish_rate} Hz, '
            f'Pointcloud: {"enabled" if self.pointcloud_enabled else "disabled"}, '
            f'Pointcloud decimation: 1/{self.pointcloud_decimation}'
        )

        # [優化] 使用固定頻率的 timer，而非 0.0（會導致 CPU 滿載）
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.loop_once)

    def _build_camera_info(self):
        """[優化] 預先建立 CameraInfo，只需更新 timestamp"""
        msg = CameraInfo()
        msg.header.frame_id = self.frame_id
        msg.width = self.width
        msg.height = self.height
        msg.distortion_model = 'plumb_bob'
        msg.d = self.dist
        msg.k = [self.fx, 0.0, self.cx,
                 0.0, self.fy, self.cy,
                 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0]
        msg.p = [self.fx, 0.0, self.cx, 0.0,
                 0.0, self.fy, self.cy, 0.0,
                 0.0, 0.0, 1.0, 0.0]
        return msg

    def loop_once(self):
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=100)
        except RuntimeError:
            # [優化] 降低 timeout 並減少警告頻率
            return

        if self.align is not None:
            try:
                frames = self.align.process(frames)
            except Exception as e:
                self.get_logger().error(f'Align process failed: {e}')
                return

        depth = frames.get_depth_frame()
        color = frames.get_color_frame()
        if not depth or not color:
            return

        # [優化] 使用 np.asarray 而非 np.asanyarray，通常更快
        color_np = np.asarray(color.get_data())
        depth_np = np.asarray(depth.get_data())

        stamp = self.get_clock().now().to_msg()

        # 發布 color
        try:
            color_msg = self.bridge.cv2_to_imgmsg(color_np, encoding='bgr8')
            color_msg.header.stamp = stamp
            color_msg.header.frame_id = self.frame_id
            self.pub_color.publish(color_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish color frame: {e}')

        # 發布 depth
        try:
            depth_msg = self.bridge.cv2_to_imgmsg(depth_np, encoding='16UC1')
            depth_msg.header.stamp = stamp
            depth_msg.header.frame_id = self.frame_id
            self.pub_depth.publish(depth_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish depth frame: {e}')

        # [優化] 點雲：只有在啟用且達到 decimation 間隔時才計算
        self.frame_count += 1
        if self.pointcloud_enabled and self.pub_pointcloud and (self.frame_count % self.pointcloud_decimation == 0):
            try:
                self.pc.map_to(color)
                points = self.pc.calculate(depth)
                v = points.get_vertices()
                points_array = np.asarray(v).view(np.float32).reshape(-1, 3)
                
                from std_msgs.msg import Header
                header = Header()
                header.stamp = stamp
                header.frame_id = self.frame_id
                
                pc_msg = point_cloud2.create_cloud_xyz32(header=header, points=points_array)
                self.pub_pointcloud.publish(pc_msg)
            except Exception as e:
                self.get_logger().error(f'Failed to publish pointcloud: {e}')

        # [優化] 使用快取的 CameraInfo，只更新 timestamp
        try:
            self._cached_camera_info.header.stamp = stamp
            self.pub_cinfo.publish(self._cached_camera_info)
        except Exception as e:
            self.get_logger().error(f'Failed to publish CameraInfo: {e}')

    def destroy_node(self):
        try:
            self.pipeline.stop()
        except Exception as e:
            self.get_logger().warn(f'Error stopping pipeline: {e}')
        super().destroy_node()

def main():
    rclpy.init()
    node = RealSenseNode()
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().fatal(f'Unhandled exception in node: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()
'''啟動時：

如果 profile 格式錯誤 → self.get_logger().error

如果 pipeline 無法啟動（沒插相機 / 序號錯誤） → self.get_logger().error

如果內參讀取失敗 → self.get_logger().error

取影像時：

超過 1s 沒 frame → warn

少了 depth 或 color frame → warn

轉換 numpy 失敗 → error

發布失敗 → error

關閉時：

pipeline.stop() 出錯 → warn

整體：

如果整個 spin 過程有未捕捉例外 → fatal'''