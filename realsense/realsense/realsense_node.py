import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import pyrealsense2 as rs

class RealSenseNode(Node):
    def __init__(self):
        super().__init__('my_rs_camera')

        # ---- parameters ----
        self.declare_parameter('rgb_profile', '640x480x30')
        self.declare_parameter('depth_profile', '640x480x30')
        self.declare_parameter('serial_no', '')
        self.declare_parameter('align_depth', True)

        self.rgb_profile = self.get_parameter('rgb_profile').value
        self.depth_profile = self.get_parameter('depth_profile').value
        self.serial_no = self.get_parameter('serial_no').value
        self.align_depth = bool(self.get_parameter('align_depth').value)

        # ---- publishers ----
        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        self.pub_color = self.create_publisher(Image, '/camera/color/image_raw', sensor_qos)
        self.pub_depth = self.create_publisher(Image, '/camera/depth/image_rect_raw', sensor_qos)
        self.pub_cinfo = self.create_publisher(CameraInfo, '/camera/color/camera_info', 10)
        self.pub_dinfo = self.create_publisher(CameraInfo, '/camera/depth/camera_info', 10)

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

        self.get_logger().info(
            f'Intrinsics: fx={self.fx:.2f}, fy={self.fy:.2f}, '
            f'cx={self.cx:.2f}, cy={self.cy:.2f}'
        )

        # 主循環
        self.timer = self.create_timer(0.0, self.loop_once)

    def make_camera_info(self):
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
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
        except RuntimeError:
            self.get_logger().warn('No frames received from RealSense within 1s.')
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
            self.get_logger().warn('Missing depth or color frame.')
            return

        try:
            color_np = np.asanyarray(color.get_data())
            depth_np = np.asanyarray(depth.get_data())
        except Exception as e:
            self.get_logger().error(f'Failed to convert frame to numpy: {e}')
            return

        stamp = self.get_clock().now().to_msg()

        try:
            color_msg = self.bridge.cv2_to_imgmsg(color_np, encoding='bgr8')
            color_msg.header.stamp = stamp
            color_msg.header.frame_id = self.frame_id
            self.pub_color.publish(color_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish color frame: {e}')

        try:
            depth_msg = self.bridge.cv2_to_imgmsg(depth_np, encoding='16UC1')
            depth_msg.header.stamp = stamp
            depth_msg.header.frame_id = self.frame_id
            self.pub_depth.publish(depth_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish depth frame: {e}')

        try:
            cinfo = self.make_camera_info()
            cinfo.header.stamp = stamp
            dinfo = self.make_camera_info()
            dinfo.header.stamp = stamp
            self.pub_cinfo.publish(cinfo)
            self.pub_dinfo.publish(dinfo)
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