#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
2026 视觉服务节点。

主架构:
1. 通过 SetVisionMode 服务切换视觉模式。
2. 通过 VisionState 统一发布当前模式的感知结果。
3. 保留少量兼容服务，方便现有测试脚本继续访问足球像素和限高结果。
"""

import threading
from functools import partial

import rclpy
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

from xmdog_msgs.srv import GetDistance

from .vision_logic import ball_logic, line_logic, obstacle_logic, target_logic

try:
    from xmdog_msgs.msg import VisionState
except ImportError:
    VisionState = None

try:
    from xmdog_msgs.srv import SetVisionMode
except ImportError:
    SetVisionMode = None


VALID_MODES = {"stage_2", "stage_3", "stage_6", "default", "idle"}


class VisionServiceNode(Node):
    def __init__(self):
        super().__init__("vision_service_node")
        self.bridge = CvBridge()
        self.current_mode = "idle"

        self.frame_lock = threading.Lock()
        self.state_lock = threading.Lock()
        self.latest_frames = {
            "nose_rgb": None,
            "d435_color": None,
            "d435_depth": None,
        }
        self.latest_camera_info = {
            "nose_rgb": None,
            "d435_color": None,
            "d435_depth": None,
        }
        self.latest_pointcloud = None
        self.latest_state_cache = self._empty_state_cache()

        self._logged_streams = set()
        self._missing_sensor_warnings = set()
        self._subscriptions = []

        self.get_logger().info("视觉服务节点启动，准备加载 2026 架构。")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._create_sensor_subscriptions(qos)

        self.state_pub = None
        self.mode_service = None
        if VisionState is not None and SetVisionMode is not None:
            self.state_pub = self.create_publisher(VisionState, "/vision/perception_state", 10)
            self.mode_service = self.create_service(SetVisionMode, "/vision/set_mode", self.set_mode_callback)
            self.get_logger().info("已启用 2026 接口: /vision/set_mode + /vision/perception_state")
        else:
            self.get_logger().warn(
                "当前 xmdog_msgs 中尚未找到 SetVisionMode/VisionState，"
                "节点将以兼容模式运行；若要完整启用 2026 接口，请先补齐消息定义。"
            )

        self._create_legacy_services()

        self.timer = self.create_timer(0.1, self.vision_processing_loop)
        self.get_logger().info("视觉节点已进入 idle 模式，等待 FSM 或测试脚本切换。")

    def _empty_state_cache(self):
        return {
            "line_found": False,
            "line_center_offset": 0.0,
            "line_is_horizontal": False,
            "ball_found": False,
            "ball_color": "",
            "ball_x_offset": 0.0,
            "ball_size_ratio": 0.0,
            "height_limit_danger": False,
            "obstacle_danger": False,
            "obstacle_distance": 99.9,
            "target_found": False,
            "target_type": "none",
            "target_distance": 99.9,
        }

    def _create_sensor_subscriptions(self, qos):
        self._subscribe_aliases(Image, ["/rgb_sensor/image_raw", "/image_raw"], partial(self._color_image_callback, "nose_rgb"), qos)
        self._subscribe_aliases(CameraInfo, ["/rgb_sensor/camera_info", "/camera_info"], partial(self._camera_info_callback, "nose_rgb"), qos)

        self._subscribe_aliases(Image, ["/camera/image_raw", "/camera/color/image_raw"], partial(self._color_image_callback, "d435_color"), qos)
        self._subscribe_aliases(CameraInfo, ["/camera/camera_info", "/camera/color/camera_info"], partial(self._camera_info_callback, "d435_color"), qos)

        self._subscribe_aliases(Image, ["/camera/depth/image_raw", "/camera/depth/image_rect_raw"], partial(self._depth_image_callback, "d435_depth"), qos)
        self._subscribe_aliases(CameraInfo, ["/camera/depth/camera_info"], partial(self._camera_info_callback, "d435_depth"), qos)

        self._subscribe_aliases(PointCloud2, ["/camera/points", "/camera/depth/color/points"], self._pointcloud_callback, qos)

    def _subscribe_aliases(self, msg_type, topics, callback, qos):
        for topic in dict.fromkeys(topics):
            subscription = self.create_subscription(msg_type, topic, callback, qos)
            self._subscriptions.append(subscription)
            self.get_logger().info(f"已订阅: {topic}")

    def _log_stream_ready_once(self, stream_name):
        if stream_name in self._logged_streams:
            return
        self._logged_streams.add(stream_name)
        self.get_logger().info(f"已收到 {stream_name} 数据流。")

    def _warn_missing_once(self, warning_key, message):
        if warning_key in self._missing_sensor_warnings:
            return
        self._missing_sensor_warnings.add(warning_key)
        self.get_logger().warn(message)

    def _color_image_callback(self, source_name, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self.frame_lock:
                self.latest_frames[source_name] = frame
            self._log_stream_ready_once(source_name)
        except Exception as exc:
            self.get_logger().error(f"{source_name} 图像转换失败: {exc}")

    def _depth_image_callback(self, source_name, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            with self.frame_lock:
                self.latest_frames[source_name] = frame
            self._log_stream_ready_once(source_name)
        except Exception as exc:
            self.get_logger().error(f"{source_name} 深度图转换失败: {exc}")

    def _camera_info_callback(self, source_name, msg):
        with self.frame_lock:
            self.latest_camera_info[source_name] = msg

    def _pointcloud_callback(self, msg):
        with self.frame_lock:
            self.latest_pointcloud = msg
        self._log_stream_ready_once("d435_pointcloud")

    def _get_latest_frame(self, *source_names):
        with self.frame_lock:
            for source_name in source_names:
                frame = self.latest_frames.get(source_name)
                if frame is not None:
                    return frame.copy()
        return None

    def _update_state_cache(self, state_dict):
        with self.state_lock:
            self.latest_state_cache = dict(state_dict)

    def _publish_state(self, state_dict):
        if self.state_pub is None or VisionState is None:
            return

        msg = VisionState()
        msg.line_found = state_dict["line_found"]
        msg.line_center_offset = float(state_dict["line_center_offset"])
        msg.line_is_horizontal = state_dict["line_is_horizontal"]
        msg.ball_found = state_dict["ball_found"]
        msg.ball_color = state_dict["ball_color"]
        msg.ball_x_offset = float(state_dict["ball_x_offset"])
        msg.ball_size_ratio = float(state_dict["ball_size_ratio"])
        msg.height_limit_danger = state_dict["height_limit_danger"]
        msg.obstacle_danger = state_dict["obstacle_danger"]
        msg.obstacle_distance = float(state_dict["obstacle_distance"])
        msg.target_found = state_dict["target_found"]
        msg.target_type = state_dict["target_type"]
        msg.target_distance = float(state_dict["target_distance"])
        self.state_pub.publish(msg)

    def set_mode_callback(self, request, response):
        if request.mode not in VALID_MODES:
            response.success = False
            response.message = f"未知视觉模式: {request.mode}"
            self.get_logger().warn(response.message)
            return response

        self.current_mode = request.mode
        response.success = True
        response.message = f"视觉模式已切换为: {self.current_mode}"
        self.get_logger().info(response.message)
        return response

    def vision_processing_loop(self):
        if self.current_mode == "idle":
            return

        state = self._empty_state_cache()

        if self.current_mode == "stage_2":
            frame = self._get_latest_frame("nose_rgb", "d435_color")
            if frame is None:
                self._warn_missing_once("stage_2_rgb", "stage_2 正在等待 RGB 图像。")
                return

            line_result = line_logic.check_horizontal(frame)
            ball_result = ball_logic.detect_largest_ball(frame)

            state["line_is_horizontal"] = line_result["is_horizontal"]
            state["line_center_offset"] = line_result["center_offset"]
            state["ball_found"] = ball_result["found"]
            state["ball_color"] = ball_result["color"]
            state["ball_x_offset"] = ball_result["x_offset"]
            state["ball_size_ratio"] = ball_result["ratio"]

        elif self.current_mode == "stage_3":
            color_frame = self._get_latest_frame("nose_rgb", "d435_color")

            if color_frame is None:
                self._warn_missing_once("stage_3_color", "stage_3 正在等待 RGB 彩色图像。")
            else:
                line_result = line_logic.detect_yellow_lines(color_frame)
                target_result = target_logic.detect_specific_targets(color_frame)
                obstacle_result = obstacle_logic.detect_all(color_frame)
                state["line_found"] = line_result["found"]
                state["line_center_offset"] = line_result["center_offset"]
                state["target_found"] = target_result["found"]
                state["target_type"] = target_result["type"]
                state["target_distance"] = target_result["distance"]
                state["height_limit_danger"] = obstacle_result["height_limit_near"]
                state["obstacle_danger"] = obstacle_result["obstacle_near"]
                state["obstacle_distance"] = obstacle_result["distance"]

        elif self.current_mode == "stage_6":
            depth_frame = self._get_latest_frame("d435_depth")
            if depth_frame is None:
                self._warn_missing_once("stage_6_depth", "stage_6 正在等待 D435 深度图像。")
                return

            soccer_result = ball_logic.detect_soccer_depth(depth_frame)
            state["ball_found"] = soccer_result["found"]
            state["ball_color"] = "soccer" if soccer_result["found"] else ""
            state["ball_x_offset"] = soccer_result["x_offset"]

        elif self.current_mode == "default":
            frame = self._get_latest_frame("nose_rgb", "d435_color")
            if frame is None:
                self._warn_missing_once("default_rgb", "default 模式正在等待黄线图像。")
                return

            line_result = line_logic.detect_yellow_lines(frame)
            state["line_found"] = line_result["found"]
            state["line_center_offset"] = line_result["center_offset"]

        self._update_state_cache(state)
        self._publish_state(state)

    def _create_legacy_services(self):
        self.create_service(GetDistance, "get_height_limit_distance", self.get_height_limit_distance_callback)
        self.create_service(GetDistance, "get_soccer_pixel_pos", self.get_soccer_pixel_pos_callback)
        self.get_logger().info("已启用精简兼容服务: /get_height_limit_distance + /get_soccer_pixel_pos")

    def get_soccer_pixel_pos_callback(self, request, response):
        del request
        depth_frame = self._get_latest_frame("d435_depth")
        if depth_frame is None:
            response.success = False
            response.distance = ""
            return response

        details = ball_logic.detect_soccer_depth_details(depth_frame)
        if not details["found"]:
            response.success = False
            response.distance = ""
            return response

        response.success = True
        response.distance = f"{int(details['center_x'])},{int(details['center_y'])},{int(details['radius'])}"
        return response

    def get_height_limit_distance_callback(self, request, response):
        del request

        color_frame = self._get_latest_frame("nose_rgb", "d435_color")
        if color_frame is not None:
            obstacle_result = obstacle_logic.detect_all(color_frame)
            response.success = True
            response.distance = "near" if obstacle_result["height_limit_near"] else "far"
            return response

        response.success = False
        response.distance = "far"
        return response


def main(args=None):
    rclpy.init(args=args)
    vision_node = None
    executor = None

    try:
        vision_node = VisionServiceNode()
        executor = MultiThreadedExecutor()
        executor.add_node(vision_node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        if vision_node is not None:
            vision_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
