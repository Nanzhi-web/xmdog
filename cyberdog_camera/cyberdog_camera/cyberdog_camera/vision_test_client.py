#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import Pose, Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from xmdog_msgs.srv import GetDistance

try:
    from xmdog_msgs.msg import VisionState
except ImportError:
    VisionState = None

try:
    from xmdog_msgs.srv import SetVisionMode
except ImportError:
    SetVisionMode = None


PRESET_POSES = {
    "default": (0.20, 5.20, 0.55, 1.57),
    "stage_2": (0.80, 0.25, 0.55, 1.57),
    "stage_3": (0.40, 8.80, 0.55, 1.57),
    "stage_6": (0.40, 13.20, 0.55, 1.57),
}


class VisionTestClient(Node):
    def __init__(self, args):
        super().__init__("vision_test_client")
        self.args = args
        self.latest_state = None
        self.nose_ready = False
        self.d435_color_ready = False
        self.d435_depth_ready = False

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._subscriptions = []
        self._subscriptions.append(self.create_subscription(Image, "/rgb_sensor/image_raw", self._nose_rgb_callback, qos))
        self._subscriptions.append(self.create_subscription(Image, "/image_raw", self._nose_rgb_callback, qos))
        self._subscriptions.append(self.create_subscription(Image, "/camera/image_raw", self._d435_color_callback, qos))
        self._subscriptions.append(self.create_subscription(Image, "/camera/color/image_raw", self._d435_color_callback, qos))
        self._subscriptions.append(self.create_subscription(Image, "/camera/depth/image_raw", self._d435_depth_callback, qos))
        self._subscriptions.append(self.create_subscription(Image, "/camera/depth/image_rect_raw", self._d435_depth_callback, qos))

        self.has_new_interface = VisionState is not None and SetVisionMode is not None
        self.mode_client = None
        if self.has_new_interface:
            self.mode_client = self.create_client(SetVisionMode, "/vision/set_mode")
            self._subscriptions.append(
                self.create_subscription(VisionState, "/vision/perception_state", self._vision_state_callback, 10)
            )
        else:
            self.get_logger().warn(
                "xmdog_msgs 中暂未找到 SetVisionMode/VisionState，本测试脚本将只能做原始图像/部分兼容服务测试。"
            )

        self.legacy_probe_client = None
        if not self.has_new_interface and self.args.mode == "stage_6":
            self.legacy_probe_client = self.create_client(GetDistance, "get_soccer_pixel_pos")
        elif not self.has_new_interface and self.args.mode == "stage_3":
            self.legacy_probe_client = self.create_client(GetDistance, "get_height_limit_distance")

        self.model_state_client = None
        for service_name in ("/gazebo/set_model_state", "/set_model_state"):
            client = self.create_client(SetModelState, service_name)
            if client.wait_for_service(timeout_sec=0.5):
                self.model_state_client = client
                self.get_logger().info(f"已连接 Gazebo 位姿服务: {service_name}")
                break

        self.create_timer(1.0, self._print_status)
        if self.legacy_probe_client is not None:
            self.create_timer(1.0, self._legacy_probe_timer)

    def _nose_rgb_callback(self, msg):
        del msg
        if not self.nose_ready:
            self.nose_ready = True
            self.get_logger().info("已收到鼻尖 RGB 图像流。")

    def _d435_color_callback(self, msg):
        del msg
        if not self.d435_color_ready:
            self.d435_color_ready = True
            self.get_logger().info("已收到 D435 彩色图像流。")

    def _d435_depth_callback(self, msg):
        del msg
        if not self.d435_depth_ready:
            self.d435_depth_ready = True
            self.get_logger().info("已收到 D435 深度图像流。")

    def _vision_state_callback(self, msg):
        self.latest_state = msg

    def _legacy_probe_timer(self):
        if self.legacy_probe_client is None or not self.legacy_probe_client.service_is_ready():
            return

        request = GetDistance.Request()
        future = self.legacy_probe_client.call_async(request)
        future.add_done_callback(self._handle_legacy_probe_result)

    def _handle_legacy_probe_result(self, future):
        try:
            result = future.result()
            if result is None:
                return
            if self.args.mode == "stage_6":
                self.get_logger().info(f"兼容探针(stage_6 足球像素): {result.distance}")
            elif self.args.mode == "stage_3":
                self.get_logger().info(f"兼容探针(stage_3 限高): {result.distance}")
        except Exception as exc:
            self.get_logger().warn(f"兼容探针调用失败: {exc}")

    def teleport_robot(self, x, y, z, yaw, model_name):
        if self.model_state_client is None:
            self.get_logger().warn("当前未发现 Gazebo SetModelState 服务，跳过传送。")
            return False

        request = SetModelState.Request()
        state = ModelState()
        state.model_name = model_name
        state.pose = Pose()
        state.pose.position.x = float(x)
        state.pose.position.y = float(y)
        state.pose.position.z = float(z)
        state.pose.orientation.z = math.sin(yaw / 2.0)
        state.pose.orientation.w = math.cos(yaw / 2.0)
        state.twist = Twist()
        state.reference_frame = "world"
        request.model_state = state

        future = self.model_state_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        result = future.result()
        if result is None:
            self.get_logger().error("机器人传送失败: Gazebo 未返回结果。")
            return False

        if result.success:
            self.get_logger().info(f"机器人已传送到 x={x:.2f}, y={y:.2f}, z={z:.2f}, yaw={yaw:.2f}")
            return True

        self.get_logger().error(f"机器人传送失败: {result.status_message}")
        return False

    def set_mode(self, mode_name):
        if not self.has_new_interface or self.mode_client is None:
            self.get_logger().warn("当前环境无法调用 /vision/set_mode。")
            return False

        if not self.mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("/vision/set_mode 服务未就绪。")
            return False

        request = SetVisionMode.Request()
        request.mode = mode_name
        future = self.mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        result = future.result()
        if result is None:
            self.get_logger().error("切换视觉模式失败: 无返回结果。")
            return False

        if result.success:
            self.get_logger().info(result.message)
            return True

        self.get_logger().error(result.message)
        return False

    def _print_status(self):
        sensor_status = (
            f"nose_rgb={'OK' if self.nose_ready else 'WAIT'} "
            f"d435_color={'OK' if self.d435_color_ready else 'WAIT'} "
            f"d435_depth={'OK' if self.d435_depth_ready else 'WAIT'}"
        )

        if self.latest_state is None:
            self.get_logger().info(f"[Raw Streams] {sensor_status}")
            return

        msg = self.latest_state
        summary = (
            f"mode={self.args.mode} "
            f"line_found={msg.line_found} "
            f"line_offset={msg.line_center_offset:.3f} "
            f"line_horizontal={msg.line_is_horizontal} "
            f"ball_found={msg.ball_found} "
            f"ball_color={msg.ball_color} "
            f"ball_offset={msg.ball_x_offset:.3f} "
            f"ball_ratio={msg.ball_size_ratio:.3f} "
            f"height_limit={msg.height_limit_danger} "
            f"obstacle={msg.obstacle_danger} "
            f"obstacle_distance={msg.obstacle_distance:.2f} "
            f"target_found={msg.target_found} "
            f"target_type={msg.target_type} "
            f"target_distance={msg.target_distance:.2f}"
        )
        self.get_logger().info(f"[VisionState] {summary} | {sensor_status}")


def parse_args(argv):
    parser = argparse.ArgumentParser(description="CyberDog vision test client")
    parser.add_argument("--mode", default="default", choices=["default", "stage_2", "stage_3", "stage_6", "idle"])
    parser.add_argument("--teleport", default="", help="使用预设传送机器人，如 stage_2/stage_3/stage_6/default")
    parser.add_argument("--x", type=float, default=None)
    parser.add_argument("--y", type=float, default=None)
    parser.add_argument("--z", type=float, default=None)
    parser.add_argument("--yaw", type=float, default=None)
    parser.add_argument("--model-name", default="robot")
    return parser.parse_known_args(argv)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    args, ros_args = parse_args(argv)

    rclpy.init(args=ros_args)
    node = VisionTestClient(args)

    try:
        if args.teleport:
            if args.teleport not in PRESET_POSES:
                node.get_logger().error(f"未知 teleport 预设: {args.teleport}")
            else:
                preset_x, preset_y, preset_z, preset_yaw = PRESET_POSES[args.teleport]
                node.teleport_robot(preset_x, preset_y, preset_z, preset_yaw, args.model_name)
        elif None not in (args.x, args.y, args.yaw):
            z = 0.55 if args.z is None else args.z
            node.teleport_robot(args.x, args.y, z, args.yaw, args.model_name)

        if args.mode != "idle":
            node.set_mode(args.mode)

        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
