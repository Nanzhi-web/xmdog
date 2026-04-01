#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading
import cv2
import os
import time
import subprocess
from datetime import datetime

# 导入 .srv 文件
from xmdog_msgs.srv import GetTarget, GetPath, GetState, GetDistance

# 导入常规模块 (确保这些在 Python 3.6 下没问题)
from .vision_logic import qr_logic, arrow_logic, height_logic

class VisionServiceNode(Node):
  
    def __init__(self):
        super().__init__('vision_service_node')
        self.get_logger().info('视觉服务节点正在初始化 (Python 3.6 模式)...')
        self.bridge = CvBridge()
        self.latest_cv_image = None
        self.image_lock = threading.Lock()
        self.latest_image_timestamp = 0
        
        # 路径配置
        self.save_dir = "/home/mi/src/cyberdog_camera/saved_images"
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        # 核心：指向你的 SD 卡虚拟环境
        self.yolo_python_bin = "/SDCARD/conda_envs/dog_env/bin/python"
        self.light_logic_script = "/home/mi/src/cyberdog_camera/cyberdog_camera/vision_logic/light_logic.py"

        # 1. 订阅
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.image_sub = self.create_subscription(Image, '/image_rgb', self.image_callback, qos)

        # 2. 服务
        self.srv_a_target = self.create_service(GetTarget, 'get_a_area_target', self.get_a_area_target_callback)
        self.srv_b_target = self.create_service(GetTarget, 'get_b_area_target', self.get_b_area_target_callback)
        self.srv_height_limit = self.create_service(GetDistance, 'get_height_limit_distance', self.get_height_limit_distance_callback)
        self.srv_yellow_light = self.create_service(GetState, 'get_yellow_light_state', self.get_yellow_light_state_callback)
        self.srv_path_choice = self.create_service(GetPath, 'get_path_choice', self.get_path_choice_callback)
            
        self.get_logger().info('视觉服务节点已就绪。')

    def image_callback(self, msg):
        try:
            with self.image_lock:
                self.latest_cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                self.latest_image_timestamp = int(time.time() * 1000)
        except Exception as e:
            self.get_logger().error('图像转换失败: %s' % str(e))

    def get_latest_image_safely(self):
        with self.image_lock:
            if self.latest_cv_image is None:
                return None
            # 10秒失效校验
            if (int(time.time() * 1000) - self.latest_image_timestamp) > 10000:
                return None
            return self.latest_cv_image.copy()

    def get_yellow_light_state_callback(self, request, response):
        """黄灯检测：跨环境调用 Python 3.8"""
        self.get_logger().info('收到黄灯请求，启动子进程...')
        image = self.get_latest_image_safely()
        if image is None:
            response.success = False
            response.state = "Light Off"
            return response

        temp_img = "/tmp/yolo_input.jpg"
        cv2.imwrite(temp_img, image)

        try:
            # 执行子进程
            process = subprocess.Popen(
                [self.yolo_python_bin, self.light_logic_script, temp_img],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True
            )
            stdout, stderr = process.communicate(timeout=30)

            # 打印子进程里的所有 DEBUG 信息
            if stdout:
                for line in stdout.strip().split('\n'):
                    self.get_logger().info('子进程: %s' % line)
            if stderr:
                self.get_logger().error('子进程错误: %s' % stderr)

            if process.returncode == 0:
                res_lines = stdout.strip().split('\n')
                response.state = res_lines[-1] if res_lines else "Light Off"
                response.success = True
            else:
                response.success = False
                response.state = "Light Off"

        except Exception as e:
            self.get_logger().error('跨环境调用异常: %s' % str(e))
            response.success = False
            response.state = "Light Off"

        return response

    # 二维码、箭头、限高回调保持逻辑简单
    def get_a_area_target_callback(self, request, response):
        img = self.get_latest_image_safely()
        res = qr_logic.detect_target_from_qr(img) if img is not None else None
        response.success = True if res else False
        response.target = res.upper() if res else ""
        return response

    def get_b_area_target_callback(self, request, response):
        img = self.get_latest_image_safely()
        res = qr_logic.detect_target_from_qr(img) if img is not None else None
        response.success = True if res else False
        response.target = res.upper() if res else ""
        return response

    def get_path_choice_callback(self, request, response):
        img = self.get_latest_image_safely()
        res, _ = arrow_logic.detect_arrow_direction(img) if img is not None else ("stone", None)
        response.success = True
        response.path = res
        return response

    def get_height_limit_distance_callback(self, request, response):
        img = self.get_latest_image_safely()
        res, _ = height_logic.detect_height_distance(img) if img is not None else ("far", None)
        response.success = True
        response.distance = res
        return response

def main(args=None):
    rclpy.init(args=args)
    node = VisionServiceNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()