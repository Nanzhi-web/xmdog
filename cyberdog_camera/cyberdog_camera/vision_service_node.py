import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading

from xmdog_msgs.srv import SetVisionMode
from xmdog_msgs.msg import VisionState
from .vision_logic import ball_logic, line_logic


class VisionServiceNode(Node):
    def __init__(self):
        super().__init__('vision_service_node')
        self.bridge = CvBridge()
        self.latest_cv_image = None
        self.image_lock = threading.Lock()

        # 当前工作的视觉模式，默认为 idle (什么都不做)
        self.current_mode = "idle"

        # 1. 订阅相机
        self.create_subscription(Image, '/rgb_camera/image_raw', self.image_callback, 5)

        # 2. 发布者：发布视觉处理结果
        self.state_pub = self.create_publisher(VisionState, '/vision/perception_state', 10)

        # 3. 服务端：供 FSM 切换视觉模式
        self.create_service(SetVisionMode, '/vision/set_mode', self.set_mode_callback)

        # 4. 定时器 (例如 10Hz)，只要不是 idle 就在跑
        self.timer = self.create_timer(0.1, self.vision_processing_loop)

        self.get_logger().info('视觉节点启动，当前处于 idle 待机模式。')

    def image_callback(self, msg):
        with self.image_lock:
            self.latest_cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def set_mode_callback(self, request, response):
        """FSM 调用此服务来切换视觉节点的工作模式"""
        valid_modes = ["stage_2", "stage_3", "stage_6", "default", "idle"]
        if request.mode in valid_modes:
            self.current_mode = request.mode
            response.success = True
            response.message = f"视觉模式已成功切换为: {self.current_mode}"
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = f"未知的模式: {request.mode}"
        return response

    def vision_processing_loop(self):
        """核心处理循环：根据 current_mode 决定执行哪些模型"""
        if self.current_mode == "idle":
            return  # 待机模式，不消耗算力

        with self.image_lock:
            if self.latest_cv_image is None:
                return
            frame = self.latest_cv_image.copy()

        msg = VisionState()

        # ================= 按模式分发计算力 =================
        if self.current_mode == "stage_2":
            # 【第二阶段：寻球】只跑黄线水平检测和最大球检测
            line_res = line_logic.check_horizontal(frame)
            msg.line_is_horizontal = line_res['is_horizontal']

            ball_res = ball_logic.detect_largest_ball(frame)
            msg.ball_found = ball_res['found']
            if msg.ball_found:
                msg.ball_color = ball_res['color']
                msg.ball_x_offset = ball_res['x_offset']
                msg.ball_size_ratio = ball_res['ratio']

        # elif self.current_mode == "stage_6":
        #     # 【第六阶段：足球追踪】只跑足球检测，要求极高帧率
        #     # 内部可以过滤，只找足球
        #     ball_res = ball_logic.detect_soccer(frame)
        #     msg.ball_found = ball_res['found']
        #     if msg.ball_found:
        #         msg.ball_x_offset = ball_res['x_offset']
        #
        # elif self.current_mode == "stage_3":
        #     # 【第三阶段：复杂场景】跑道线对齐 + 障碍物 + 目标
        #     line_res = line_logic.detect_yellow_lines(frame)
        #     msg.line_found = line_res['found']
        #     msg.line_center_offset = line_res['center_offset']
        #
        #     obs_res = obstacle_logic.detect_all(frame)
        #     msg.height_limit_danger = obs_res['height_limit_near']
        #     msg.obstacle_danger = obs_res['obstacle_near']
        #     msg.obstacle_distance = obs_res['distance']
        #
        #     target_res = target_logic.detect_specific_targets(frame)
        #     msg.target_found = target_res['found']
        #     if msg.target_found:
        #         msg.target_type = target_res['type']
        #         msg.target_distance = target_res['distance']

        elif self.current_mode == "default":
            # 【默认阶段：纯跑道对齐】只跑黄线提取，负担极小
            line_res = line_logic.detect_yellow_lines(frame)
            msg.line_found = line_res['found']
            msg.line_center_offset = line_res['center_offset']

        # 将当前算出的数据发布给 FSM
        self.state_pub.publish(msg)