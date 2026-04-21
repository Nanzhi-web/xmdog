import rclpy
from xmdog_msgs.msg import VisionState
from xmdog_msgs.srv import SetVisionMode
from .state import State


class TrackAlignmentState(State):
    def __init__(self, node):
        super().__init__("TrackAlignment")
        self.node = node
        self.latest_vision = None

        # 1. 创建视觉数据订阅器
        self.vision_sub = self.node.create_subscription(
            VisionState,
            '/vision/perception_state',
            self.vision_callback,
            10
        )

        # 2. 创建模式切换客户端
        self.mode_client = self.node.create_client(SetVisionMode, '/vision/set_mode')

    def vision_callback(self, msg):
        """缓存最新的视觉处理结果"""
        self.latest_vision = msg

    def activate_vision_mode(self, mode_name):
        """调用服务切换视觉模式"""
        if not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().error('Vision service not available')
            return

        req = SetVisionMode.Request()
        req.mode = mode_name  # 如 "default" 或 "stage_3" 触发黄线检测
        self.mode_client.call_async(req)

    def execute(self, locomotion_controller):
        # 进入状态时，通知视觉节点启动黄线检测逻辑
        self.activate_vision_mode("default")

        while rclpy.ok():
            if self.latest_vision and self.latest_vision.line_found:
                # 获取黄线中点偏移量：负数偏左，正数偏右
                offset = self.latest_vision.line_center_offset

                # --- 控制算法 (例如比例控制 P) ---
                kp_yaw = -0.5  # 转向增益
                forward_speed = 0.3  # 基础前进速度

                # 计算目标转向速度，使 offset 趋向 0
                yaw_speed = offset * kp_yaw

                # 下发给运动控制模块
                locomotion_controller.set_vel(forward_speed, 0.0, yaw_speed)
            else:
                # 未发现线时的安全逻辑
                locomotion_controller.set_vel(0.0, 0.0, 0.0)

            # 配合 FSM 调度频率
            rclpy.spin_once(self.node, timeout_sec=0.01)