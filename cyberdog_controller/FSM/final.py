#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FSM 主节点（使用 /clock 作为仿真时间来源）
保存路径: FSM/final.py
"""
from .states import *
import rclpy
from rclpy.node import Node
import threading
import time
from rosgraph_msgs.msg import Clock as RosClock

# 导入所有需要的 .srv 文件
from xmdog_msgs.srv import GetTarget, GetDistance, GetState, GetPath


class FINAL(Node):
    def __init__(self):
        super().__init__('fsm_node')
        self.get_logger().info('FSM 节点 (客户端) 正在初始化...')

        # --- 1. 创建服务客户端（不阻塞等待） ---
        self.a_target_client = self.create_client(GetTarget, 'get_a_area_target')
        self.b_target_client = self.create_client(GetTarget, 'get_b_area_target')
        self.height_client = self.create_client(GetDistance, 'get_height_limit_distance')
        self.light_client = self.create_client(GetState, 'get_yellow_light_state')
        self.path_client = self.create_client(GetPath, 'get_path_choice')

        # --- 2. 初始化 FSM 变量 ---
        self.A_AREA_TARGET = None
        self.B_AREA_TARGET = "B-2"
        self.PATH_CHOICE = "stone"
        # states 模块应定义具体状态类
        self.states = [From_Charging_To_A_FullPath(), From_A_Center_To_S_Curve_FullPath(), From_Stone_to_BFullPath()]
        self.current_state = None
        self.current_state_index = 0

        # --- 3. /clock 订阅相关（用于获取仿真时间，适配所有 rclpy 版本） ---
        self._clock_lock = threading.Lock()
        self._clock_time_msg = None  # 存 builtin_interfaces/Time (sec/nsec)
        self._last_sim_time = None   # float seconds，用于计算 dt
        self._clock_sub = self.create_subscription(RosClock, '/clock', self._on_clock, 10)

        self.get_logger().info('FSM 节点 __init__ 完成。')

    # /clock 回调
    def _on_clock(self, msg: RosClock):
        with self._clock_lock:
            # 存储 builtin_interfaces/Time
            self._clock_time_msg = msg.clock

    # 服务等待（和之前逻辑相同）
    def wait_for_all_services(self):
        self.get_logger().info('FSM 正在等待所有视觉服务连接...')
        clients = {
            'A-Target': self.a_target_client,
            'B-Target': self.b_target_client,
            'Height': self.height_client,
            'Light': self.light_client,
            'Path': self.path_client
        }
        for name, client in clients.items():
            if not client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(f"服务 '{name}' 连接超时！")
                return False
        self.get_logger().info('所有视觉服务已连接！FSM准备就绪。')
        return True

    # 从订阅缓存中返回仿真时间（秒），若未收到或为0则返回 None
    def get_sim_time_seconds(self):
        with self._clock_lock:
            t = self._clock_time_msg
        if t is None:
            return None
        sec = getattr(t, 'sec', getattr(t, 'secs', 0))
        nsec = getattr(t, 'nanosec', getattr(t, 'nsecs', 0))
        sim_time = float(sec) + float(nsec) * 1e-9
        if sim_time == 0.0:
            return None
        return sim_time

    # 返回相对上一次调用的仿真时间增量（秒），并更新 last time
    def get_sim_dt(self):
        s = self.get_sim_time_seconds()
        if s is None:
            return None
        with self._clock_lock:
            last = self._last_sim_time
            self._last_sim_time = s
        if last is None:
            return None
        return s - last

    # 等待 /clock 发布非零时间，返回首次非零时间（秒），超时返回 None
    def wait_for_sim_time(self, timeout=10.0, poll=0.05):
        t0 = time.time()
        while True:
            s = self.get_sim_time_seconds()
            if s is not None:
                with self._clock_lock:
                    self._last_sim_time = s
                return s
            if time.time() - t0 > timeout:
                return None
            self.get_logger().info("等待模拟时钟(/clock) 启动...")
            time.sleep(poll)

    # FSM 执行循环（状态的 execute() 应使用 fsm_node.get_sim_time_seconds()/get_sim_dt()）
    def execute(self):
        while self.current_state_index < len(self.states) and rclpy.ok():
            self.current_state = self.states[self.current_state_index]
            self.get_logger().info(f"Current state: {self.current_state.name}")
            self.current_state.fsm_node = self  # 注入 fsm_node
            try:
                self.current_state.execute()
            except Exception as e:
                self.get_logger().error(f"State {self.current_state.name} 执行出错: {e}")
            self.get_logger().info(f"Finished executing {self.current_state.name}")
            self.current_state_index += 1


def main(args=None):
    rclpy.init(args=args)

    fsm_node = None
    executor = None
    try:
        # 1. 创建节点（不依赖 use_sim_time 参数，直接订阅 /clock）
        fsm_node = FINAL()

        # 2. 创建并启动 Spin 线程（多线程执行器以兼容服务/订阅并发）
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(fsm_node)
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        # 3. 等待依赖服务就绪
        if not fsm_node.wait_for_all_services():
            raise Exception("一个或多个视觉服务连接失败，FSM 退出。")

        # 4. 等待 /clock 开始发布非零仿真时间
        sim_start = fsm_node.wait_for_sim_time(timeout=15.0)
        if sim_start is None:
            fsm_node.get_logger().warn("未收到 /clock 非零时间（超时），将继续但请注意可能使用墙钟。")
        else:
            fsm_node.get_logger().info(f"已接收仿真时间，开始时间: {sim_start:.6f}s")

        # 5. 运行 FSM 逻辑（状态内部应使用 fsm_node 的仿真时间接口）
        fsm_node.execute()

    except KeyboardInterrupt:
        pass
    except Exception as e:
        import traceback
        if fsm_node:
            fsm_node.get_logger().error(f'FSM 崩溃: {e}')
            fsm_node.get_logger().error(traceback.format_exc())
        else:
            print(f'FSM 初始化失败: {e}')
            traceback.print_exc()
    finally:
        if executor:
            executor.shutdown()
        if fsm_node:
            fsm_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()