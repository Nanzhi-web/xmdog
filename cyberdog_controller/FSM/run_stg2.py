import threading
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock as RosClock

# 1. 导入你的 FSM 逻辑类
from FSM.fsm_copy import Find_Ball


class FSMRunnerNode(Node):
    def __init__(self):
        super().__init__("fsm_runner_node")
        self._clock_sub = self.create_subscription(RosClock, "/clock", self._on_clock, 10)
        self.get_logger().info("ROS2 FSM 驱动节点已启动")

    def _on_clock(self, msg):
        pass  # 这里仅作为节点占位，确保能接收时钟


def main():
    rclpy.init()
    node = FSMRunnerNode()

    # 在后台旋转节点，处理 ROS 通信
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    # 2. 实例化 FSM
    fsm_logic = Find_Ball()

    # 3. 【核心修复】将 node 注入到所有层级的状态中
    for state in fsm_logic.states:
        # 第一层：注入给 initial_2, walk_to_mid 等
        state.fsm_node = node

        # 第二层：注入给这些状态内部持有的 basic_states (如 Standing, Walking)
        if hasattr(state, 'basic_states'):
            for bs in state.basic_states:
                bs.fsm_node = node

    print("--- 注入完成，开始执行 ---")
    try:
        fsm_logic.execute()
    except Exception as e:
        node.get_logger().error(f"运行出错: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()