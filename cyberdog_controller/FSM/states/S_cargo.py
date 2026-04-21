# slope_route_fixed.py
from .state import State
from .basic_state import *
import time
import os


# ======================================================================================
# 各个阶段的状态类
# ======================================================================================

class PathDecisionAtSExit(State):
    """在S弯出口进行路径决策（固定选择上坡路线）"""

    def __init__(self):
        super().__init__("PathDecisionAtSExit")
        self.path_choice = "slope"  # 固定选择上坡路线

    def execute(self):
        print(f"Executing {self.name}")

        # 模拟路径决策过程
        stages = [
            ("到达S弯出口", Standing(1)),
            ("进行路径决策", Standing(2)),
            ("确认选择上坡路线", Standing(1)),
        ]

        for desc, action in stages:
            print(f"  - {desc}")
            action.execute()

        # 固定选择上坡路线
        self.path_choice = "slope"
        print(f">>> 路径选择结果: {self.path_choice}")

        # 语音播报
        try:
            os.system('ros2 topic pub /tts std_msgs/String "data: 选择上坡路线" -1')
        except Exception as e:
            print(f"语音播报失败: {e}")

        print(f"Finished executing {self.name}")


class From_S_Exit_To_Slope_Entry(State):
    """S弯出口到坡道入口定位"""

    def __init__(self):
        super().__init__("From_S_Exit_To_Slope_Entry")

    def execute(self):
        print(f"Executing {self.name}")

        stages = [
            ("向前移动离开S弯", Walking_Forward(4.5)),
            ("停顿观察", Standing(1)),
            ("右转准备定位", Spin_Right(3)),
            ("前进到坡道入口", Walking_Forward(3)),
            ("左转面向坡道", Spin_Left(5.9)),
            ("微调定位", Walking_Forward_Slow(0.8)),
            ("准备上坡", Standing(1)),
        ]

        for desc, action in stages:
            print(f"  - {desc}")
            action.execute()

        print(f"Finished executing {self.name}")


class Slope_Uphill_Phase(State):
    """上坡阶段"""

    def __init__(self):
        super().__init__("Slope_Uphill_Phase")

    def execute(self):
        print(f"Executing {self.name}")

        stages = [
            ("坡底调整", Standing(1)),
            ("开始上坡", Walking_Forward(8)),
            ("坡中前进", Walking_Forward(2)),
            ("继续上坡", Walking_Forward(6)),
            ("到达坡顶", Standing(1)),
        ]

        for desc, action in stages:
            print(f"  - {desc}")
            action.execute()

        print(f"Finished executing {self.name}")


class Slope_Downhill_With_Yellow_Stop(State):
    """下坡阶段包含黄灯检测"""

    def __init__(self):
        super().__init__("Slope_Downhill_With_Yellow_Stop")
        self.yellow_light_detected = True  # 固定检测到黄灯

    def execute(self):
        print(f"Executing {self.name}")

        stages = [
            ("开始下坡", Walking_Slope(11)),
            ("下坡完成", Standing(1)),
            ("直行寻找黄灯", Walking_Forward(4)),
            ("到达黄灯位置", Standing(1)),
            ("语音播报黄灯", self._yellow_light_announcement),
            ("黄灯等待3秒", Standing(3)),
            ("继续前往B区", Walking_Forward(3)),
        ]

        for desc, action in stages:
            print(f"  - {desc}")
            if callable(action):
                action()
            else:
                action.execute()

        print(f"Finished executing {self.name}")

    def _yellow_light_announcement(self):
        """黄灯语音播报"""
        print("  - 语音播报: 检测到黄灯，请注意安全")
        try:
            os.system('ros2 topic pub /tts std_msgs/String "data: 检测到黄灯，请注意安全" -1')
        except Exception as e:
            print(f"  - 语音播报失败: {e}")


class Go_To_B_Area_Fixed(State):
    """前往B区配货区"""

    def __init__(self, target_b_area="B1"):
        super().__init__("Go_To_B_Area_Fixed")
        self.target_b_area = target_b_area

    def execute(self):
        print(f"Executing {self.name} - 目标: {self.target_b_area}")

        stages = [
            ("直行前往B1", Walking_Forward(27)),
            ("到达B1区域", Standing(1)),
            ("左移对准库位", Shift_Left(7.5)),
            ("前进到配货点", Walking_Forward(5)),
            ("到达B1完成", Standing(2)),
        ]

        for desc, action in stages:
            print(f"  - {desc}")
            action.execute()

        print(f"Finished executing {self.name}")


# ======================================================================================
# Main Path Planner: S弯到B区上坡路线
# ======================================================================================

class From_S_To_B_Area_FullPath(State):
    """
    路径控制器：完成从S弯到B区的完整上坡路线流程
    """

    def __init__(self, target_b_area="B1"):
        super().__init__("From_S_To_B_Area_FullPath")
        self.stages = []
        self.current_stage_index = 0
        self.path_choice = "slope"  # 路径选择结果
        self.target_b_area = target_b_area  # 目标B区

    def _initialize_path(self):
        """构建完整的状态路径"""
        print("--- 开始构建从S弯到B区的上坡路线路径 ---")

        # 1: 路径决策（固定选择上坡）
        self.stages.append(PathDecisionAtSExit())

        print("--- 初始路径构建完成（路径决策后） ---")

    def execute(self):
        """执行从S弯到B区的完整流程"""
        print("=" * 80)
        print("开始执行: S弯出口 -> 路径选择 -> 上坡 -> 下坡 -> 黄灯检测 -> B区")
        print("=" * 80)

        # 先构建初始路径（路径决策）
        self._initialize_path()

        # 执行路径决策阶段
        current_stage = self.stages[0]
        print(f"\n--- 开始阶段 1/1: {current_stage.name} ---")
        current_stage.execute()
        print(f"--- 完成阶段 1/1: {current_stage.name} ---")

        # 获取路径选择结果
        if isinstance(current_stage, PathDecisionAtSExit):
            self.path_choice = current_stage.path_choice
            print(f"\n>>> 路径选择完成！识别结果: {self.path_choice} <<<\n")

        # 根据路径选择结果，动态添加后续阶段
        if self.path_choice == 'slope':
            print(">>> 路径规划：执行上坡路线")

            # 添加完整的上坡路线阶段
            self.stages.append(From_S_Exit_To_Slope_Entry())
            self.stages.append(Slope_Uphill_Phase())
            self.stages.append(Slope_Downhill_With_Yellow_Stop())
            self.stages.append(Go_To_B_Area_Fixed(self.target_b_area))

            route_name = '上坡路线'
        else:
            print(f">>> 路径规划：执行石子路线（当前固定为上坡路线）")
            # 这里可以扩展石子路线的逻辑
            self.stages.append(From_S_Exit_To_Slope_Entry())
            self.stages.append(Slope_Uphill_Phase())
            self.stages.append(Slope_Downhill_With_Yellow_Stop())
            self.stages.append(Go_To_B_Area_Fixed(self.target_b_area))
            route_name = '石子路线'

        # 执行剩余阶段
        for i in range(1, len(self.stages)):
            current_stage = self.stages[i]
            print(f"\n--- 开始阶段 {i + 1}/{len(self.stages)}: {current_stage.name} ---")
            current_stage.execute()
            print(f"--- 完成阶段 {i + 1}/{len(self.stages)}: {current_stage.name} ---")

        print("\n" + "=" * 80)
        print(f">>> S弯到B区流程完成！通过{route_name}到达{self.target_b_area} <<<")
        print("=" * 80 + "\n")

        return self.path_choice


# ======================================================================================
# 对外接口(供主状态机)
# ======================================================================================

def execute_slope_route_to_b_area(target_b_area="B1"):
    """
    执行S弯到B区的完整上坡路线流程

    流程：
    1. S弯出口路径决策
    2. 坡道入口定位
    3. 上坡阶段
    4. 下坡+黄灯检测
    5. 前往B区配货区

    Returns:
        str: 返回路径选择结果
    """
    controller = From_S_To_B_Area_FullPath(target_b_area)
    path_choice = controller.execute()
    return path_choice


# =============================================================================
# 视觉接口
# =============================================================================

# 全局变量（为了兼容原有接口）
YELLOW_LIGHT_STATE = "detected"  # 固定检测到黄灯
PATH_CHOICE = "slope"  # 固定选择上坡路线


def update_yellow_light_state(detected):
    """视觉接口：黄灯状态更新"""
    global YELLOW_LIGHT_STATE
    YELLOW_LIGHT_STATE = "detected" if detected else "none"
    print(f"[VISUAL INTERFACE] 黄灯检测状态更新: {YELLOW_LIGHT_STATE}")


def update_path_choice(choice):
    """视觉接口：路径选择更新"""
    global PATH_CHOICE
    PATH_CHOICE = choice
    print(f"[VISUAL INTERFACE] 路径选择更新: {PATH_CHOICE}")


def get_yellow_light_state():
    """视觉接口：获取黄灯状态"""
    return YELLOW_LIGHT_STATE


def get_path_choice():
    """视觉接口：获取路径选择"""
    return PATH_CHOICE


def reset_visual_states():
    """视觉接口：重置视觉状态"""
    global YELLOW_LIGHT_STATE, PATH_CHOICE
    YELLOW_LIGHT_STATE = "none"
    PATH_CHOICE = "slope"  # 默认上坡路线
    print("[VISUAL INTERFACE] 视觉状态已重置")


# ======================================================================================
# 测试代码
# ======================================================================================

if __name__ == "__main__":
    print("=== S弯到B区上坡路线模块测试 ===")

    # 执行完整流程
    path_choice = execute_slope_route_to_b_area("B1")

    print(f"\n=== 测试完成 ===")
    print(f"路径选择: {path_choice}")