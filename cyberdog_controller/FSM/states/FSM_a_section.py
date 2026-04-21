from .state import State
from .basic_state import *
import time
import rclpy
from xmdog_msgs.srv import GetTarget # <-- 导入服务定义

# ======================================================================================
# Simulated Camera Inputs (for testing without real camera)
# ======================================================================================
A_AREA_TARGET = 'A-1' 

# ======================================================================================
# Standard States for Charging Station to Area A
# ======================================================================================

class Initial_State(State):
    """初始状态：机器人在充电站准备出发"""
    def __init__(self):
        super().__init__("Initial_State")
        self.basic_states = [Standing(3)]

class From_Charging_To_A_QR(State):
    """从充电站行驶到A区二维码识别位置，停在码前"""
    def __init__(self):
        super().__init__("From_Charging_To_A_QR")
        self.basic_states = [
            Walking_Forward(4.9),      # 向前走到合适距离
            Standing(1),
            Spin_Right(5.92),                   # 右转面向A区
            Standing(1),
            Walking_Forward(5),                 # 前进到扫码位置
            Standing(2),                        # 停在码前，准备扫码
        ]

class Scan_A_Area_QR(State):
    """在码前进行二维码扫描识别库位（不移动，只扫码）"""
    def __init__(self):
        super().__init__("Scan_A_Area_QR")
        self.qr_result = None
        self.basic_states = [Standing(0)]  # 站立不动，只进行扫码
        
    def execute(self):
        print(f"Executing {self.name} - 开始扫描二维码")
        super().execute()
        
         # --- 新的交互逻辑 ---
        self.fsm_node.get_logger().info('正在向视觉节点请求A选择(get_qr_code_choice)...')
        request = GetTarget.Request() # 创建一个空请求

        # 1. 异步调用 'get_path_choice' 服务
        #    (self.fsm_node.path_client 是在 fsm.py 的 __init__ 中创建的)
        future = self.fsm_node.a_target_client.call_async(request)
        
        # 2. FSM节点spin并阻塞，等待服务返回结果
        rclpy.spin_until_future_complete(self.fsm_node, future)

        # 3. 拿到结果
        try:
            response = future.result()
            if response.success:
                # 4. 【赋值给State的临时变量】
                #    将你的服务返回值 (response.path, 比如 "slope")
                #    赋给 State 内部的 self.arrow_result
                self.qr_result = response.target
                self.fsm_node.get_logger().info(f'视觉节点返回: {self.qr_result}')
            else:
                self.fsm_node.get_logger().error('视觉服务调用失败, 默认使用 "A-1"')
                self.qr_result = "A-1" # 识别失败, 走默认路径
        except Exception as e:
            self.fsm_node.get_logger().error(f'服务调用异常: {e}')
            self.qr_result = "A-1" # 异常, 走默认路径

        # self.qr_result = A_AREA_TARGET
        print(f"✓ 识别到A区库位: {self.qr_result}")
        print(f"✓ 语音播报: A区库位{self.qr_result[-1]}")
        print(f"Finished executing {self.name}")

class Go_To_A1_Loading(State):
    """从二维码位置前往A1配货库位"""
    def __init__(self):
        super().__init__("Go_To_A1_Loading")
        self.basic_states = [
            Walking_Forward(7),
            Standing(1),
            Spin_Left(5.92),
            Standing(1),
            Walking_Forward(5),
            Standing(1),
            Spin_Left(5.95),
            Standing(0.1),
            # Spin_by_line(6),
            Standing(1),
            Walking_Forward(4.5),
            Standing(1),
        ]

class Go_To_A2_Loading(State):
    """从二维码位置前往A2配货库位"""
    def __init__(self):
        super().__init__("Go_To_A2_Loading")
        self.basic_states = [
            Walking_Forward(4),
            Standing(1),
            Spin_Right(5.85),
            Standing(1),
            Spin_by_line(6),
            Standing(1),
            Walking_Forward(4.4),
            Standing(1),
            Spin_Right(5.86),
            Standing(1),
            Spin_by_line(6),
            Standing(1),
            Walking_Forward(4),
            Standing(1),
            Spin_by_line(5),
            Standing(1),
        ]

class Loading_At_A(State):
    """在A区配货库位进行配货"""
    def __init__(self):
        super().__init__("Loading_At_A")
        self.basic_states = [
            Laying(6),
            Standing(3),
        ]

class Return_From_A1_To_A_Center(State):
    """从A1配货库位返回A区中心"""
    def __init__(self):
        super().__init__("Return_From_A1_To_A_Center")
        self.basic_states = [
            Walking_Backward(3.5),
            Standing(1),
            Spin_Right(5.95),
            Standing(1),
            Walking_Backward(3),
            Standing(1),
            Spin_Right(5.92),
            Standing(1),
            Walking_Backward(10.5),
            Spin_Left(5.92),
        ]

class Return_From_A2_To_A_Center(State):
    """从A2配货库位返回A区中心"""
    def __init__(self):
        super().__init__("Return_From_A2_To_A_Center")
        self.basic_states = [
            Walking_Backward(4.5),
            Standing(1),
            Spin_Right(5.95),
            Standing(1),
            Walking_Backward(5),
            Standing(1),
            Spin_Right(5.92),
            Standing(1),
            Walking_Backward(12),
            Spin_Left(5.92),
        ]

# ======================================================================================
# Main Path Planner: Charging Station to Area A
# ======================================================================================

class From_Charging_To_A_FullPath(State):
    """
    高层控制器：完成从充电站到A区的完整流程
    """
    def __init__(self):
        super().__init__("From_Charging_To_A_FullPath")
        self.stages = []
        self.current_stage_index = 0
        self.qr_result = None
        self.fsm_node = None

    def _initialize_path(self):
        """构建完整的状态路径"""
        print("--- 开始构建从充电站到A区的路径 ---")
        
        # 1: 初始化
        self.stages.append(Initial_State())
        
        # 2: 从充电站走到A区二维码前面并停下
        self.stages.append(From_Charging_To_A_QR())
        
        # 3: 在码前扫码（不移动）
        self.stages.append(Scan_A_Area_QR())
        
        
        print("--- 初始路径构建完成（扫码前） ---")

    def execute(self):
        """执行从充电站到A区的完整流程"""
        print("="*80)
        print("开始执行: 充电站 -> A区二维码 -> 扫码 -> 配货")
        print("="*80)
        
        # 先构建初始路径（到扫码为止）
        self._initialize_path()
        i = 0
        # 执行前3个阶段：初始化 -> 走到码前 -> 扫码
        for _ in range(3):
            current_stage = self.stages[i]
            print(f"\n--- 开始阶段 {i+1}/3: {current_stage.name} ---")
            current_stage.fsm_node = self.fsm_node
            current_stage.execute()
            print(f"--- 完成阶段 {i+1}/3: {current_stage.name} ---")
            i = i + 1

            # 在扫码阶段执行完后，获取扫码结果
            if isinstance(current_stage, Scan_A_Area_QR):
                self.qr_result = current_stage.qr_result
                print(f"\n>>> 扫码完成！识别结果: {self.qr_result} <<<\n")
        
        # 根据扫码结果，动态添加后续阶段
        if self.qr_result == 'A-1':
            print(">>> 路径规划：前往 A1 配货库位")
            self.stages.append(Go_To_A1_Loading())
            self.stages.append(Loading_At_A())
            self.stages.append(Return_From_A1_To_A_Center())
            loading_bay = 'A1'
        elif self.qr_result == 'A-2':
            print(">>> 路径规划：前往 A2 配货库位")
            self.stages.append(Go_To_A2_Loading())
            self.stages.append(Loading_At_A())
            self.stages.append(Return_From_A2_To_A_Center())
            loading_bay = 'A2'
        else:
            print(f"❌ 错误：无效的二维码识别结果 {self.qr_result}")
            return None
        
        i = 0

        # 执行剩余阶段（前往库位 -> 配货 -> 返回中心）
        for i in range(3, len(self.stages)):
            current_stage = self.stages[i]
            print(f"\n--- 开始阶段 {i+1}/{len(self.stages)}: {current_stage.name} ---")
            current_stage.fsm_node = self.fsm_node
            current_stage.execute()
            print(f"--- 完成阶段 {i+1}/{len(self.stages)}: {current_stage.name} ---")
            i = i + 1
        
        print("\n" + "="*80)
        print(f">>> 充电站到A区流程完成！已在{loading_bay}完成配货，返回A区中心 <<<")
        print("="*80 + "\n")
        
        return self.qr_result


# ======================================================================================
# 对外接口(供主状态机)
# ======================================================================================

def execute_charging_to_a_section():
    """
    执行充电站到A区的完整流程
    
    流程：
    1. 从充电站出发
    2. 走到A区二维码识别位置
    3. 停下来扫描二维码
    4. 根据识别结果前往A1或A2配货库位
    5. 完成配货
    6. 返回A区中心
    
    Returns:
        str: 返回识别到的库位信息 ('A-1' 或 'A-2')，失败返回 None
    """
    controller = From_Charging_To_A_FullPath()
    qr_result = controller.execute()
    return qr_result
