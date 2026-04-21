from .state import State
from .basic_state import *
import time
#from camera import qr_scanner_main

# ======================================================================================
# Simulated Camera Inputs (for testing without real camera)
# ======================================================================================
# Module-level variable to simulate distance from height limit bar.
# Possible values: 'far', 'near'
HEIGHT_LIMIT_DISTANCE = 'far' 

# Module-level variable to simulate which unloading bay in Area B is targeted.
# Possible values: 'B1', 'B2'
B_AREA_TARGET = 'B2' 

# ======================================================================================
# Global Variables for Odometry
# ======================================================================================
# Global variable to store the measured time from stone road to height limiter
STONE_TO_LIMITER_TIME = 0.0
# Assumed constant walking speed in meters per second for motion_id 5
WALKING_SPEED_MPS = 0.15 
# Assumed total distance from the end of the stone road to the B-section intersection
# This value needs to be tuned based on the actual map. Let's use 8 meters as an example.
TOTAL_STONE_TO_B_ENTRANCE_DISTANCE = 3.5 

REMAINING_DISTANCE = 0.0

# ======================================================================================
# Standard States
# ======================================================================================
# State to start the sequence
class Initial_State(State):
    def __init__(self):
        super().__init__("Initial_State")
        self.basic_states = [Standing(3)]

class From_S_to_Stone_Road(State):
    def __init__(self):
        super().__init__("From_S_to_Stone_Road")
        self.basic_states = [
            Spin_Left(2), Walking_Forward(3), Shift_Left(3),
        ]

# Path: Crossing the Stone Road
class Cross_Stone_Road(State):
    def __init__(self):
        super().__init__("Cross_Stone_Road")
        self.basic_states = [
            Standing(3),  
            Walking_Stone(35),
            Standing(1),
        ]

# Path: Crossing under the Height Limiter
class Cross_Height_Limiter(State):
    def __init__(self):
        super().__init__("Cross_Height_Limiter")
        self.basic_states = [
            Walking_Forward_Slow(1.0),
            Standing(1),
            Walking_High_Limiter(10),
            Standing(3),
            #Spin_Right(0.1),
            Standing(1),
        ]

# Path: From Section B entrance to B1 or B2
class Go_To_B1(State):
    def __init__(self):
        super().__init__("Go_To_B1")
        self.basic_states = [
            Shift_Left(2.0),
            Standing(1),
            Walking_Forward(4.0),
            Standing(1),
        ]
class Go_To_B2(State):
    def __init__(self):
        super().__init__("Go_To_B2")
        self.basic_states = [
            Shift_Right(4.0),
            Standing(1),
            Walking_Forward(4.0),
            Standing(1),
        ]
# Action: Unload at B
class Unload_At_B(State):
    def __init__(self):
        super().__init__("Unload_At_B")
        self.basic_states = [
            Laying(6), 
            Standing(3),
        ]

# Path: Return from B1 or B2 to Section B entrance
class Return_From_B1(State):
    def __init__(self):
        super().__init__("Return_From_B1")
        self.basic_states = [
            Standing(1),
            Spin_Left(14.0),
            Walking_Forward(4.0),
            Standing(1),
        ]
class Return_From_B2(State):
    def __init__(self):
        super().__init__("Return_From_B2")
        self.basic_states = [
            Standing(1),
            Spin_Left(14.0),
            Walking_Forward(4.0),
            Shift_Right(5.0),
            Standing(1),
        ]

# Path: Return across Stone Road
class Return_Cross_Stone_Road(State):
    def __init__(self):
        super().__init__("Return_Cross_Stone_Road")
        self.basic_states = [
            Walking_Stone(44),
            #Shift_by_angle(5),
            ]

# Path: From Stone Road to S-Curve
class Return_To_S_Curve(State):
    def __init__(self):
        super().__init__("Return_To_S_Curve")
        self.basic_states = [
            #Walking_Stone_by_distance(50, 30),
            Walking_Forward_Slow(1),
            Standing(1),
            Shift_Left(2),
            Standing(1),
            Walking_Forward(1.75),
            Standing(1),
            Spin_Right(1.2)
        ]

# Path: Navigate S-Curve in reverse
class Navigate_S_Curve_Reverse(State):
    def __init__(self):
        super().__init__("Navigate_S_Curve_Reverse")
        self.basic_states = [
            Walking_Left_Turn(4),
            Walking_Forward_Slow(2),
            Walking_Right_Turn(7.5),
            Walking_Forward_Slow(3),
            Walking_Left_Turn(9.5),
            Walking_Forward_Slow(4),
            Walking_Right_Turn(8),
            Walking_Forward(1),
        ]

# # State to end the sequence
# class Finish_State(State):
#     def __init__(self):
#         super().__init__("Finish_State")
#         self.basic_states = [Laying(5)]


# ======================================================================================
# NEW Odometry and Measurement States
# ======================================================================================

class Measure_And_Go_To_Limiter(State):
    """
    从石板路结束后开始，向前行走，同时使用仿真时间测量耗时，
    直到接收到相机 'near' 信号或达到最大距离（超时）。
    将测量到的时间存储在全局变量 STONE_TO_LIMITER_TIME 中。
    """
    def __init__(self):
        super().__init__("Measure_And_Go_To_Limiter")
        self.max_walk_distance = 2.0  # meters
        self.timeout_duration = self.max_walk_distance / WALKING_SPEED_MPS
        self.locomotion = LocomotionController()

    def execute(self):
        global STONE_TO_LIMITER_TIME
        print(f"--- 开始阶段: {self.name} (超时设置: {self.timeout_duration:.2f}s) ---")
        self.started = True

        rclpy.init()
        sim_clock = SimulationClock()
        thread = threading.Thread(target=rclpy.spin, args=(sim_clock,), daemon=True)
        thread.start()
        time.sleep(1)

        start_time = -1
        try:
            for _ in range(5):
                _start_time = sim_clock.get_sim_time()
                if _start_time:
                    start_time = _start_time.nanosec / 1e9 + _start_time.sec
                    break
                time.sleep(0.2)
            
            if start_time == -1:
                print("错误：无法获取仿真时间，使用超时值作为备用。")
                STONE_TO_LIMITER_TIME = self.timeout_duration
                return

            while True:
                _current_time = sim_clock.get_sim_time()
                current_time = _current_time.nanosec / 1e9 + _current_time.sec
                elapsed_time = current_time - start_time

                if elapsed_time >= self.timeout_duration:
                    print(f"警告：在前进 {self.max_walk_distance} 米后仍未收到 'near' 信号。超时！")
                    STONE_TO_LIMITER_TIME = self.timeout_duration
                    break

                if HEIGHT_LIMIT_DISTANCE == 'near':
                    print(f"成功：收到 'near' 信号。耗时: {elapsed_time:.2f} 秒。")
                    STONE_TO_LIMITER_TIME = elapsed_time
                    break
                
                self.locomotion.set_motion(5)
                time.sleep(0.1)

        finally:
            self.locomotion.set_motion(1)
            time.sleep(0.5)
            sim_clock.destroy_node()
            rclpy.shutdown()

        self.finished = True
        print(f"--- 完成阶段: {self.name} (记录时间: {STONE_TO_LIMITER_TIME:.2f}s) ---")

class Continue_To_B_Entrance(State):
    """
    使用已测量的 STONE_TO_LIMITER_TIME，计算并执行
    从限高杆到B区路口的剩余行走时间。
    """
    def __init__(self):
        super().__init__("Continue_To_B_Entrance")
        self.basic_states = []

    def execute(self):
        global REMAINING_DISTANCE
        print(f"--- 开始阶段: {self.name} ---")
        
        distance_walked_to_limiter = STONE_TO_LIMITER_TIME * WALKING_SPEED_MPS
        remaining_distance = TOTAL_STONE_TO_B_ENTRANCE_DISTANCE - distance_walked_to_limiter
        REMAINING_DISTANCE = remaining_distance

        if remaining_distance > 0:
            remaining_time = remaining_distance / WALKING_SPEED_MPS
            print(f"已走距离: {distance_walked_to_limiter:.2f}m. 剩余距离: {remaining_distance:.2f}m. 需继续前进 {remaining_time:.2f}s.")
            self.basic_states = [Walking_Forward(remaining_time), Standing(1), Shift_Right(2.0), Standing(1), Lookup(3.9), Standing(3)]
        else:
            print(f"已走距离: {distance_walked_to_limiter:.2f}m. 已超过到B区路口的总距离. 不再前进。")
            self.basic_states = [Standing(1), Shift_Right(2.0), Standing(1), Lookup(3.9), Standing(3)]

        super().execute()
        print(f"--- 完成阶段: {self.name} ---")

class Return_To_Limiter_Odom(State):
    """
    返程时，使用里程计（总距离/速度）从B区路口走到限高杆位置。
    """
    def __init__(self):
        super().__init__("Return_To_Limiter_Odom")
        self.basic_states = []

    def execute(self):
        print(f"--- 开始阶段: {self.name} ---")
        time_to_walk = REMAINING_DISTANCE / WALKING_SPEED_MPS
        time_to_walk2 = (TOTAL_STONE_TO_B_ENTRANCE_DISTANCE - REMAINING_DISTANCE) / WALKING_SPEED_MPS
        print(f"返程：使用里程计从B区路口前往限高杆位置，预计行走 {time_to_walk:.2f} 秒。")
        
        self.basic_states = [
            Standing(1),
            Walking_Forward(time_to_walk),
            Standing(3),
            Walking_Forward_Slow(1.0),
            Standing(1),
            Walking_High_Limiter(10),
            Standing(3),
            #Spin_Right(0.1),
            Walking_Forward(time_to_walk2),
        ]
        super().execute()
        print(f"--- 完成阶段: {self.name} ---")


# ======================================================================================
# Main Competition Path Planner
# ======================================================================================

class From_Stone_to_BFullPath(State):
    """
    这是一个高层控制器，用于执行这段路程流程。
    它本身不是一个State，而是负责创建和执行一个State列表。
    """
    def __init__(self):
        super().__init__("From_Stone_to_BFullPath")
        self.stages = [Initial_State(), From_S_to_Stone_Road(),]
        self.current_stage_index = 0

    def _initialize_path(self):
        """根据相机模拟变量，构建完整的状态路径"""
        print("--- 根据模拟传感器输入，构建完整路径 ---")
        
        # 1. 出发，过石板路
        self.stages.append(Cross_Stone_Road())

        # 2. 测量到限高杆的距离，穿过它，然后继续走到B区路口
        self.stages.append(Measure_And_Go_To_Limiter())
        self.stages.append(Cross_Height_Limiter())
        self.stages.append(Continue_To_B_Entrance())

        # 3. 到达B区路口后，根据目标卸货
        if B_AREA_TARGET == 'B1':
            print(f"路径规划：选择 B1")
            self.stages.append(Go_To_B1())
        else: # B2
            print(f"路径规划：选择 B2")
            self.stages.append(Go_To_B2())
        
        self.stages.append(Unload_At_B())

        # 4. 从B区返回
        if B_AREA_TARGET == 'B1':
             self.stages.append(Return_From_B1())
        else:
            self.stages.append(Return_From_B2())

        # 5. 返程，通过里程计过限高杆和石板路
        self.stages.append(Return_To_Limiter_Odom())
        self.stages.append(Return_Cross_Stone_Road())

        # 6. 返回S弯并结束
        self.stages.append(Return_To_S_Curve())
        self.stages.append(Navigate_S_Curve_Reverse())
        #self.stages.append(Finish_State())
        
        print("--- 路径规划完成 ---")

    def execute(self):
        """执行整个比赛流程"""
        self._initialize_path()

        while self.current_stage_index < len(self.stages):
            current_stage = self.stages[self.current_stage_index]
            print(f"--- 开始阶段: {current_stage.name} ---")
            current_stage.execute()
            print(f"--- 完成阶段: {current_stage.name} ---")
            
            self.current_stage_index += 1
        
        print(">>> 比赛全程结束! <<<")
