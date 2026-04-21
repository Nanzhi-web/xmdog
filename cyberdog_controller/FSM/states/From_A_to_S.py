from .state import State
from .basic_state import *
from camera.path_scanner import path_scanner_main

class From_Section_A_To_S_Route(State):
    def __init__(self):
        super().__init__("From_Section_A_To_S_Route")
        self._check_method = None
        self.basic_states = []

    def build_states(self, check_method):
        self.basic_states = [
            Standing(1),
            Walking_Forward_Slow(6),
            Standing(1),
        ]

    @property
    def check_method(self):
        if self._check_method is None:
            self._check_method = self._init_check_method()
        return self._check_method

    def _init_check_method(self):
        method = path_scanner_main
        self.build_states(method)
        return method
    
    def execute(self):
        print(f"Executing {self.name} (从A区中心出发)")
        _ = self.check_method
        super().execute()
        print(f"Finished executing {self.name}")

class S_Route(State):
    def __init__(self):
        super().__init__("S_Route")
        self.basic_states = []

    def build_states(self):
        self.basic_states = [
            Walking_Left_Turn(8),
            Walking_Forward_Slow(4),
            Walking_Right_Turn(9.5),
            Walking_Forward_Slow(3),
            Walking_Left_Turn(7.5),
            Walking_Forward_Slow(2),
            Walking_Right_Turn(4.02),
            Standing(1),
        ]   
    
    def execute(self):
        print(f"Executing {self.name}")
        self.build_states()
        super().execute()
        print(f"Finished executing {self.name}")

class From_A_Center_To_S_Curve_FullPath(State):
    def __init__(self):
        super().__init__("From_A_Center_To_S_Curve_FullPath")
        self.stages = [
            From_Section_A_To_S_Route(),
            S_Route()
        ]

    def execute(self):
        print("="*80)
        print("开始执行：A区中心→S弯入口→完整S弯")
        print("="*80)
        for i, stage in enumerate(self.stages):
            print(f"\n--- 阶段 {i+1}/{len(self.stages)}: {stage.name} ---")
            stage.execute()
        print("\n>>> A区中心到S弯流程完成 <<<")

def execute_a_center_to_s_curve():
    controller = From_A_Center_To_S_Curve_FullPath()
    controller.execute()
