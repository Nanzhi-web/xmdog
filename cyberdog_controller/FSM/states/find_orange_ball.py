from .state import State
from .basic_state import *

class initial_2(State):
    def __init__(self):
        super().__init__("initial_2")
        self.basic_states = [
            Standing(5),
        ]

class walk_to_mid(State):
    def __init__(self):
        super().__init__("walk_to_mid")
        self.basic_states = [
            Walking_Left_Turn(1.9)
        ]

class kick_first_ball(State):
    def __init__(self):
        super().__init__("kick_first_ball")
        self.basic_states = [
            Walking_Right_Turn(2.3),
            Walking_Forward(7),
            Walking_Right_Turn(2.6)
        ]

class kick_second_ball(State):
    def __init__(self):
        super().__init__("kick_second_ball")
        self.basic_states = [
            Walking_Backward(2),
            Spin_Left(12.5),
            Walking_Left_Turn(3.4)
        ]

class kick_third_ball(State):
    def __init__(self):
        super().__init__("kick_third_ball")
        self.basic_states = [
            Shift_Right(1.2),
            Spin_Right(1.3),
            Walking_Forward(12)
        ]

class kick_fourth_ball(State):
    def __init__(self):
        super().__init__("kick_fourth_ball")
        self.basic_states = [
            Walking_Backward(2),
            Spin_Right(2),
            Walking_Right_Turn(3),
            Spin_Right(2.86),
            Walking_Forward(18)
        ]

class walk_to_end(State):
    def __init__(self):
        super().__init__("walk_to_end")
        self.basic_states = [
            Walking_Backward(1.5),
            Spin_Right(2.2),
            Walking_Forward(8)
        ]