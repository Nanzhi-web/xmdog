from .state import State
from .basic_state import *

class initial_stg2(State):
    def __init__(self):
        super().__init__("initial_2")
        self.basic_states = [
            Standing(5),
        ]

class scan_row(State):
    def __init__(self):
        super().__init__("walk_to_mid")
        self.basic_states = [
            Walking_Left_Turn(1.9)
        ]

class back_to_bottom(State):
    def __init__(self):
        super().__init__("walk_to_mid")
        self.basic_states = [
            Walking_Left_Turn(1.9)
        ]

class line_align(State):
    def __init__(self):
        super().__init__("walk_to_mid")
        self.basic_states = [
            Walking_Left_Turn(1.9)
        ]