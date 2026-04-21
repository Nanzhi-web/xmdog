from .states import *

class Find_Ball:
    def __init__(self):
        # self.states = [Inital(), DebugStand()]

        # original
        self.states = [initial_2(), walk_to_mid(), kick_first_ball(),kick_second_ball(), kick_third_ball(), kick_fourth_ball(), walk_to_end()]
        self.current_state = None
        self.current_state_index = 0

    def execute(self):
        while self.current_state_index < len(self.states):
            self.current_state = self.states[self.current_state_index]
            print(f"Current state: {self.current_state.name}")
            self.current_state.execute()
            print(f"Finished executing {self.current_state.name}")
            self.current_state_index += 1

    
if __name__ == "__main__":
    fsm_copy = Find_Ball()
    fsm_copy.execute()
    print("execution completed.")

