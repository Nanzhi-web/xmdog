from .states import *


class FSM:
    def __init__(self):
        # self.states = [Inital(), DebugStand()]

        # original
        self.states = [Inital(), From_Stone_To_S(), Reverse_S_Route(), From_S_Route_To_Section_A(), Section_A()]
        self.current_state = None
        self.current_state_index = 0
        self.A1_or_A2 = None
        self.B1_or_B2 = "B-2"
        self.slope_or_stone = "stone"
        self.visit_B_count = 0
        self.visit_A_count = 0

    def execute(self):
        while self.current_state_index < len(self.states):
            self.current_state = self.states[self.current_state_index]
            print(f"Current state: {self.current_state.name}")
            self.current_state.execute()

            if isinstance(self.current_state, Section_A):
                if self.visit_A_count == 0:
                    self.A1_or_A2 = self.current_state.qr_result
                    if self.A1_or_A2 == "A-1":
                        self.states.append(From_Section_A_To_Section_A1())
                        self.states.append(Section_A_Rotation())
                        self.states.append(From_Section_A1_To_Section_A())
                    elif self.A1_or_A2 == "A-2":
                        self.states.append(From_Section_A_To_Section_A2())
                        self.states.append(Section_A_Rotation())
                        self.states.append(From_Section_A2_To_Section_A())
                    else:
                        print("Invalid QR code detected.")
                        break
                    self.states.append(From_Section_A_To_S_Route())
                    self.states.append(S_Route())
                    self.states.append(From_S_Route_To_Choose_Route())
                    self.visit_A_count += 1
                elif self.visit_A_count == 1:
                    if self.A1_or_A2 == "A-1":
                        self.states.append(From_Section_A_To_Section_A2())
                        self.states.append(Section_A_Rotation())
                        self.states.append(From_Section_A2_To_Section_A())
                    elif self.A1_or_A2 == "A-2":
                        self.states.append(From_Section_A_To_Section_A1())
                        self.states.append(Section_A_Rotation())
                        self.states.append(From_Section_A1_To_Section_A())
                    else:
                        print("Invalid QR code detected.")
                        break
                    self.states.append(From_Section_A_To_Charging())
                    self.states.append(Finish())
                else:
                    print("Invalid Section A visit count.")
                    break

            elif isinstance(self.current_state, From_S_Route_To_Choose_Route):
                # CNN to be trained
                self.slope_or_stone = self.current_state.arrow_result  # Placeholder for actual CNN result
                if self.slope_or_stone == "slope":
                    print("Left Arrow detected, proceeding with slope route.")
                    self.states.append(From_S_To_Slope())
                    self.states.append(Slope())
                    self.states.append(Yellow_Light())
                    self.states.append(Section_B_Scan())
                    self.states.append(From_Yellow_Light_To_Section_B())
                    self.states.append(Section_B())
                elif self.slope_or_stone == "stone":
                    self.states.append(From_S_To_Stone())
                    self.states.append(Stone())
                    self.states.append(Between_Limit_Height_and_Stone())
                    self.states.append(Limit_Height())
                    self.states.append(Section_B_Scan())
                    self.states.append(From_Limit_Height_To_Section_B())
                    self.states.append(Section_B())
                else:
                    print("Invalid route detected.")
                    break

            elif isinstance(self.current_state, Section_B_Scan):
                self.B1_or_B2 = self.current_state.qr_result
                self.B1_or_B2 = "B-2"  # Placeholder for actual QR code result

            elif isinstance(self.current_state, Section_B):
                self.visit_B_count += 1
                if self.visit_B_count == 1:
                    # self.B1_or_B2 = self.current_state.qr_result
                    self.B1_or_B2 = "B-2"  # Placeholder for actual QR code result
                    if self.B1_or_B2 == "B-1":
                        self.states.append(From_Section_B_To_Section_B1())
                        self.states.append(Section_B_Rotation())
                        self.states.append(From_Section_B1_To_Section_B())
                        self.states.append(Section_B())
                    elif self.B1_or_B2 == "B-2":
                        self.states.append(From_Section_B_To_Section_B2())
                        self.states.append(Section_B_Rotation())
                        self.states.append(From_Section_B2_To_Section_B())
                        self.states.append(Section_B())
                    else:
                        print("Invalid QR code detected.")
                        break
                elif self.visit_B_count == 2:
                    if self.B1_or_B2 == "B-1":
                        self.states.append(From_Section_B_To_Section_B2())
                        self.states.append(Section_B_Rotation())
                        self.states.append(From_Section_B2_To_Section_B())
                        self.states.append(Section_B())
                    elif self.B1_or_B2 == "B-2":
                        self.states.append(From_Section_B_To_Section_B1())
                        self.states.append(Section_B_Rotation())
                        self.states.append(From_Section_B1_To_Section_B())
                        self.states.append(Section_B())
                    else:
                        print("Invalid QR code detected.")
                        break
                elif self.visit_B_count == 3:
                    if self.slope_or_stone == "slope":
                        self.states.append(From_Section_B_To_Limit_Height())
                        self.states.append(Limit_Height())
                        self.states.append(Stone())
                        self.states.append(From_Stone_To_S())
                    elif self.slope_or_stone == "stone":
                        self.states.append(From_Section_B_To_Yellow_Light())
                        self.states.append(Yellow_Light())
                        self.states.append(Slope())
                        self.states.append(From_Slope_To_S())
                    else:
                        print("Invalid route detected.")
                        break
                    self.states.append(From_Choose_Route_To_S_Route())
                    self.states.append(Reverse_S_Route())
                    self.states.append(From_S_Route_To_Section_A())
                    self.states.append(Section_A())
                else:
                    print("Invalid Section B visit count.")
                    break

            print(f"Finished executing {self.current_state.name}")
            self.current_state_index += 1


if __name__ == "__main__":
    fsm = FSM()
    fsm.execute()
    print("FSM execution completed.")

