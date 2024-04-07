"""
Arm must be externally rotated, abducted prior to starting. 
"""

import sys
import os
src_dir = os.path.join(os.path.dirname(__file__), '..', 'src')
src_dir = os.path.abspath(src_dir)

sys.path.append(src_dir)

import kinematics as k
import dynamixel_2_0 as d

DOF = 5
# Change to device appropriate device/com port
DEVICENAME = "/dev/ttyUSB0"

MINIMUM = [d.MM_SHOULDER_FLEX_EX["min"],
           d.MM_SHOULDER_ABDUCTION["min"],
           d.MM_SHOULDER_ROT["min"],
           d.MM_ELBOW_FLEX_EX["min"],
           d.MM_PRO_SUP["min"]]

MAXIMUM = [d.MM_SHOULDER_FLEX_EX["max"],
           d.MM_SHOULDER_ABDUCTION["max"],
           d.MM_SHOULDER_ROT["max"],
           d.MM_ELBOW_FLEX_EX["max"],
           d.MM_PRO_SUP["max"]]

RESTING = [d.MM_SHOULDER_FLEX_EX["zero"],
           d.MM_SHOULDER_ABDUCTION["zero"],
           d.MM_SHOULDER_ROT["zero"],
           d.MM_ELBOW_FLEX_EX["zero"],
           d.MM_PRO_SUP["zero"]]

class Arm():
    def __init__(self):
        self._kinematics = k.Kinematics()
        self._motors = d.Motors(DOF, DEVICENAME)
        self._target_pos = []
        self._start_motors()

    def _calc_angles(self, coord: list) -> None:
        self._kinematics.get_angles(coord[0], coord[1], coord[2])
        return

    def _start_motors(self) -> None:
        self._motors.torque_toggle(1)
        self._motors.set_goal(RESTING)
        self._motors.get_current_pos()
        return
    
    def _angle_to_dynamixel(self) -> None:
        self._target_pos = [int(round((180+angle)*d.ANGLE_TO_DYNA)) for angle in self._kinematics.angles]
        self._target_pos[1] -= 2044
        self._target_pos[3] -= 2044
        return
    
    def _check_limits(self) -> bool:
        for i in range(DOF):
            if self._target_pos[i] not in range(MINIMUM[i], MAXIMUM[i]):
                return False
        return True
    
    def get_current_pos(self) -> list:
        self._motors.get_current_pos()
        return self._motors.pos
    
    def move_to_coord(self, coord: list) -> bool:
        self._calc_angles(coord)
        self._angle_to_dynamixel()
        if self._check_limits():
            chk = input(f'Confirm move to:\nMotor Angles:\n{self._kinematics.angles}\nTarget Positions:\n{self._target_pos}, <y> to confirm:\n')
            if chk.lower() == 'y':
                self._motors.set_goal(self._target_pos)
                return True
            else:
                print('Aborting...')
                return False
        else:
            print(f'Unable to reach position {coord[0]} {coord[1]} {coord[2]}')
            return False
        
    def pour(self) -> None:
        pour = input('<y> to pour')
    
    def end_session(self) -> None:
        self._motors.port_close()
        return

TEST_COORD = [166.2, -334.9, -4.9]
#TEST_COORD = [L4, -L2, L1-L4] #rest position

def get_angles() -> list:
    coords = ['X', 'Y', 'Z']
    for i in range(len(coords)):
        entry = input(f'Enter {coords[i]} Position or <Q> to Quit:\n')
        if entry.capitalize() == 'Q':
            raise ValueError('Quitting...') 
        else:
            try:
                int(entry)
                coords[i] = float(entry)
            except:
                raise ValueError('Invalid Entry.')
    return coords

def main():
    arm = Arm()
    while True:
        try:
            coords = get_angles()
            arm.move_to_coord(coords)
        except ValueError as e:
            print(e.args)
            break
    arm.end_session()

if __name__ == "__main__":
    main()