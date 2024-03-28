import forward_kinematic_5dof as fk
import inverse_kinematic_5dof as ik

"""
Kinematics Class
"""

# Limb measurements in mm
L1 = 300        # Height, equivalent to l0 matlab
L2 = 30         # Depth, equivalent to d0 matlab
L3 = 265        # Shoulder to elbow, equivalent to l1 matlab
L4 = 235        # Elbow to end effector, equivalent to l2 matlab

# Other Constants
DOF = 5         # Degrees of freedom
DIM = 4         # Dimension of arrays
STEPS = 100     # Motor steps
ORIGIN = ik.mat.Matrix([[0], [0], [0], [1]])
REST_X = 235
REST_Y = -30
REST_Z = 35

class Kinematics():
    def __init__(self):
        self.i_tm = ik.hard_trans_matrix()
        self.i_pm = ik.position_matrix(self.i_tm)
        self.f_pos = []
        self.angles = []

    def get_angles(self, x: float = REST_X, y: float = REST_Y, z: float = REST_Z) -> list:
        self.angles = ik.get_angles(self.i_pm, x, y, z)
        return self.angles
        
    def fk_check(self) -> list:
        self.f_pos = fk.fk_end(self.angles) 
        return self.f_pos

TEST_COORD = [300, -100, 100]

def main():
    kin = Kinematics()
    kin.get_angles(TEST_COORD[0], TEST_COORD[1], TEST_COORD[2])
    kin.fk_check()
    print(kin.angles)
    print(kin.f_pos)

if __name__ == "__main__":
    main()