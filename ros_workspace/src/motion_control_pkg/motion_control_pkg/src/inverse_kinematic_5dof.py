import sys
import sympy as sym
import sympy.matrices as mat
import math as m
import matplotlib as mpl
import numpy as np
from forward_kinematic_5dof import fk_end

"""
Step constraints for each motor: Base (Range)
    Shoulder flex/ext: resting ~2800 (1200-4095)
    Shoulder abd/add: resting ~2000 (0-2047)
    Elbow: 1000 (0-1500)
    Wrist: 860 (0-1023)
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
ORIGIN = mat.Matrix([[0], [0], [0], [1]])
REST_X = 235
REST_Y = -30
REST_Z = 35

# Get input positions
def get_position():
    if len(sys.argv) == 4:
        return [float(coord) for coord in sys.argv[1:]]   # Returns [x y z]
    else:
        coord = input("Enter Coordinates: <x> <y> <z>\n").split(" ")
        if len(coord) == 0:
            exit()
        if len(coord) != 3:
            print("Invalid Inputs")
            print(*coord)
            get_position()
        return [float(co) for co in coord]    # Returns [x y z]

# symbolic variables, will give value in radians
q1 = sym.symbols('q1')
q2 = sym.symbols('q2')
q3 = sym.symbols('q3')
q4 = sym.symbols('q4')
q5 = sym.symbols('q5')

# DH table 
a = [0, 0, 0, 0, 0, 0]
alpha = mat.Matrix([(sym.pi/2), -(sym.pi/2), -(sym.pi/2), -(sym.pi/2), -(sym.pi/2), 0])
d = mat.Matrix([L1, L2, 0, L3, 0, L4])
theta = mat.Matrix([0, q1-(sym.pi/2), -(sym.pi/2)-q2, q3-(sym.pi/2), q4-(sym.pi/2), q5])

# Get symbolic transformation matrices
def trans_matrix(a_t: mat.Matrix = a, alpha_t: mat.Matrix = alpha, d_t: mat.Matrix = d, theta_t: mat.Matrix = theta, dof = DOF):

    transformation_matrices = np.zeros((DOF+1, DIM, DIM), dtype='object')

    # Initial transformation matrix
    transformation_matrices[0, :, :] = mat.Matrix([[1, 0, 0, 0],
                                            [0, 0, -1, 0],
                                            [0, 1, 0, d[0]],
                                            [0, 0, 0, 1]])

    # Update transform matrix for each position
    for i in range(1, dof+1):
        trans_mat = mat.Matrix([[sym.cos(theta_t[i]), -sym.sin(theta_t[i])*round(m.cos(alpha_t[i])), sym.sin(theta_t[i])*round(m.sin(alpha_t[i])), a_t[i]*sym.cos(theta_t[i])],
                                        [sym.sin(theta_t[i]), sym.cos(theta_t[i])*round(m.cos(alpha_t[i])), -sym.cos(theta_t[i])*round(m.sin(alpha_t[i])), a_t[i]*sym.sin(theta_t[i])],
                                        [0, round(m.sin(alpha_t[i])), round(m.cos(alpha_t[i])), d_t[i]],
                                        [0, 0, 0, 1]])
        transformation_matrices[i, :, :] = transformation_matrices[i-1, :, :] * trans_mat

    return transformation_matrices 

# Hard coded transformation matrix
def hard_trans_matrix(theta_t: mat.Matrix = theta, dof = DOF):
    transform_matrix = np.zeros((dof+1, DIM, DIM), dtype='object')

    transform_matrix[0, :, :] = mat.Matrix([[1, 0, 0, 0],
                                [0, 0, -1, 0],
                                [0, 1, 0, d[0]],
                                [0, 0, 0, 1]])
    
    # T: -(pi/2 - q1); A: -pi/2
    trans_mat = mat.Matrix([[sym.sin(q1), 0, sym.cos(q1), 0],
                            [-sym.cos(q1), 0, sym.sin(q1), 0],
                            [0, -1, 0, d[1]],
                            [0, 0, 0, 1]])
    
    transform_matrix[1, :, :] = transform_matrix[0, :, :] * trans_mat

    # T: -(pi/2 + q2); A: -pi/2
    trans_mat = mat.Matrix([[-sym.sin(q2), 0, sym.cos(q2), 0],
                            [-sym.cos(q2), 0, -sym.sin(q2), 0],
                            [0, -1, 0, d[2]],
                            [0, 0, 0, 1]])

    transform_matrix[2, :, :] = transform_matrix[1, :, :] * trans_mat

    # T: -(pi/2 - q3); A: -pi/2
    trans_mat = mat.Matrix([[sym.sin(q3), 0, sym.cos(q3), 0],
                            [-sym.cos(q3), 0, sym.sin(q3), 0],
                            [0, -1, 0, d[3]],
                            [0, 0, 0, 1]])

    transform_matrix[3, :, :] = transform_matrix[2, :, :] * trans_mat

    # T: -(pi/2 - q4); A: -pi/2
    trans_mat = mat.Matrix([[sym.sin(q4), 0, sym.cos(q4), 0],
                            [-sym.cos(q4), 0, sym.sin(q4), 0],
                            [0, -1, 0, d[4]],
                            [0, 0, 0, 1]])

    transform_matrix[4, :, :] = transform_matrix[3, :, :] * trans_mat

    # T: q5; A: 0
    trans_mat = mat.Matrix([[sym.cos(q5), -sym.sin(q5), 0, 0],
                            [sym.sin(q5), sym.cos(q5), 0, 0],
                            [0, 0, 1, d[5]],
                            [0, 0, 0, 1]])

    transform_matrix[5, :, :] = transform_matrix[4, :, :] * trans_mat

    return transform_matrix 

# Generate symbolic position matrix
def position_matrix(trans_mat, dof: int = DOF):

    pos_mat = np.zeros((dof+2, DIM, 1), dtype='object')

    pos_mat[0, :, :] = ORIGIN

    for i in range(dof + 1):
        pos_mat[i+1, :, :] = trans_mat[i, :, :] * ORIGIN

    return pos_mat

def write_matrix(pos_mat, file_name: str = "position_matrices"):
    with open(f'{file_name}.txt', "w") as f:
        f.write(f'DH Table\n============\na | {str(a)}\nd | {str(d)}\nalpha | {str(alpha)}\ntheta | {str(theta)}\n============\n')
        f.write(pos_mat)

# Get angles
def get_angles(pos_matrix, x: float = REST_X, y: float = REST_Y, z: float = REST_Z):

    # Checks
    x_chk = False
    y_chk = False
    z_chk = False

    # Joint coordinates
    shoulder_co = np.array([[0], [-L2], [L1]])
    effector_co = np.array([[x], [y], [z]])

    """
    Boundary conditions - Not sure how to implement
    [min max]
    c_q1 = [-(sym.pi/3), sym.pi]
    c_q2 = [0, sym.pi]
    c_q3 = [-(sym.pi/2), sym.pi/3]
    c_q4 = [-(sym.pi/2), sym.pi/3]
    """

    # Required length of arm
    ef_length = np.linalg.norm(effector_co-shoulder_co)

    if (ef_length > L3 + L4):     # Check arm length
        print(f'END\nDelta: {ef_length - (L3 + L4)}')
        print('Out of Range\n')
        return
    # Special case where arm at origin
    elif (x == REST_X) and (y == REST_Y) and (z == REST_Z):
        return [0, 0, 0, 0, 0]
    # Special case where y is matching
    elif (y == REST_Y):
        y_chk = True
    # Special case where z is same as rest and distance is same as forearm
    elif (z == REST_Z) and (np.linalg.norm(effector_co-np.array([[0], [-L2], [REST_Z]]))==L4):
        z_chk = True
    elif (x == REST_X):
        x_chk = True
    else:
        pass
    
    # Joint to solve coordinates
    elbow_co = sym.Matrix(pos_matrix[5, :3, 0])
    e_eff_co = sym.Matrix(pos_matrix[6, :3, 0])
                      
    # Solve for elbow first (q1, q2)
    # Get equations for elbow
    e_eq1 = sym.Eq(elbow_co[2], z)      # Level elbow
    # Distance from elbow to end point
    e_dist = elbow_co-effector_co
    e_dist = sym.sqrt(e_dist[0]**2 + e_dist[1]**2)
    e_eq2 = sym.Eq(e_dist, L4)
    # Get equations for end effector
    ee_eq1 = sym.Eq(e_eff_co[0], x)
    ee_eq2 = sym.Eq(e_eff_co[1], y)
    ee_eq3 = sym.Eq(e_eff_co[2], z)
    # Other equations
    eq1 = sym.Eq(e_eff_co[2]-elbow_co[2], 0)    # Forearm is parallel with floor
    # Extra y-constraint if needed
    eq2 = sym.Eq(e_eff_co[1]-elbow_co[1], 0)

    if y_chk:
        e_eq1 = e_eq1.subs(q2, 0)
        e_eq2 = e_eq2.subs(q2, 0)
        ee_eq1 = ee_eq1.subs(q2, 0)
        ee_eq2 = ee_eq2.subs(q2, 0)
        ee_eq3 = ee_eq3.subs(q2, 0)
        eq1 = eq1.subs(q2, 0)
        answer = sym.nsolve((e_eq1, e_eq2, ee_eq1, ee_eq2, ee_eq3, eq1), (q1, q3, q4), (0, 0, 0), dict=True, verify=False, check=False)
        answer[0].update({q2: 0})
    elif z_chk:
        e_eq1 = e_eq1.subs([(q2, 0), (q1, 0), (q4, 0)])
        e_eq2 = e_eq2.subs([(q2, 0), (q1, 0), (q4, 0)])
        ee_eq1 = ee_eq1.subs([(q2, 0), (q1, 0), (q4, 0)])
        ee_eq2 = ee_eq2.subs([(q2, 0), (q1, 0), (q4, 0)])
        ee_eq3 = ee_eq3.subs([(q2, 0), (q1, 0), (q4, 0)])
        eq1 = eq1.subs([(q2, 0), (q1, 0), (q4, 0)])
        answer = sym.nsolve((e_eq1, e_eq2, ee_eq1, ee_eq2, ee_eq3, eq1), (q3), (-sym.pi/2, sym.pi/3), dict=True, verify=False, check=False)
        answer[0].update({q1: 0, q2: 0, q4: 0})
    elif x_chk:
        e_eq1 = e_eq1.subs(q1, 0)
        e_eq2 = e_eq2.subs(q1, 0)
        ee_eq1 = ee_eq1.subs(q1, 0)
        ee_eq2 = ee_eq2.subs(q1, 0)
        ee_eq3 = ee_eq3.subs(q1, 0)
        eq1 = eq1.subs(q1, 0)
        answer = sym.nsolve((e_eq1, e_eq2, ee_eq1, ee_eq2, ee_eq3, eq1), (q2, q3, q4), (0, 0, 0), dict=True, verify=False, check=False)
        answer[0].update({q1: 0})
    else:
        answer = sym.nsolve((e_eq1, e_eq2, ee_eq1, ee_eq2, ee_eq3, eq1), (q1, q2, q3, q4), (0, 0, 0, 0), dict=True, verify=False, check=False)
    
    if not len(answer):
        print("No valid solutions.")
        return
    
    # Get angles
    angle = [answer[0].get(q1), answer[0].get(q2), answer[0].get(q3), answer[0].get(q4)]
    angle = [round(m.degrees((float(rad)))%360) for rad in angle]
    
    # Get angle in range -180 to 180 degrees
    chk_ang = lambda ang: ang - 360 if (ang > 180) else (ang + 360 if ang < -180 else ang)
    angle = [chk_ang(ang) for ang in angle]

    angle.append(-angle[1])     # q5 where q5 = -q2

    return angle

def calc_angles(angles):        # Convert angles for dynamixel motors
    steps = [(angle/(360/4096)) for angle in angles]   # Anti-clockwise Positive 
    return [np.rint(step) for step in steps]

def fk_check(angles: list, input_coord: list = [REST_X, REST_Y, REST_Z]):
    print("FK CHECK")
    fk_coord = fk_end(angles)
    delta = []
    for i in range(3):
        delta.append(round(abs(input_coord[i]-fk_coord[i]), 1))
    print(f'End\t\t\tDelta\nX\tY\tZ\tX\tY\tZ\n{fk_coord[0]}\t{fk_coord[1]}\t{fk_coord[2]}\t{delta[0]}\t{delta[1]}\t{delta[2]}')

def print_list(lists: list):
    strings = ""
    for item in lists:
        strings += str(item)+"\t"
    print(strings)

# Test Coordinates - Change HERE
TEST_COORD = [370, -350, 360]
TEST = True

# Automated Transformation Matrices
def main():
    tm = trans_matrix()
    pos = position_matrix(tm)
    write_matrix(str(pos), "position_matrices_main")
    origin_reset = False
    if TEST:
        coord = TEST_COORD
    else:
        coord = get_position()
    angles = get_angles(pos, coord[0], coord[1], coord[2])    
    if angles is not None:
        print("q1\tq2\tq3\tq4\tq5")
        print_list(angles)
        fk_check(angles, coord)
        # checking if angle is within constraints (hasn't been applied yet)
        cons_min = [-60, 0, -90, -90, -180]
        cons_max = [180, 180, 60, 60, 180]
        for i in range(len(angles)):
            if angles[i] not in range(cons_min[i], cons_max[i]):
                origin_reset = True
                print('q%d angle out of bounds' % (i+1))
    if origin_reset:
        angles = [0,0,0,0,0]
        print('Arm has been reset to origin')

# Hardcoded Transformation Matrices - slightly faster
def hard_main():
    tm = hard_trans_matrix()
    pos = position_matrix(tm)
    write_matrix(str(pos), "position_matrices_hard")
    origin_reset = False
    if TEST:
        coord = TEST_COORD
    else:
        coord = get_position()
    angles = get_angles(pos, coord[0], coord[1], coord[2])    
    if angles is not None:
        print("q1\tq2\tq3\tq4\tq5")
        print_list(angles)
        fk_check(angles, coord)
        cons_min = [-60, 0, -90, -90, -180]
        cons_max = [180, 180, 60, 60, 180]
        for i in range(len(angles)):
            if angles[i] not in range(cons_min[i], cons_max[i]):
                origin_reset = True
                print('q%d angle out of bounds' % (i+1))
    
    if origin_reset:
        angles = [0,0,0,0,0]
        print('Arm has been reset to origin')

if __name__ == "__main__":
    print("Main")
    main()
    print("====================")
    print("Hard Main")
    hard_main()
