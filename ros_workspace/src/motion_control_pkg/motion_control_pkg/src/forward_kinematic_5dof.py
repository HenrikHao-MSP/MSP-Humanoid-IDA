import numpy as np
import sys
import math as m

## Constants
DOF = 5     # Degrees of freedom
DIM = 4     # Dimensions of matrices
DECI_PLACE = 1
ORIGIN = np.array([[0], [0], [0], [1]], dtype=float)  # [X, Y, Z, 1]

## Arm Lengths in mm
L1 = 300    # Origin to motor 1 of shoulder
L2 = 30     # Shoulder to motor 2 of shoulder
L3 = 265    # Upper arm length
L4 = 235    # Elbow to wrist length

## Get input angles via sys or prompt in degrees and returns Theta values for DH table
def get_theta():
    if len(sys.argv) == DOF+1:
        angles = [float(arg) for arg in sys.argv[1:]]       
    else:
        angle = input("Enter Angles of Joints: <q1> <q2> <q3> <q4> <q5>\n").split(" ")
        if len(angle) == 0:
            exit()
        elif len(angle) != DOF:
            print("Invalid Inputs")
        else:
            print("q1\tq2\tq3\tq4\tq5")
            print(f'{angle[0]}\t{angle[1]}\t{angle[2]}\t{angle[3]}\t{angle[4]}\t')
            angles = [float(ang) for ang in angle]

    return np.array([0, m.radians(angles[0]-90), m.radians(-angles[1]-90), 
        m.radians(angles[2]-90), m.radians(angles[3]-90), m.radians(angles[4])], dtype=float)          # in radians

## DH Table (Global)
a = np.array([0, 0, 0, 0, 0, 0], dtype=float)                                               # in mm
alpha = np.array([(m.pi/2), -(m.pi/2), -(m.pi/2), -(m.pi/2), -(m.pi/2), 0], dtype=float)    # in radians
d = np.array([L1, L2, 0, L3, 0, L4], dtype=float)   
if __name__ == "__main__":                                        # in mm
    theta = get_theta()
else:
    theta = np.zeros((DOF+1, 1), dtype=float)

## Transformation Matrix from DH Table
def trans_matrix(a_t: np.ndarray = a, alpha_t: np.ndarray = alpha, d_t: np.ndarray = d, theta_t: np.ndarray = theta, dof = DOF):
    transform_matrix = np.zeros((dof+1, DIM, DIM), dtype=float)

    # Initial transformation matrix
    transform_matrix[0, :, :] = np.array([[1, 0, 0, 0],
                            [0, 0, -1, 0],
                            [0, 1, 0, d[0]],
                            [0, 0, 0, 1]], dtype=float)

    # Update transform matrix for each position
    for i in range(1, dof+1):
        transform_matrix[i, :, :] = np.array([[m.cos(theta_t[i]), -m.sin(theta_t[i])*m.cos(alpha_t[i]), m.sin(theta_t[i])*m.sin(alpha_t[i]), a_t[i]*m.cos(theta_t[i])],
                                        [m.sin(theta_t[i]), m.cos(theta_t[i])*m.cos(alpha_t[i]), -m.cos(theta_t[i])*m.sin(alpha_t[i]), a_t[i]*m.sin(theta_t[i])],
                                        [0, m.sin(alpha_t[i]), m.cos(alpha_t[i]), d_t[i]],
                                        [0, 0, 0, 1]], dtype=float)
        np.matmul(transform_matrix[i-1, :, :], transform_matrix[i, :, :], out=transform_matrix[i, :, :])

    return transform_matrix 

### For Reference Only ###
# def optimised_trans_matrix(theta_t: np.ndarray = theta, dof = DOF):
#     transform_matrix = np.zeros((DIM, DIM, dof+1), dtype=float)

#     transform_matrix[0, :, :] = np.array([[1, 0, 0, 0],
#                                 [0, 0, -1, 0],
#                                 [0, 1, 0, d[0]],
#                                 [0, 0, 0, 1]], dtype=float)
    
#     transform_matrix[:, :, 1] = np.array([[m.cos(theta_t[1]), 0, -m.sin(theta_t[1]), 0],
#                                 [m.sin(theta_t[1]), 0, m.cos(theta_t[1]), 0],
#                                 [0, -1, 0, d[1]],
#                                 [0, 0, 0, 1]], dtype=float)
    
#     np.matmul(transform_matrix[0, :, :], transform_matrix[:, :, 1], out=transform_matrix[:, :, 1])

#     transform_matrix[:, :, 2] = np.array([[m.cos(theta_t[2]), 0, -m.sin(theta_t[2]), 0],
#                                 [m.sin(theta_t[2]), 0, m.cos(theta_t[2]), 0],
#                                 [0, -1, 0, 0],
#                                 [0, 0, 0, 1]], dtype=float)
    
#     np.matmul(transform_matrix[:, :, 1], transform_matrix[:, :, 2], out=transform_matrix[:, :, 2])
    
#     transform_matrix[:, :, 3] = np.array([[m.cos(theta_t[3]), 0, -m.sin(theta_t[3]), 0],
#                                 [m.sin(theta_t[3]), 0, m.cos(theta_t[3]), 0],
#                                 [0, -1, 0, d[3]],
#                                 [0, 0, 0, 1]], dtype=float)

#     np.matmul(transform_matrix[:, :, 2], transform_matrix[:, :, 3], out=transform_matrix[:, :, 3])

#     transform_matrix[:, :, 4] = np.array([[m.cos(theta_t[4]), 0, -m.sin(theta_t[4]), 0],
#                                 [m.sin(theta_t[4]), 0, m.cos(theta_t[4]), 0],
#                                 [0, -1, 0, 0],
#                                 [0, 0, 0, 1]], dtype=float)

#     np.matmul(transform_matrix[:, :, 3], transform_matrix[:, :, 4], out=transform_matrix[:, :, 4])

#     transform_matrix[:, :, 5] = np.array([[m.cos(theta_t[5]), -m.sin(theta_t[5]), 0, 0],
#                                 [m.sin(theta_t[5]), m.cos(theta_t[5]), 0, 0],
#                                 [0, 0, 1, d[5]],
#                                 [0, 0, 0, 1]], dtype=float)

#     np.matmul(transform_matrix[:, :, 4], transform_matrix[:, :, 5], out=transform_matrix[:, :, 5])

#     return transform_matrix 

# Gets array for position of motors in [X, Y, Z, 1]
# Call pos[:3, :, <motor>] for Coordinate of Motor
def get_joints(transformation_matrix: np.ndarray):
    pos = np.zeros((DOF+2, DIM, 1), dtype=float)
    pos[0, :, :] = ORIGIN

    for i in range(DOF+1):
        np.matmul(transformation_matrix[i, :, :], ORIGIN, out=pos[i+1, :, :])
    
    return pos

def fk_all(angles: list):
    ## DH Table (Local)
    a = np.array([0, 0, 0, 0, 0, 0], dtype=float)                                               # in mm
    alpha = np.array([(m.pi/2), -(m.pi/2), -(m.pi/2), -(m.pi/2), -(m.pi/2), 0], dtype=float)    # in radians
    d = np.array([L1, L2, 0, L3, 0, L4], dtype=float)                                           # in mm
    theta = np.array([0, m.radians(angles[0]-90), m.radians(-angles[1]-90), 
                    m.radians(angles[2]-90), m.radians(angles[3]-90), m.radians(angles[4])], dtype=float)          # in radians

    otm = trans_matrix(a, alpha, d, theta)
    pos = get_joints(otm) 

    return pos

def fk_end(angles: list):
    ## DH Table (Local)
    a = np.array([0, 0, 0, 0, 0, 0], dtype=float)                                               # in mm
    alpha = np.array([(m.pi/2), -(m.pi/2), -(m.pi/2), -(m.pi/2), -(m.pi/2), 0], dtype=float)    # in radians
    d = np.array([L1, L2, 0, L3, 0, L4], dtype=float)                                           # in mm
    theta = np.array([0, m.radians(angles[0]-90), m.radians(-angles[1]-90), 
                    m.radians(angles[2]-90), m.radians(angles[3]-90), m.radians(angles[4])], dtype=float)          # in radians

    otm = trans_matrix(a, alpha, d, theta)
    pos = get_joints(otm)

    # Print positions
    end_pos = [round(float(ele), DECI_PLACE) for ele in np.nditer(pos[DOF+1, :3, :])]

    return end_pos

def main():
    otm = trans_matrix()
    pos = get_joints(otm)
    elb_pos = [round(float(ele), DECI_PLACE) for ele in np.nditer(pos[4, :3, :])]
    print("Elbow\nX\tY\tZ\t")
    print(f'{elb_pos[0]}\t{elb_pos[1]}\t{elb_pos[2]}\t')
    end_pos = [round(float(ele), DECI_PLACE) for ele in np.nditer(pos[DOF+1, :3, :])]
    print("End\nX\tY\tZ\t")
    print(f'{end_pos[0]}\t{end_pos[1]}\t{end_pos[2]}\t')
    return

if __name__ == "__main__":
    main()