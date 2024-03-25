import rclpy
import sys
import sympy as sym
import sympy.matrices as mat
import math as m
import numpy as np
import time as t
from rclpy.node import Node
from interfaces.msg import BottleInfo
from std_msgs.msg import Bool

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
NUM_STEPS = 1000
# DH table
q1 = sym.symbols('q1')
q2 = sym.symbols('q2')
q3 = sym.symbols('q3')
q4 = sym.symbols('q4')
q5 = sym.symbols('q5')
a = [0, 0, 0, 0, 0, 0]
alpha = mat.Matrix([(sym.pi/2), -(sym.pi/2), -(sym.pi/2), -(sym.pi/2), -(sym.pi/2), 0])
d = mat.Matrix([L1, L2, 0, L3, 0, L4])
theta = mat.Matrix([0, q1-(sym.pi/2), -(sym.pi/2)-q2, q3-(sym.pi/2), q4-(sym.pi/2), q5])
# path gen
x_dot0 = 0
y_dot0 = 0
z_dot0 = 0
x_dotf = x_dot0
y_dotf = y_dot0
z_dotf = z_dot0
# main arm 
ERR = 0
STEP_NUM = 20
ARM_ORIGIN = [L4, L2, L1-L3]

class ArmControlNode(Node):
    def __init__(self):
        super().__init__('arm_control_node')
        self.subscription = self.create_subscription(
            BottleInfo,
            'bottle_info',
            self.bottle_info_callback,
            10)
        self.publisher_ = self.create_publisher(
            Bool,
            'arm_move_success',  # Topic name for the success flag
            10)
        self.subscription  # prevent unused variable warning

    def bottle_info_callback(self, msg):
        self.get_logger().info(f'Received bottle info: {msg.position}')
        # Control logic to move the arm to the bottle's position
        success = self.move_arm_to_position(msg.position)
        # Publish the success flag
        success_msg = Bool()
        success_msg.data = success
        self.publisher_.publish(success_msg)
        self.get_logger().info(f'Published move success flag: {success}')

    def move_arm_to_position(self, position):
        # Placeholder for arm control logic

        # check if position is reachable
        check = self.get_angles(position)
        #TODO check if get_angles returns ERR
        if check == ERR:
            return ERR
        else:
            # assuming given a position, arm to reach that position, pour a drink, and return to origin
            path = self.pathing(ARM_ORIGIN, position)
            if path == ERR:
                return ERR
            self.delay(1.5)
            current_coord = position

            #TODO: motor control
            #pour()

            self.delay(1)
            path = self.pathing(current_coord, ARM_ORIGIN)
            if path == ERR:
                return ERR
            self.delay(1)
            
            # Here, simulate moving the arm and return True for success
            self.get_logger().info(f'Moving arm to position: {position}')
            # Simulate a successful operation
            return True
    
    ###### general helpers ######
    def pathing(self, cur_coord, coord):
        path = self.path_gen(STEP_NUM, cur_coord, coord)
        i = 0
        err = 1
        while (i < STEP_NUM):
            i_step = [path[0][i], path[1][i], path[2][i]]
            steps = self.abs_steps(*i_step)
            if steps == ERR:
                err = ERR
                self.pathing(prev_step, ORIGIN)
                break
            prev_step = i_step

            # motor control
            #runarm(*steps)
            i+=1
        if err == ERR:
            return err
        
    def delay(self, time):
        start = t.time()
        while t.time()-start < time:
            continue


    ###### IK helpers ###### 
    # main IK function, hard coded
    def inverse_kinematics(self, coord):
        tm = self.hard_trans_matrix()
        pos = self.position_matrix(tm)
        # write_matrix(str(pos), "position_matrices_hard")
        origin_reset = False

        # TODO: change input of positon
        #coord = self.get_position()
        angles = self.get_angles(pos, coord[0], coord[1], coord[2])    
        if angles is not None:
            print("q1\tq2\tq3\tq4\tq5")
            #print_list(angles)
            #fk_check(angles, coord)
            cons_min = [-60, 0, -90, -90, -180]
            cons_max = [180, 180, 60, 60, 180]
            for i in range(len(angles)):
                if angles[i] not in range(cons_min[i], cons_max[i]):
                    origin_reset = True
                    print('q%d angle out of bounds' % (i+1))
        
        if origin_reset:
            angles = [0,0,0,0,0]
            print('Arm has been reset to origin')

    # Hard coded transformation matrix
    def hard_trans_matrix(self, theta_t: mat.Matrix = theta, dof = DOF):
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
    def position_matrix(self, trans_mat, dof: int = DOF):
        pos_mat = np.zeros((dof+2, DIM, 1), dtype='object')
        pos_mat[0, :, :] = ORIGIN

        for i in range(dof + 1):
            pos_mat[i+1, :, :] = trans_mat[i, :, :] * ORIGIN

        return pos_mat

    #TODO: check if needed
    def write_matrix(pos_mat, file_name: str = "position_matrices"):
        with open(f'{file_name}.txt', "w") as f:
            f.write(f'DH Table\n============\na | {str(a)}\nd | {str(d)}\nalpha | {str(alpha)}\ntheta | {str(theta)}\n============\n')
            f.write(pos_mat)

    #TODO: check if needed
    # Get input positions
    def get_position(self):
        if len(sys.argv) == 4:
            return [float(coord) for coord in sys.argv[1:]]   # Returns [x y z]
        else:
            coord = input("Enter Coordinates: <x> <y> <z>\n").split(" ")
            if len(coord) == 0:
                exit()
            if len(coord) != 3:
                print("Invalid Inputs")
                print(*coord)
                self.get_position()
            return [float(co) for co in coord]    # Returns [x y z]

    # note: boundary conditions not yet implemented
    def get_angles(self, pos_matrix, x: float = REST_X, y: float = REST_Y, z: float = REST_Z):
        # Checks
        x_chk = False
        y_chk = False
        z_chk = False

        # Joint coordinates
        shoulder_co = np.array([[0], [-L2], [L1]])
        effector_co = np.array([[x], [y], [z]])

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
        eq1 = sym.Eq(e_eff_co[2]-elbow_co[2], 0)
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

    def abs_steps(self, x, y, z):
        steps = []
        d_steps = self.get_angles(x, y, z)
        if d_steps == 0:
            return 0
        for i in range(DOF):
            steps.append(int((ORIGIN[i]+d_steps[i])%STEPS[i]))
        return steps


    ##### path gen helpers ###### 
    def path_gen(self, steps, start=None, final=None):
        # changed slightly from original code
        x_factors = self.poly_factors(start[0], x_dot0, final[0], x_dotf)
        y_factors = self.poly_factors(start[1], y_dot0, final[1], y_dotf)
        z_factors = self.poly_factors(start[2], z_dot0, final[2], z_dotf)

        x_steps = self.steps_generation(steps, x_factors)
        y_steps = self.steps_generation(steps, y_factors)
        z_steps = self.steps_generation(steps, z_factors)
        path = np.array([x_steps, y_steps, z_steps])
        print(path)
        # print(path[0][0])
        return path
    
    def poly_factors(self, ini_coor, ini_v, fin_coor, fin_v):
        TF = 1
        A = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [1, TF, pow(TF, 2), pow(TF, 3)], [0, 1, 2 * TF, 3 * pow(TF, 2)]])
        B = np.array([ini_coor, ini_v, fin_coor, fin_v])
        X = np.linalg.inv(A).dot(B)

        return X

    def steps_generation(self, num_steps, factors):
        steps = []
        unit_t = 1/num_steps
        t = 0

        for i in range(num_steps):
            t = t + unit_t
            steps.append(float(factors[0]) + float(factors[1]) * t + float(factors[2]) * pow(t, 2) + float(factors[3]) * pow(t, 3))

        return steps


def main(args=None):
    rclpy.init(args=args)
    arm_control_node = ArmControlNode()
    rclpy.spin(arm_control_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
