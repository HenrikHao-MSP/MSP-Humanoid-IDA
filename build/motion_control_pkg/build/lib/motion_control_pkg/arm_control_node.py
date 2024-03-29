import rclpy
import sys
import kinematics as k
import dynamixel_2_0 as dyna
import math as m
import numpy as np
import time as t
from rclpy.node import Node
from interfaces.msg import BottleInfo
from std_msgs.msg import Bool

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
ARM_ORIGIN = [k.L4, k.L2, k.L1-k.L3]

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
        self.kinematics = k.Kinematics()
        self.angles = []
        self.dynamixel_in = []

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
        self.angles = self.kinematics.get_angles(coord[0], coord[1], coord[2])
        
        origin_reset = False

        # TODO: change input of positon and update max values
        if self.angles is not None:
            print("q1\tq2\tq3\tq4\tq5")
            cons_min = [dyna.MM_SHOULDER_FLEX_EX["min"], 
                        dyna.MM_SHOULDER_ABDUCTION["min"], 
                        dyna.MM_SHOULDER_ROT["min"], 
                        dyna.MM_ELBOW_FLEX_EX["min"], 
                        dyna.MM_PRO_SUP["min"]]
            cons_min = self.angle_to_dyna(cons_min)
            cons_max = [dyna.MM_SHOULDER_FLEX_EX["max"], 
                        dyna.MM_SHOULDER_ABDUCTION["max"], 
                        dyna.MM_SHOULDER_ROT["max"], 
                        dyna.MM_ELBOW_FLEX_EX["max"], 
                        dyna.MM_PRO_SUP["max"]]
            cons_max = self.angle_to_dyna(cons_max)
            for i in range(len(self.angles)):
                if self.angles[i] not in range(cons_min[i], cons_max[i]):
                    origin_reset = True
                    print('q%d angle out of bounds' % (i+1))
        
        if origin_reset:
            self.angles = [0,0,0,0,0]
            print('Arm has been reset to origin.')
        return

    def angle_to_dyna(self, angles: list) -> list:
        dyna_input = [round(angle*dyna.ANGLE_TO_DYNA) for angle in angles]
        return dyna_input

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
