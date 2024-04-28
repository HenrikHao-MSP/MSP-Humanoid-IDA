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
DOF = 5
# Change to device appropriate device/com port
DEVICENAME = "/dev/ttyUSB0"

MINIMUM = [dyna.MM_SHOULDER_FLEX_EX["min"],
           dyna.MM_SHOULDER_ABDUCTION["min"],
           dyna.MM_SHOULDER_ROT["min"],
           dyna.MM_ELBOW_FLEX_EX["min"],
           dyna.MM_PRO_SUP["min"]]

MAXIMUM = [dyna.MM_SHOULDER_FLEX_EX["max"],
           dyna.MM_SHOULDER_ABDUCTION["max"],
           dyna.MM_SHOULDER_ROT["max"],
           dyna.MM_ELBOW_FLEX_EX["max"],
           dyna.MM_PRO_SUP["max"]]

RESTING = [dyna.MM_SHOULDER_FLEX_EX["zero"],
           dyna.MM_SHOULDER_ABDUCTION["zero"],
           dyna.MM_SHOULDER_ROT["zero"],
           dyna.MM_ELBOW_FLEX_EX["zero"],
           dyna.MM_PRO_SUP["zero"]]

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
        #self.subscription = self.create_subscription(
        #    Bool, 
        #    'pouring_complete', 
        #    self.pouring_complete_callback,
        #    10
        #)
        self.subscription = self.create_subscription(
            Float32,
            'liquid_level',
            self.liquid_level_callback,
            10)
        self.publisher_ = self.create_publisher(
            Bool,
            'arm_move_success',  # Topic name for the success flag
            10)
        self.subscription  # prevent unused variable warning
        ### Initiate motors and kinematics ###
        self._kinematics = k.Kinematics()
        self._motors = dyna.Motors(DOF, DEVICENAME)
        self._target_pos = []
        self._start_motors()
        self.liquid_level_threshold = 80.0  # Example threshold, adjust as needed
        self.finish_pour = False

    def bottle_info_callback(self, msg):
        self.get_logger().info(f'Received bottle info: {msg.position}')
        # Control logic to move the arm to the bottle's position
        success = self.move_to_coord(msg.position)
        # Publish the success flag
        success_msg = Bool()
        success_msg.data = success
        self.publisher_.publish(success_msg)
        self.get_logger().info(f'Published move success flag: {success}')

    #def pouring_complete_callback(self, msg):
    #    self.get_logger().info(f'Pouring is complete: {msg}')
    #    self.finish_pour = Bool()
    #    self.finish_pour = msg
    
    ### Main function call to move arm ###
    def move_to_coord(self, coord: list) -> bool:
        # IK function
        self._calc_angles(coord)
        # Convert to position of motors
        self._angle_to_dynamixel()
        # Checks if within limits
        if self._check_limits():
            # Move to arm position
            self.get_logger().info(f'Moving arm to: {coord[0]} {coord[1]} {coord[2]}')
            self._motors.set_goal(self._target_pos)
            # Continually check movement
            while self._motors.check_moving():
                continue
            # Check end position 
            c_pos = self.get_current_pos()
            if c_pos != self._target_pos:
                self.get_logger().info(f'Unable to reach position: {coord[0]} {coord[1]} {coord[2]}')
                return False
            return True     # successful movement
        else:
            self.get_logger().info(f'Unable to reach position: {coord[0]} {coord[1]} {coord[2]}')
            return False    
        
    def pour(self):
        # set angle of motor to tilt from upright to below 90 
        c_pos = self.get_current_pos()
        self._target_pos[DOF-1] = self._target_pos[DOF-1] - round(100*dyna.ANGLE_TO_DYNA)
        self._motors.set_goal(self._target_pos)
        # hold that until get the msg that pouring is complete 
        while not self.finish_pour:
            continue
        # set angle back to previous value 
        self._target_pos = c_pos
        self._motors.set_goal(self._target_pos)
        self.finish_pour = False
        pass

    # copied from pouring_node
    def liquid_level_callback(self, msg):
        current_level = msg.data
        self.get_logger().info(f'Current liquid level: {current_level}%')
        if current_level > self.liquid_level_threshold:
            #self.stop_pouring()
            self.finish_pour = True

    # copied from pouring_node
    #def stop_pouring(self):
    #    # Placeholder for actual arm stop command
    #    self.get_logger().info('Liquid level threshold exceeded. Stopping pouring...')
    #    # Insert your arm control logic here to stop pouring
    #    pour_msg = Bool()
    #    pour_msg.data = True
    #    self.publisher_.publish(pour_msg)

    def grip(self):
        # implemented by EE
        pass

    ###### general helpers ######
    ### IK Function - Updates _kinematics.angles ###
    def _calc_angles(self, coord: list) -> None:
        self._kinematics.get_angles(coord[0], coord[1], coord[2])
        return
    ### Start motors ###
    def _start_motors(self) -> None:
        self._motors.torque_toggle(1)
        self._motors.set_goal(RESTING)
        self._motors.get_current_pos()
        return
    ### Convert angles to dynamixel positions - Updates self._target_pos ### 
    def _angle_to_dynamixel(self) -> None:
        self._target_pos = [round(angle*dyna.ANGLE_TO_DYNA) for angle in self._kinematics.angles]
        return
    ### Check limits from input - limits have already been set for arm ###
    def _check_limits(self) -> bool:
        for i in range(DOF):
            if self._target_pos[i] not in range(MINIMUM[i], MAXIMUM[i]):
                self.get_logger().info(f'q{i+1} outside valid range.')
                return False
        return True
    def _check_limits(self, id: int, pos: int) -> bool:
        if pos not in range(MINIMUM[id-1], MAXIMUM[id-1]):
            self.get_logger().info(f'q{id} outside valid range.')
            return False
        else:
            return True
    ### Specify motor to move
    def _move_part(self, id: int, dyna_pos: int) -> None:
        if self._check_limits(id, dyna_pos):
            self._motors.set_goal(id, dyna_pos)
        return
    ### Returns current dynamixel positions ###
    def get_current_pos(self) -> list:
        self._motors.get_current_pos()
        return self._motors.pos

    ### End session ###    
    def end_session(self) -> None:
        self._motors.port_close()
        return
    
    def pathing(self, cur_coord, coord):
        path = self.path_gen(STEP_NUM, cur_coord, coord)
        i = 0
        err = 1
        while (i < STEP_NUM):
            i_step = [path[0][i], path[1][i], path[2][i]]
            steps = self.abs_steps(*i_step)
            if steps == ERR:
                err = ERR
                self.pathing(prev_step, ARM_ORIGIN)
                break
            prev_step = i_step

            # motor control
            #runarm(*steps)
            i+=1
        if err == ERR:
            return err
    ### Delay function ###    
    def delay(self, time):
        start = t.time()
        while t.time()-start < time:
            continue

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
        self.get_logger().info(path)
        # self.get_logger().info(path[0][0])
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
    arm_control_node.end_session()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
