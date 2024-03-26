import sys
import DynamixelSDK.python.src.dynamixel_sdk as dxl
from dynamixel_consts import * 

# Control table values - will need to find safe values
PROF_VEL = 50           # Max speed 0-32767 * 1.374 deg/s
PROF_ACC = 50           # Max accel 0-32767 * 21.4577 deg/s^2
MX_ACCEL = 60           # Max accel 0-254 * 8.583 deg/s^2

# Default settings for U2D2
DEVICENAME = 'COM8'  # Replace with the appropriate device name

# Initialize the U2D2 and Dynamixel motors
PACKET_HANDLER = dxl.PacketHandler(PROTOCOL_VERSION)

class Motors:
    """
    Initialise with DoF, DeviceName (ie COM Port) and Baudrate
    """
    def __init__(self, dof: int, devicename: str=DEVICENAME, baudrate: int=57600) -> None:
        self._port_handler = dxl.PortHandler(devicename)
        self.baudrate = baudrate
        self.num_motors = dof
        self.torque_status = []
        self.pos = []
        self.load = []
        self._port_open()
        self._sync_torque()
        self._set_limits()
        self.motors = self._get_motors()
        self._set_profile()
        # self.torque_toggle(set=1)       # Activate motor

    # Port management    
    """
    Open port and setup baudrate
    If fails, to exit program
    Check connection and restart
    """
    def _port_open(self) -> None:   
        # Open the port
        port_attempt = 0
        print("Attempting to open the port...")
        while port_attempt < 3:
            port_attempt += 1
            try:
                self._port_handler.openPort()
                print("\tSucceeded to open the port.")
                break
            except:
                print("...")
        if port_attempt == 3:
            print("\tFailed to open the port.\nExiting program.")
            exit(1)
        
        # Set baudrate
        baud_attempt = 0
        print("Attempting to set baudrate...")
        while baud_attempt < 3:
            baud_attempt += 1
            try: 
                self._port_handler.setBaudRate(self.baudrate)
                print("\tSucceeded to set the baudrate.")
                break
            except:
                print("...")
        if baud_attempt == 3:
            print("\tFailed to set the baudrate.\nExiting program.")
            exit(1)
        return

    def port_close(self) -> None:   # Close the port and end session
        print("Closing port...")
        self.torque_toggle(set=0)
        self._port_handler.closePort()
        print("Closed.")
        return

    def _get_motors(self) -> list:   # Get motor types
        motor_types = []
        for i in range(self.num_motors):
            m_type = PACKET_HANDLER.read2ByteTxRx(self._port_handler, i+1, ADDR_MODEL_NUM)
            motor_types.append(m_type)
        print(motor_types)
        return motor_types

    # Motor Functions    
    def set_goal(self, target_pos: list) -> None:
        for i in range(len(target_pos)):
            PACKET_HANDLER.write4ByteTxRx(self._port_handler, i+1, ADDR_GOAL_POS, target_pos[i]) 
        return
    
    def set_goal(self, id: int, target_pos: int) -> None:
        PACKET_HANDLER.write4ByteTxRx(self._port_handler, id, ADDR_GOAL_POS, target_pos)
        return

    def _sync_torque(self) -> None:      # Set all torque to zero
        print("Resetting torque enable to 0...")
        for i in range(self.num_motors):
            PACKET_HANDLER.write1ByteTxRx(self._port_handler, i+1, ADDR_TORQUE_ENABLE, 0) 
            self.torque_status.append(0)
        print("Resetting complete.")
        return 
    
    """
    Limits based on dynamixel_consts.py - to edit in there
    """
    def _set_limits(self) -> None:
        # Write limits
        PACKET_HANDLER.write4ByteTxRx(self._port_handler, SHOULDER_AB_AD, ADDR_MAX_POS, MM_SHOULDER_ABDUCTION["max"])
        PACKET_HANDLER.write4ByteTxRx(self._port_handler, SHOULDER_AB_AD, ADDR_MIN_POS, MM_SHOULDER_ABDUCTION["min"])
        PACKET_HANDLER.write4ByteTxRx(self._port_handler, SHOULDER_F_E, ADDR_MAX_POS, MM_SHOULDER_FLEX_EX["max"])
        PACKET_HANDLER.write4ByteTxRx(self._port_handler, SHOULDER_F_E, ADDR_MIN_POS, MM_SHOULDER_FLEX_EX["min"])
        PACKET_HANDLER.write4ByteTxRx(self._port_handler, SHOULDER_ROT, ADDR_MAX_POS, MM_SHOULDER_ROT["max"])
        PACKET_HANDLER.write4ByteTxRx(self._port_handler, SHOULDER_ROT, ADDR_MIN_POS, MM_SHOULDER_ROT["min"])
        PACKET_HANDLER.write4ByteTxRx(self._port_handler, ELBOW, ADDR_MAX_POS, MM_ELBOW_FLEX_EX["max"])
        PACKET_HANDLER.write4ByteTxRx(self._port_handler, ELBOW, ADDR_MIN_POS, MM_ELBOW_FLEX_EX["min"])
        PACKET_HANDLER.write4ByteTxRx(self._port_handler, PRO_SUP, ADDR_MAX_POS, MM_PRO_SUP["max"])
        PACKET_HANDLER.write4ByteTxRx(self._port_handler, PRO_SUP, ADDR_MIN_POS, MM_PRO_SUP["min"])

    def torque_toggle(self, set: int=None, id: int=None) -> None:
        if id==None:
            if set==None:
                for i in range(self.num_motors):
                    PACKET_HANDLER.write1ByteTxRx(self._port_handler, i+1, ADDR_TORQUE_ENABLE, not self.torque_status[i])
                self.torque_status[i] = not self.torque_status[i]
            else:
                for i in range(self.num_motors):
                    PACKET_HANDLER.write1ByteTxRx(self._port_handler, i+1, ADDR_TORQUE_ENABLE, set)
                self.torque_status[i] = set
        else:
            if set==None:
                PACKET_HANDLER.write1ByteTxRx(self._port_handler, id, ADDR_TORQUE_ENABLE, not self.torque_status[id-1])
                self.torque_status[id-1] = not self.torque_status[id-1]
            else:
                PACKET_HANDLER.write1ByteTxRx(self._port_handler, id, ADDR_TORQUE_ENABLE, set)
                self.torque_status[id-1] = set
        return
            
    def _set_profile(self, vel: int=PROF_VEL, accel: int=PROF_ACC, id: int=None) -> None:
        if id==None:
            for i in range(self.num_motors):
                PACKET_HANDLER.write4ByteTxRx(self._port_handler, i+1, ADDR_VELOCITY_PROF, vel)
                PACKET_HANDLER.write4ByteTxRx(self._port_handler, i+1, ADDR_ACCELERATION_PROF, accel)
        else:
            PACKET_HANDLER.write4ByteTxRx(self._port_handler, id, ADDR_VELOCITY_PROF, vel)
            PACKET_HANDLER.write4ByteTxRx(self._port_handler, id, ADDR_ACCELERATION_PROF, accel)
        return

    # Read functions
    def get_current_pos(self, id: int=None) -> None:
        if id==None:
            for i in range(self.num_motors):
                self.pos[i] = PACKET_HANDLER.read4ByteTxRx(self._port_handler, i+1, ADDR_PRESENT_POS)
        else:
            self.pos[id-1] = PACKET_HANDLER.read4ByteTxRx(self._port_handler, id, ADDR_PRESENT_POS)
        return 
    
    def get_current_load(self, id: int=None) -> None:
        if id==None:
            for i in range(self.num_motors):
                self.load[i] = PACKET_HANDLER.read2ByteTxRx(self._port_handler, i+1, ADDR_PRESENT_CURRENT)
        else:
            self.load[id-1] = PACKET_HANDLER.read2ByteTxRx(self._port_handler, id, ADDR_PRESENT_CURRENT)
        return 
    
    ## Setup Functions
    def _set_id(self, id: int) -> None:
        print("Setting ID...")
        PACKET_HANDLER.write1ByteTxRx(self._port_handler, ID_BROADCAST, ADDR_ID, id)
        print(f'ID set to {id}')
        return
    
    def _read_id(self, id: int=ID_BROADCAST) -> int:
        id = PACKET_HANDLER.read1ByteTxRx(self._port_handler, id, address=ADDR_ID)
        print(f'ID of Motor is {id}')
        return id
    
    def _mx_set_id(self, id: int) -> None:
        print("Setting ID...")
        PACKET_HANDLER.write1ByteTxRx(self._port_handler, ID_BROADCAST, MX_ADDR_ID, id)
        print(f'ID set to {id}')
        return

    def _mx_read_id(self, id: int=ID_BROADCAST) -> int:
        id = PACKET_HANDLER.read1ByteTxRx(self._port_handler, id, MX_ADDR_ID)
        print(f'ID of Motor is {id}')
        return id
    
# Will need to change to something valid
TEST_POSITION = [2000, 2000, 2000, 2000, 2000]

# Test Angles
def main():
    arm = Motors(5)
    while True:
        id = input(f'Enter motor to check <1> - <{arm.num_motors}>, anything else to quit.\n')
        if id.isnumeric() and int(id) <= arm.num_motors and int(id) > 0:
            arm.torque_toggle(1, int(id))
            print(f'Checking Motor {id}')
        else:
            break
        while True:
            pos = input(f'Enter position to check <0> - <4095>, anything else to quit. \n')
            if pos.isnumeric() and int(pos) >= 0: #and int(pos) < 4096:
                arm.set_goal(int(id), int(pos))
            else:
                arm.torque_toggle(0, int(id))
                break
    arm.port_close()

def setup_arm():
    arm = Motors(5)
    # arm._set_id(PRO_SUP)
    arm._mx_set_id(SHOULDER_ROT)
    arm._read_id(SHOULDER_F_E)
    arm._read_id(SHOULDER_AB_AD)
    arm._read_id(SHOULDER_ROT)
    arm._read_id(ELBOW)
    arm._read_id(PRO_SUP)
    arm._mx_read_id(SHOULDER_F_E)
    arm._mx_read_id(SHOULDER_AB_AD)
    arm._mx_read_id(SHOULDER_ROT)
    arm._mx_read_id(ELBOW)
    arm._mx_read_id(PRO_SUP)
    arm.port_close()
    
if __name__ == "__main__":
    main()
    # setup_arm()