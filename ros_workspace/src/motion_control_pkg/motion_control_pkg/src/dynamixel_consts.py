# Control table addresses for Dynamixel motors MX and XM
ADDR_MODEL_NUM = 0
# Model numbers
MX_64_1 = 310
MX_64_2 = 311
XM_540 = 1130
XH_540 = 1110
XW_540 = 1180

ID_BROADCAST = 254

ADDR_TORQUE_ENABLE = 64     # <0> off <1> on
ADDR_GOAL_POS = 116         # Position 0-4095 * 0.088 deg     
ADDR_VELOCITY_PROF = 112
ADDR_ACCELERATION_PROF = 108
ADDR_MIN_POS = 52
ADDR_MAX_POS = 48
## Reads
ADDR_ID = 7
ADDR_PRESENT_POS = 132
ADDR_PRESENT_CURRENT = 126
ADDR_MOVING = 122

CONTROL_METHOD = 0     # <0> Velocity Control <1> Time Control

# MX64 Prot 1
MX_ADDR_ID = 3
MX_ADDR_GOAL_POS = 30
MX_TORQUE_ENABLE = 24
MX_GOAL_ACC = 73
## Reads
MX_ADDR_PRESENT_POS = 36
MX_ADDR_PRESENT_LOAD = 40
MX_ADDR_MOVING = 46

# Protocol version
PROTOCOL_VERSION = 2.0

"""
Assign motor id to movement
"""
SHOULDER_F_E = 1
SHOULDER_AB_AD = 2
SHOULDER_ROT = 3
ELBOW = 4
PRO_SUP = 5

# TODO refine and implement
# Joint constraints for Dynamixels 0-4095
MM_SHOULDER_FLEX_EX = {    "min": 980,
                        "max": 2600,
                        "zero": 1200}
MM_SHOULDER_ABDUCTION = {  "min": 0,
                        "max": 1780,
                        "zero": 0}
MM_SHOULDER_ROT = {    "min": 0,
                    "max": 2000,
                    "zero": 0}
MM_ELBOW_FLEX_EX = {   "min": 0,
                    "max": 2000,
                    "zero": 1023}
MM_PRO_SUP = { "min": 0,
            "max": 4095,
            "zero": 2000}

ANGLE_TO_DYNA = 4095/360    # Converting from angle to dynamixel
