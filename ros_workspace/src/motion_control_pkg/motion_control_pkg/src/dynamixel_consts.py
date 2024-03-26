# Control table addresses for Dynamixel motors MX and XM
ADDR_MODEL_NUM = 0
# Model numbers
MX_64_1 = 310
MX_64_2 = 311
XM_540 = 1130
XH_540 = 1110
XW_540 = 1180

ADDR_TORQUE_ENABLE = 64     # <0> off <1> on
ADDR_GOAL_POS = 116         # Position 0-4095 * 0.088 deg     
ADDR_VELOCITY_PROF = 112
ADDR_ACCELERATION_PROF = 108
ADDR_MIN_POS = 52
ADDR_MAX_POS = 48
## Reads
ADDR_PRESENT_POS = 132
ADDR_PRESENT_CURRENT = 126
ADDR_MOVING = 122

CONTROL_METHOD = 0     # <0> Velocity Control <1> Time Control

# Protocol version
PROTOCOL_VERSION = 2.0

# TODO refine and implement
# Joint constraints for Dynamixels 0-4095
SHOULDER_FLEX_EX = {    "min": 10,
                        "max": 10}
SHOULDER_ABDUCTION = {  "min": 10,
                        "max": 10}
SHOULDER_ROT = {    "min": 10,
                    "max": 10}
ELBOW_FLEX_EX = {   "min": 10,
                    "max": 10}
PRO_SUP = { "min": 10,
            "max": 10}
