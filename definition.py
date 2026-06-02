from enum import Enum, auto
import time

MAX_POS_THRUST = 12.0 # Newtons
MAX_NEG_THRUST = -12.0 # Newtons
NEUTRAL = 1498 # us
MIN_DIST = 1.5 # m

# Ang1 - Target Angle
# Ang2 - Current Angle
def ang_diff(ang1, ang2):
    return (ang1 - ang2+180) % 360 - 180

# Auto assigns each state to a sequential number for easy modification
class FSM(Enum):
    MOTORS_OFF = auto()
    MANUAL_CNTRL = auto()
    AUTO_NAV = auto()
    HOLD_POS = auto()
    RETURN = auto()
    IDLE = auto()


class ERRS():
    NO_GPS = 0x010
    NO_TARGET = 0x09
    INVALID_CMD = 0x08
    BRIDGE_ERR = 0x07
    NO_ERR = 0x00

def get_time_ms():
    return time.perf_counter()*1000
