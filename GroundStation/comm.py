import struct
import telem


## BOOLEAN Values
BINARY_TRUE = 0xFF
BINARY_FALSE = 0x00

## Sending Header Formats
HEADER_GPS_WAYPOINTS = 3
HEADER_MOVE = 5
HEADER_WINCH = 7
HEADER_GAINS = 9
HEADER_MODE = 11
HEADER_TELEM = 4
HEADER_ERROR = 6
HEADER_ACK = 0x00
HEADER_ACTION = 13
HEADER_WINCH_FUNC = 15

## Unique
ID_FWD = 0x00
ID_BCK = 0x03
ID_LEFT = 0x0C
ID_RIGHT = 0x30
ID_STOP = 0xC0
ID_COAST = 0x81
ID_UP = 0x00
ID_DN = 0xFF

ID_K = 0x99
ID_REVERT = 0x00
ID_OVERWRITE = 0xFF

## ACTION Headers
ID_MAG_CHANGE = 0x02
LORA_TX = 0x04
THRUST_IDLE = 0x06
ID_RST = 0x12
ID_MIN_DIST = 0x14
ID_AUTO_RETURN = 0x16


# No Magic Numbers
GPS_MULT = 1_000_000 # 6 decimal place for GPS coords
TELEM_MULT = 10 # 1 decimal place for telemetry data


def boolB(bool):
    if (bool):
        return BINARY_TRUE
    return BINARY_FALSE

def Bbool(bool):
    if (bool > 64):
        return True
    return False

def format_command(cmd_name, *args):
    cmd_map = {
        "gps_waypoints": pack_GPS_waypoints,
        "move": pack_move_command,
        "winch": pack_winch_command,
        "gain": pack_gain_command,
        "mode": pack_mode_command,
        "error": pack_error,
        "winch_auto": pack_winch_function,
        "action": pack_action_cmd
    }
    if cmd_name not in cmd_map:
        print(f"Error: {cmd_name} is not valid")
        return
    packet = cmd_map[cmd_name](*args)
    if packet == -1 or packet is None:
        print(f"Failed to pack command: {cmd_name} with args {args}")
        return -1
    print(f"Packed: {cmd_name} with args {args} into {packet.hex(' ')}")
    return packet
    
def unpack_command(msg):
    parsed = None
    if (len(msg) > 1):
        header = msg[0]
        data = msg[1:]
        cmd_map = {
            HEADER_GAINS: unpack_gain,
            HEADER_MODE: unpack_mode_command,
            HEADER_TELEM: unpack_telem,
        }
        if header not in cmd_map:
            return -1
        parsed = cmd_map[header](data)
    else:
        if struct.unpack('<B', msg)[0] != 0:
            return -1
        print("ACK")
        parsed = 0
    telem.data['connection'] = True
    return parsed

def pack_move_command(direction, power):
    identifier = None
    if (direction == 'forward'):
        identifier = ID_FWD
    elif (direction == 'backward'):
        identifier = ID_BCK
    elif (direction == 'left'):
        identifier = ID_LEFT
    elif (direction == 'right'):
        identifier = ID_RIGHT
    elif (direction == 'coast'):
        identifier = ID_COAST
    elif (direction == 'stop'):
        identifier = ID_STOP
    
    if (identifier is not None):
        parity = identifier ^ 0xFF
        return struct.pack('<BBBB', HEADER_MOVE, identifier, int(power), parity)
    return -1

def pack_winch_command(direction, speed):
    identifier = None
    if (direction == 'up'):
        identifier = ID_UP
    elif (direction == 'down'):
        identifier = ID_DN
    elif (direction == 'stop'):
        identifier = ID_STOP
    if (identifier is not None):
        parity = identifier ^ 0xFF
        return struct.pack('<BBBB', HEADER_WINCH, identifier, speed, parity)
    return -1

def pack_winch_function(data):
    if len(data) == 3:
        A = int(data[0]*100)
        T = int(data[1]*10)
        O = int(data[2]*100)
        return struct.pack('<BhHh', HEADER_WINCH_FUNC, A, T, O)
    return -1

def pack_gain_command(data):
    head_gains = data[0].copy()
    pos_gains = data[1].copy()
    for key in head_gains:
        head_gains[key] = int(head_gains[key]*1000)
        pos_gains[key] = int(pos_gains[key]*1000)
    Kp = head_gains['Kp']    
    print(HEADER_GAINS)
    print(*head_gains.values())
    print(*pos_gains.values())
    return struct.pack('<B8I', HEADER_GAINS, *head_gains.values(), *pos_gains.values())
    
def unpack_gain( data):
        h1, h2, h3, h4, p1, p2, p3, p4 = struct.unpack('<8I', data)
        gains_head = (h1, h2, h3, h4)
        gains_pos = (p1, p2, p3, p4)
        return (gains_head, gains_pos)
    

def pack_mode_command(pwrBool, holdBool, autoBool):
    if (pwrBool == False):
        holdBool = False
        autoBool = False
    bool1 = boolB(pwrBool)
    bool2 = boolB(holdBool)
    bool3 = boolB(autoBool)
    fmt = '<BBBB'        
    return struct.pack(fmt, HEADER_MODE, bool1, bool2, bool3)

def unpack_mode_command(data):
    pwrMode, holdMode, autoMode = struct.unpack('<BBB', data)
    pwrMode = Bbool(pwrMode)
    holdMode = Bbool(holdMode)
    autoMode = Bbool(autoMode)
    return [pwrMode, holdMode, autoMode]

# Max Waypoints accepted: 30
def pack_GPS_waypoints(coord_dict):
    if (len(coord_dict)*4+1 > 255):
        return -1
    msg_format = '<B'
    payload = [HEADER_GPS_WAYPOINTS]
    for i in range(0, len(coord_dict)):
        lat = int(round(coord_dict[i]['lat'], 6)*GPS_MULT)
        lon = int(round(coord_dict[i]['lon'], 6)*GPS_MULT)
        msg_format += 'ii'
        payload.append(lat)
        payload.append(lon)
    return struct.pack(msg_format, *payload)

def unpack_telem(data):
    lat, lon, head, tar_head, tar_dist, soc, speed, accel, l_thrust, r_thrust, depth, roll, pitch, omega, pwr, hold, auto, Return, auto_return, error, ack = struct.unpack('<iiHHHBHhbbBhhhBBBBBBB', data)
    telem.data['lat'] = lat / GPS_MULT
    telem.data['lon'] = lon / GPS_MULT
    telem.data['heading'] = head
    telem.data['tar_heading'] = tar_head
    telem.data['tar_dist'] = tar_dist
    telem.data['soc'] = soc
    telem.data['speed'] = speed / TELEM_MULT
    telem.data['acceleration'] = accel / TELEM_MULT
    telem.data['l_thrust'] = l_thrust
    telem.data['r_thrust'] = r_thrust
    telem.data['winch_depth'] = depth / 10
    telem.data['roll'] = roll / TELEM_MULT
    telem.data['pitch'] = pitch / TELEM_MULT
    telem.data['omega'] = omega / TELEM_MULT
    telem.data['pwr'] = Bbool(pwr)
    telem.data['hold'] = Bbool(hold)
    telem.data['auto'] = Bbool(auto)
    telem.data['return'] = Bbool(Return)
    telem.data['auto_return'] = Bbool(auto_return)
    telem.data['error-code'] = error
    telem.data['ack'] = Bbool(ack)

def pack_error(code):
    return struct.pack('<BBB', boolB(False), HEADER_ERROR, code)

def unpack_error(data):
    return struct.unpack('<B', data)

def pack_action_cmd(act_type, values):
    fmt = '<BB4H'
    if values is None or len(values) != 4:
        return -1
    for i in range(len(values)):
        values[i] = int(values[i])
    type_map = {
        'hold_radius': ID_MIN_DIST,
        'auto_return': ID_AUTO_RETURN
    }
    if act_type not in type_map:
        return -1
    id = type_map[act_type]
    if id == ID_AUTO_RETURN:
        values[0] = boolB(values[0])
    return struct.pack(fmt, HEADER_ACTION, id, *values)
