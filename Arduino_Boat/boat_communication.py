from __future__ import annotations
from typing import TYPE_CHECKING
from arduino.app_utils import *
import struct
import time
from nav import Nav_Info
from definitions import FSM
import definitions as DEF
from definitions import ERRS
# Below imports have race conditions
if TYPE_CHECKING:
    from boat_state import BoatVars

## BOOLEAN Values
BINARY_TRUE = 0xFF
BINARY_FALSE = 0x00

## Sending Header Formats
HEADER_ACK = 0
HEADER_GPS_WAYPOINTS = 3
HEADER_MOVE = 5
HEADER_WINCH = 7
HEADER_GAINS = 9
HEADER_MODE = 11
HEADER_TELEM = 4
HEADER_ERROR = 6
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

## Action IDs
ID_RST = 0x12
ID_MIN_DIST = 0x14
ID_MAG_CHANGE = 0x02
LORA_TX = 0x04
THRUST_IDLE = 0x06
ID_AUTO_RETURN = 0x16

# No Magic Numbers
GPS_MULT = 1_000_000 # 6 decimal place for GPS coords
TELEM_MULT = 10 # 1 decimal place for telemetry data

class Parser:
    def __init__(self, nav : Nav_Info, params : BoatParams, boat_state: BoatVars):
        self.params = params
        self.nav = nav
        self.boat_state = boat_state
        self.ack_pending = False

    def boolB(self, bool):
        if (bool):
            return BINARY_TRUE
        return BINARY_FALSE

    def Bbool(self, bool):
        if (bool > 64):
            return True
        return False

    def format_command(self, cmd_name, *args):
        cmd_map = {
            "mode": self.pack_mode_command,
            "telem": self.pack_telem,
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
        
    def unpack_command(self, msg):
        parsed = None
        if (len(msg) > 1):
            header = msg[0]
            data = msg[1:]
            cmd_map = {
                HEADER_GPS_WAYPOINTS: self.unpack_GPS,
                HEADER_MOVE: self.unpack_move,
                HEADER_WINCH: self.unpack_winch_command,
                HEADER_MODE: self.unpack_mode_command,
                HEADER_ACTION:self.unpack_action,
                HEADER_GAINS:self.unpack_gain,
                HEADER_WINCH_FUNC: self.unpack_winch_func_cmd
            }
            if header not in cmd_map:
                return -1
            parsed = cmd_map[header](data)            
        else:
            if struct.unpack('<B', msg)[0] != 0:
                return -1
            parsed = 0
        self.params.connection = True
        return parsed

    def unpack_move(self, data):
        id, pwr, parity = struct.unpack('<BBB', data)
        if pwr == 0 or pwr is None:
            pwr = 10
        dir = -1
        if parity == (id ^ 0xFF):
            if (id == ID_FWD):
                dir = 1
            elif (id == ID_BCK):
                dir = 2
            elif (id == ID_LEFT):
                dir = 3
            elif (id == ID_RIGHT):
                dir = 4
            elif (id == ID_STOP):
                dir = 0
        if parity != (id ^ 0xFF):
            return -1
        if (id != ID_COAST):
            self.boat_state.state = FSM.MANUAL_CNTRL
            print("MANUAL CNTRL")
        self.boat_state.update_dir(dir)
        self.boat_state.update_pwr(pwr)
        
    
    # Move the winch given a single up or down command from the webserver.
    # If the power is off, the winch is inhibited from moving.
    def unpack_winch_command(self, data):  
        id, speed, parity = struct.unpack('<BBB', data)
        if parity != (id ^ 0xFF):
            return -1
        if self.params.pwr == False:
            return
        if (id == ID_UP):
            DEF.BridgeCall("move_winch", 1, notify=True)
            return 'up'
        elif (id == ID_DN):
            DEF.BridgeCall("move_winch", 2, notify=True)
            return 'down'
        elif (id == ID_STOP):
            DEF.BridgeCall("move_winch", 3, notify=True)
            return 'stop'
        
    def unpack_winch_func_cmd(self, data):
        print("Activating Winch Auto")
        A, T, O = struct.unpack('<hHh', data)
        A = A/100.0
        T = T/10.0
        O = O/100.0
        DEF.BridgeCall("start_winch_auto", A, T, O, notify=True)
        
    # May want to come back one day and add some form of max positional discrepancy adjustment
    def unpack_gain(self, data):
        self.ack_pending = True
        print("Unpacking gains...")
        h1, h2, h3, h4, p1, p2, p3, p4 = struct.unpack('<8I', data)
        gains_head = [h1/1000.0, h2/1000.0, h3/1000.0, h4/1000.0]
        gains_pos = [p1/1000.0, p2/1000.0, p3/1000.0, p4/1000.0]
        self.boat_state.update_gains(gains_head, gains_pos)
        return [h1, h2, h3, h4, p1, p2, p3, p4]

    def pack_mode_command(self):
        bool1 = self.boolB(self.params.pwr)
        bool2 = self.boolB(self.params.hold)
        bool3 = self.boolB(self.params.auto)
        fmt = '<BBBB'        
        return struct.pack(fmt, HEADER_MODE, bool1, bool2, bool3)

    def unpack_mode_command(self, data):
        pwrMode, holdMode, autoMode = struct.unpack('<BBB', data)
        self.params.pwr = self.Bbool(pwrMode)
        self.params.hold = self.Bbool(holdMode)
        self.params.auto = self.Bbool(autoMode)
        self.ack_pending = True
        if (self.params.pwr == False):
            self.boat_state.state = FSM.MOTORS_OFF
        elif (self.params.pwr == True):
            DEF.BridgeCall("set_pwr", True)
            if self.params.hold == True:
                self.boat_state.state = FSM.HOLD_POS
            elif self.params.auto == True:
                self.boat_state.state = FSM.AUTO_NAV
            elif self.boat_state.state not in [FSM.MANUAL_CNTRL, FSM.RETURN]:
                self.boat_state.state = FSM.IDLE

    def unpack_GPS(self, data):
        self.ack_pending = True
        print("GPS Data Length:", len(data))
        if len(data) % 8 != 0 or len(data) == 0:
            self.nav.clr()
            self.nav.hasTarget = False
            return -1
        num_waypoints = int(len(data)/8)
        print("  Number of waypoints:", num_waypoints)
        if num_waypoints == 0:
            self.nav.clr()
        lat = []
        lon = []
        fmt = '<' + 'ii' * num_waypoints
        coords = struct.unpack(fmt, data)
        self.nav.clr()
        for i in range(0, len(coords), 2):
            lat_i = coords[i]/GPS_MULT
            lon_i = coords[i+1]/GPS_MULT
            lat.append(lat_i)
            lon.append(lon_i)
            if (i == 0):
                self.nav.update_target(lat_i, lon_i)
            else:
                self.nav.add_waypoint((lat_i, lon_i))
        return lat, lon

    def unpack_action(self, data):
        self.ack_pending = True
        header, val1, val2, val3, val4 = struct.unpack('<B4H', data)
        if header == ID_MIN_DIST:
            print("Changing Min Dist", DEF.MIN_DIST, "->", val1)
            DEF.MIN_DIST = val1
            return 1
        elif header == ID_RST:
            self.boat_state.reset_pids
            return 1
        elif header == ID_AUTO_RETURN:
            self.boat_state.auto_return = self.Bbool(val1)
            return 1
        return -1

    def pack_telem(self):
        lat = int(round(self.params.lat, 6)*GPS_MULT)
        lon = int(round(self.params.lng, 6)*GPS_MULT)
        head = abs(int(self.params.heading))
        soc = abs(int(self.params.soc))
        speed = abs(int(round(self.params.speed, 1)*TELEM_MULT))
        accel = int(round(self.params.accel_x, 1)*TELEM_MULT)
        l_thrust = int(self.params.l_thrust*100)
        r_thrust = int(self.params.r_thrust*100)
        depth = abs(int(self.params.winch_depth*10))
        if depth > 255:
            depth = 0
        roll = int(round(self.params.roll, 1)*TELEM_MULT)
        pitch = int(round(self.params.pitch, 1)*TELEM_MULT)
        omega = int(round(self.params.omega, 1)*TELEM_MULT)
        tar_heading = abs(int(self.nav.tar_heading))
        tar_dist = abs(int(self.nav.calc_dist()))

        pwr = self.boolB(self.params.pwr)
        hold = self.boolB(self.params.hold)
        auto = self.boolB(self.params.auto)
        Return = self.boolB(self.params.Return)
        auto_return = self.boolB(self.boat_state.auto_return)
        fmt = '<BiiHHHBHhbbBhhhBBBBBBB' # 1+1+4+4+2+2+2+1+2+2+1+1+1+2+2+2+1+1+1+1+1+1 = 36 bytes per packet (13.7% of total)
        return struct.pack(fmt, HEADER_TELEM, lat, lon, head, tar_heading, tar_dist, soc, speed, accel, l_thrust, r_thrust, depth, roll, pitch, omega, pwr, hold, auto, Return, auto_return, self.params.code, self.boolB(self.ack_pending))


class BoatParams:
    def __init__(self):
        self.lat = 0.0
        self.lng = 0.0
        self.speed = 0.0
        self.heading = 0.0

        self.soc = 0.0
        self.accel_x = 0.0
        self.accel_y = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.omega = 0.0
        self.l_thrust = 0.0
        self.r_thrust = 0.0

        self.winch_depth = 0.0
        self.pwr = False
        self.hold = False
        self.auto = False
        self.Return = False

        self.connection = False
        self.code = 0x00

    def update_telem(self, vals):
        if vals is None: return
        self.soc = vals[0]
        self.accel_x = vals[1]*9.81
        self.accel_y = vals[2]*9.81
        self.roll = vals[3]
        self.pitch = vals[4]
        self.omega = vals[5]
        self.winch_depth = vals[6]

    def set_err(self, err_val):
        if err_val >= self.code:
            self.code = err_val

    # need to have error hierarchy
    def clr_error(self, clr_val):
        if self.code == clr_val:
            self.code = ERRS.NO_ERR

class LoRa:
    def __init__(self, parser : Parser, params : BoatParams):
        self.parser = parser
        self.last_receive = time.perf_counter()*1000
        self.last_transmission = time.perf_counter()*1000
        self.params = params
        self.transmission_interval = 1700 # in ms
        
    def receive(self):
        packet = DEF.BridgeCall("req_msg")
        if packet is None: return
        pkt_size = packet[0] # get total packet size
        last_rssi = packet[1]# get rssi
        pkt_content = packet[6 : 2+pkt_size] # ignore pkt size and radio head bytes
        print('-> RECEIVE:', pkt_content.hex(' '), 'RSSI =', last_rssi, 'dBm', 'Size =', pkt_size)
        msg = self.parser.unpack_command(pkt_content) 
        if msg != -1:
            self.parser.params.connection = True
            self.last_receive = time.perf_counter()*1000
        self.params.clr_error(ERRS.BRIDGE_ERR)

    def transmit(self):
        packet = self.parser.pack_telem()
        self.last_transmission = time.perf_counter()*1000
        if (packet is not None and packet != -1):
            result = DEF.BridgeCall("transmit_msg", packet, notify=True)
            if result is None: return
            # print("===============SENDING================")
            # print("Packet:", packet.hex(' '))
            if self.parser.ack_pending == True:
                print("ACK Sent!")
                self.parser.ack_pending = False       