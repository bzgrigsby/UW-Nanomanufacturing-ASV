import csv
from datetime import datetime
from flask import Flask, render_template, send_file, abort
from flask_socketio import SocketIO
from typing import cast
import telem
import comm
import lora_setup as lora
from intercomm import Intercomm
import json
import os
import time
import queue
import struct
import subprocess
import sqlite3
import io
# from PathFindingAlgorithum.src import Path as path
# from PathFindingAlgorithum.src import TestEnv


class CSVLogger:
    def __init__(self, filename="boat_telemetry_log.csv"):
        self.filename = filename
        self.start_time = time.perf_counter()
        self.headers = [
            "timestamp", "seconds", "lat", "lng", "depth", "heading", "speed", "roll", "pitch"
        ]
        self._initialize_csv()

    def _initialize_csv(self):
        with open(self.filename, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(self.headers)
    def log_row(self, data):
        try:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            row = [
                timestamp,
                round(time.perf_counter() - self.start_time, 3),
                data.get('lat', 0.0),
                data.get('lon', 0.0),
                data.get('winch_depth', 0.0),
                data.get('heading', 0.0),
                data.get('speed', 0.0),
                data.get('roll', 0.0),
                data.get('pitch', 0.0),
                data.get('rssi', 0.0)
            ] 
            with open(self.filename, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(row)
        except Exception as e:
            print("Error logging row:", e)
        

class AppConfig:
    def __init__(self, filename="config.json"):
        self.filename = filename
        self.data = {
            "dark_mode": True,
            "man_nav": True,
            "waypoints": [],
            "backup-waypoints": [],
            "pwr_mode": False,
            "auto_mode": False,
            "hold_mode": False,
            "head_gains": {'Kp': 10.0, 'Ki': 0.05, 'Kd': 0.3, 'N': 50.0},
            "head_gains_og": {'Kp': 10.0, 'Ki': 0.05, 'Kd': 0.3, 'N': 50.0},
            "pos_gains": {'Kp': 5.0, 'Ki': 0.05, 'Kd': 1.0, 'N': 30.0},
            "pos_gains_og": {'Kp': 5.0, 'Ki': 0.05, 'Kd': 1.0, 'N': 30.0},
            "hold-radius": 400, # mm
            "base_lat": 47.653167,
            "base_lon": -122.299607,
            "max_winch_depth": 5,
            "radius": 500,
            "boat_lat": 47.648972,
            "boat_lon":  -122.297246,
            "pwrLim": 30,
            
        }
        self.load()
    def load(self):
        if os.path.exists(self.filename):
            with open(self.filename, 'r') as f:
                self.data.update(json.load(f))
                print("Configuration Loaded")

    def save(self):
        with open(self.filename, 'w') as f:
            json.dump(self.data, f, indent=4)
    
    def set(self, key, value):
        self.data[key] = value
        self.save()
        return value

    def get(self, key):
        return self.data.get(key)
    
    def clearWaypoints(self):
        self.data['waypoints'] = []
        self.save()
    
    def addWaypoint(self, waypoint):
        print(waypoint)
        self.data['waypoints'].append(waypoint)
        self.save()

    def addWaypoints(self, waypoints):
        self.clearWaypoints()
        self.data['waypoints'] = waypoints
        self.data['backup_waypoints'] = waypoints   
        self.save()


app = Flask(__name__)
socketio = SocketIO(app)
config = AppConfig()
inter = Intercomm()
LoRa = lora.LoRa_rfm9x(inter)
csv_logger = CSVLogger()

Q = queue.Queue() 
T_Q = []
IMPT = queue.Queue()
Q_MAX_SIZE = 6

last_time = time.perf_counter()

@app.route('/')
def main():
    return render_template('main.html')

@app.route('/download-csv')
def download_csv():
    try:
        csv_filepath = 'boat_telemetry_log.csv'
        return send_file(
            csv_filepath,
            mimetype='text/csv',
            as_attachment=True,
            download_name='boat_telemetry_log.csv'
            )
    except Exception as e:
        return "File Download Failed:", 404

TILE_DB = [
    "tiles/everett_estuary_high_res.mbtiles",
    "tiles/UW_and_Seattle.mbtiles",
    "tiles/Washington_MaxZoom_14.mbtiles"
]
def get_tile_from_db(db_path, z, x, y_tms):
    if not os.path.exists(db_path):
        return None
    try:
        db_conn = sqlite3.connect(db_path)
        cursor = db_conn.cursor()
        cursor.execute("SELECT tile_data FROM tiles WHERE zoom_level = ? AND tile_column = ? AND tile_row = ?", 
                        (z, x, y_tms))
        row = cursor.fetchone()
        db_conn.close()
        if row and row[0]:
            return row[0]
    except sqlite3.Error as e:
        print(f"Database error reading {db_path}: {e}")
        return None

@app.route('/map-tiles/<int:z>/<int:x>/<int:y>')
def get_tile(z, x, y):
    y_tms = (1 << z) - 1 - y # Convert Leaflet Y to TMS Y axis standard
    for db_path in TILE_DB: # Loop through databases in priority order
        tile_blob = get_tile_from_db(db_path, z, x, y_tms)
        if tile_blob:
            return send_file(io.BytesIO(tile_blob), mimetype='image/png')
    abort(404)

## ================================================================ ##
## ================= Webserver Recieving ONLY ===================== ##
@socketio.on('connect')
def send_InitData():
    addToQ(struct.pack('<B', 0))
    socketio.emit('load_config', config.data)

@socketio.on('clear_waypoints')
def handle_clear_waypoints():
    config.clearWaypoints()
    print("Waypoints cleared.")

@socketio.on('compute-path')
def handle_Path_compute(data):   
    print(data)
    start_lat = float(data['start_lat'])
    start_lng = float(data['start_lng'])
    end_lng = float(data['end_lat'])
    end_lat = float(data['end_lng'])
    # des_depth = config.get("max_winch_depth")
    # print(des_depth)
    # waypoints = None

    # if waypoints is not None:
    #     print(waypoints)
    #     for coord in waypoints:
    #         lat_i = coord[0]
    #         lon_i = coord[1]
    #         point = {'lat': lat_i, 'lon': lon_i} 
    #         config.addWaypoint(point)
    socketio.emit('GPS_waypoint_update', config.get("waypoints"))

        
    # waypoints = path.get_path(start_lat, start_lng, end_lat, end_lng, des_depth, 1)
    # if (waypoints is not None):
    #     print(waypoints)
    # else:
    #     print(waypoints)


@socketio.on('set-base-coords')
def handle_Base_coords(data):
    config.set('base_lat', data['lat'])
    config.set('base_lon', data['lon'])

@socketio.on('set-max-winch-depth')
def handle_Winch_Depth(data):
    config.set('max_winch_depth', data['winch'])

@socketio.on('restore-gains')
def handle_revert_gains():
    socketio.emit('gain_update', {
        'heading': config.get("head_gains_og"),
        'position': config.get("pos_gains_og")
    })

@socketio.on('settings-change')
def handle_settings_change(data):
    config.set('dark_mode', data['dark_mode'])
    
@socketio.on('restore-waypoints')
def handle_restore_waypoints():
    socketio.emit('GPS_waypoint_update', config.get('backup_waypoints'))

@socketio.on('baud-rate')
def handle_baud_rate_change(data):
    config.set('baud_rate', data)

@socketio.on('req_GPS_waypoints')
def handle_GPS_request():
    socketio.emit('GPS_waypoint_update', config.get("waypoints"))


@socketio.on('restart-webserver')
def handle_webserver_restart():
    try:
        script_path = os.path.expanduser('~/bounce.sh')
        subprocess.Popen(['/bin/bash', script_path], 
                         stdout=subprocess.DEVNULL, 
                         stderr=subprocess.DEVNULL)
    except Exception as e:
        print("failure to restart:", e)

## ================================================================ ##
## =================== Boat Communication ========================= ##
move_seq_id = 0
@socketio.on('drive_command')
def handle_drive_command(data):
    global move_seq_id
    direction = data['dir']
    pwr = data['pwr']
    move_seq_id += 1
    curr_id = move_seq_id
    packet = comm.format_command('move', direction, pwr)
    if packet == None or packet == -1:
        return
    addToQ(packet)

@socketio.on('winch_command')
def handle_winch_command(data):
    dir = data['direction']
    speed = data['speed']
    packet = comm.format_command('winch', dir,  speed)
    addToQ(packet, False, True)

@socketio.on("control_mode")
def handle_control_law(data):
    if inter.ack_pending:
        return
    config.set("auto_mode", data.get('auto', False))
    config.set("pwr_mode", data.get('pwr', False))
    config.set("hold_mode", data.get('hold', False))
    packet = comm.format_command('mode', config.get('pwr_mode'), config.get('hold_mode'), config.get('auto_mode'))
    addToQ(packet, True)
    
@socketio.on('receive_GPS_coords')
def handle_receive_GPS_coords(data):
    if inter.ack_pending:
        return
    config.addWaypoints(data['waypoints'])
    if config is not None:
        packet = comm.format_command("gps_waypoints", config.get('waypoints'))
        addToQ(packet, True)

@socketio.on('set-gains')
def handle_PID_command(data):
    if inter.ack_pending:
        return
    print(data)
    head_gains = config.set('head_gains', data['heading'])
    pos_gains = config.set('pos_gains', data['position'])
    packet = comm.format_command('gain', (head_gains, pos_gains))
    addToQ(packet, True)    

@socketio.on('set-winch-auto')
def handle_winch_auto(data):
    if inter.ack_pending:
        return
    print(data)
    packet = comm.format_command('winch_auto', data)
    addToQ(packet, True)

@socketio.on('set-hold-radius')
def handle_hold_radius(data):
    if inter.ack_pending:
        return
    print(data)
    packet = comm.format_command('action', 'hold_radius', [data/1000.0, 0, 0, 0])
    addToQ(packet, True)

@socketio.on('toggle-auto-return')
def toggle_auto_return(data):
    if inter.ack_pending:
        return
    print("Toggling Auto Return:", data)
    bit = 1 if data else 0
    packet = comm.format_command('action', 'auto_return', [bit, 0, 0, 0])
    addToQ(packet, True)

def get_time_ms():
    return time.perf_counter()*1000

# Override ONLY for coast commands
def addToQ(packet, important=False, override=False):
    if Q.qsize() < Q_MAX_SIZE or (override and Q.qsize() < Q_MAX_SIZE+1):
        Q.put(packet)
        if important:
            inter.set_pend_ack()
        IMPT.put(important)
    else:
        socketio.emit('cmd-overload')
        print("<===Q FULL===>")
    
last_receive = 0
last_transmission = time.perf_counter()*1000 # time of last transmission in ms
# Transmit any items stuck in the queue. If no transmission have occured in the past 5000ms (5s), then send an acknowledgement packet.
# If the boat does not receive any transmissions for 6500ms (6.5s) then it will go into the return state.
def transmit():
    global last_transmission, last_receive
    while True:
        curr_time = time.perf_counter()
        if Q.qsize() > 0:
            msg = Q.get()
            importance = IMPT.get()
            for _ in range(2):
                LoRa.transmit(msg, importance)
                socketio.sleep(0.13)
            last_transmission = curr_time
        # This transmits an "I'm still connected!" message after 5s of no quened cmds
        elif(curr_time >= last_transmission +  3 and curr_time >= last_receive + 0.5):
            print(" * ")
            LoRa.transmit(struct.pack('<B', 0), False)
            last_transmission = curr_time
        else:
            socketio.sleep(cast(int,0.01))
socketio.start_background_task(transmit)
    

## ================================================================ ##
## ==================== Webserver Sending ========================= ##
def background_receive():
    global last_receive
    packet = None
    while True:
        if time.perf_counter() - last_receive > 10:
            telem.data['connection'] = False
        if inter.ack_pend_time() > 4.0:
            socketio.emit('ack-failure')
            inter.ack_pending = False
        if (Q.qsize() == 0):
            packet = LoRa.rfm9x.receive(timeout=0.3)
            if packet is not None:
                last_receive = time.perf_counter()
                print("Packet:", packet.hex(' '), "len=", len(packet), "RSSI=", LoRa.rfm9x.last_rssi, "dBm")
                msg = comm.unpack_command(packet)
                if msg == -1:
                    print("Received invalid command")
                    continue
                else: 
                    telem.data['dist'] = telem.dist_p2p(config.get('base_lat'), config.get('base_lon'), 
                                                        telem.data['lat'], telem.data['lon'])  
                    telem.data['rssi'] = LoRa.rfm9x.last_rssi 
                    if inter.ack_pending and telem.data['ack'] == True:
                        socketio.emit('ack-received')
                        inter.ack_pending = False
                        telem.data['ack'] = False
                csv_logger.log_row(telem.data)
            socketio.emit('telemetry_update', telem.data)
            socketio.sleep(cast(int,0.01))
socketio.start_background_task(background_receive)
