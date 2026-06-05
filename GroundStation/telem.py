# This file is for data that needs not persist between reloads. 
# This includes connection status, general boat telemetry data, 
# whether or not the boat is on manual control, and more
# Also includes a function for calculating the distance bewteen two points
import math
EARTH_RADIUS = 6371 # km

is_connected = False
dCount = 0


data = {
    'lat': 0,
    'lon': 0,
    'soc': 0,
    'connection': False,
    'dist': 0,
    'speed': 0,
    'acceleration': 0.0,
    'heading': 0,
    'tar_heading': 0,
    'tar_dist': 0,
    'l_thrust': 0,
    'r_thrust': 0,
    'roll': 0,
    'pitch': 0,
    'omega': 0,
    'winch_depth': 0,
    'timestamp': '2024-06-01 12:00:00',
    'rssi': 0,
    'pwr': False,
    'hold': False,
    'auto': False,
    'return': False,
    'auto_return': True,
    'error': 0x00
}


def dist_p2p(lat1, lon1, lat2, lon2):
    dLat = (lat2 - lat1)*math.pi/180
    dLon = (lon2 - lon1)*math.pi/180
    lat1 = lat1*math.pi/180
    lat2 = lat2*math.pi/180
    return 2*EARTH_RADIUS*1000*math.asin(math.sqrt(math.sin(dLat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dLon/2)**2))
