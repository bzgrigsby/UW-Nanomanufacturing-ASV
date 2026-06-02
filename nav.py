from __future__ import annotations
from typing import TYPE_CHECKING
from arduino.app_utils import *
import math
from definitions import ERRS
from definitions import ang_diff
import definitions as DEF
from typing import Optional
from math import sin, cos, pi
from collections import deque
from haversine import haversine, Unit
import time


# Note for magnetometer: True North and Magnetic North are not the same. Need a reference Declination  to lookup discrepancy and adjust on the fly given boat's current location.
KNOTS_TO_MS = 1/1.94384

if TYPE_CHECKING:
    from boat_communication import BoatParams

class Nav_Info:
    def __init__(self, params : BoatParams):
        self.lat = 0.0          # decimal deg
        self.lng = 0.0          # decimal deg
        self.speed = 0.0        # m/s
        self.heading = 0.0      # deg
        self.gps_head = 0.0     # deg
        self.tar_lat = 0.0      # deg
        self.tar_lng = 0.0      # deg

        self.hasGPS = False
        self.hasTarget = False

        self.lat_filter = None
        self.lng_filter = None
        self.last_update = time.perf_counter()
        
        self.tar_heading = 0.0  # deg
        self.origin_dist = 0.0  # m
        self.head_err = 0.0     # deg
        self.wayQue = deque()
        self.reverseQue = deque()
        self.waypoint_reached = False
        self.tar_waypoint = None
        self.params = params
        
    def calc_des_heading(self):
        if self.hasGPS and self.hasTarget:
            phi1 = self.lat*pi/180
            lam1 = self.lng*pi/180
            phi2 = self.tar_lat*pi/180
            lam2 = self.tar_lng*pi/180
            delta_lam = lam2 - lam1
            y = sin(delta_lam)*cos(phi2)
            x = cos(phi1)*sin(phi2)-sin(phi1)*cos(phi2)*cos(delta_lam)
            th = math.atan2(y,x)*180/pi
            th = (th + 360) % 360
            self.tar_heading = th
            self.calc_heading_error()
            return th
        return 0

    def calc_heading_error(self):
        if self.hasGPS and self.hasTarget:
            self.head_err = ang_diff(self.tar_heading, self.heading)
            return self.head_err
        return 0
    
    def calc_dist(self):
        if self.hasGPS and self.hasTarget:
            self.dist = haversine((self.lat, self.lng), (self.tar_lat, self.tar_lng), unit=Unit.METERS)
            if (self.dist < DEF.MIN_DIST):
                self.waypoint_reached = True
            return self.dist   
        return 0
    
    def update_target(self, *args):
        self.waypoint_reached = False
        if len(args) >= 1:
            if (len(args) > 1):
                self.tar_lat = args[0]
                self.tar_lng = args[1]
            else:
                self.tar_lat = args[0][0]
                self.tar_lng = args[0][1]
        else:
            self.hasTarget = False
            return None
        self.hasTarget = True
        self.tar_waypoint = (self.tar_lat, self.tar_lng)
        self.calc_dist()
        self.calc_des_heading()

    def get_lat_lng(self):
        if self.hasGPS:
            return (self.lat, self.lng)
        return None
    
    def get_target(self):
        if self.hasTarget:
            return (self.tar_lat, self.tar_lng)
        return None

    def get_speed(self):
        return self.speed


    def get_heading(self):
        return self.heading
    
    def update_info(self):
        try:
            values = Bridge.call("get_nav_info")
            self.heading = self.correctHeading(values[5]/100.0)
            self.params.heading = self.heading
            if values[0] == 0:
                self.hasGPS = False
                return None
            self.hasGPS = True
            raw_lat = values[1]/1000000.0
            raw_lng = values[2]/1000000.0
            if (self.lat_filter is None):
                self.lat_filter = CoordFilter(raw_lat)
                self.lng_filter = CoordFilter(raw_lng)
                self.lat = raw_lat
                self.lng = raw_lng
            else:
                self.lat = self.lat_filter.update(raw_lat)
                self.lng = self.lng_filter.update(raw_lng)
            self.speed = values[3]*KNOTS_TO_MS/100.0
            self.gps_head = values[4]/100.0
            self.params.lat = self.lat
            self.params.lng = self.lng
            self.params.heading = self.heading
            self.calc_des_heading()
            self.calc_dist()
            self.params.clr_error(ERRS.BRIDGE_ERR)
            last_update = time.perf_counter()
            return (self.lat, self.lng, self.speed, self.heading)
        except Exception as e:
            self.params.set_err(ERRS.BRIDGE_ERR)
            print("Failed to update GPS", e)
            return None
        

    def correctHeading(self, mag_heading):
        if not self.hasGPS or self.speed < 0.4:
            return mag_heading
        MIN_SPEED = 0.4
        MAX_SPEED = 3.0 
        clamped_speed = max(MIN_SPEED, min(self.speed, MAX_SPEED))
        
        # Calculate weight (0.0 = Pure GPS, 1.0 = Pure Magnetometer)
        weight = 1.0 - ((clamped_speed - MIN_SPEED) / (MAX_SPEED - MIN_SPEED))

        delta = self.gps_head - mag_heading
        delta = (delta + 180) % 360 - 180
        corrected = mag_heading + delta * (1.0 - weight)
        return corrected % 360
    
    def clr(self):
        self.wayQue.clear()
        self.reverseQue.clear()

    def add_waypoint(self, waypoint):
        self.wayQue.append(waypoint)
        print(self.wayQue)

    def pop(self):
        if (len(self.wayQue) > 0):
            if self.tar_waypoint is not None:
                self.reverseQue.append(self.tar_waypoint)
            self.tar_waypoint = self.wayQue.popleft()
            return self.tar_waypoint
        return None
    
    def revpop(self):
        if len(self.reverseQue) > 0:
            self.tar_waypoint = self.reverseQue.pop()
            return self.tar_waypoint
        return None
         
    
    def length(self):
        return len(self.wayQue)
    
    def rev_len(self):
        return len(self.reverseQue)

'''
    * Note: The process nosie and measurement noise are the SQUARE of the actual value
    * In 100ms, with a top speed of 5 m/s, the boat can move about 0.5m
    * The GPS has a drift of about 1m in those same 100ms
    * These need to be put into degrees.
''' 
class CoordFilter:
    def __init__(self, init_coord, process_noise=2.0145e-11, measurement_noise=8.05e-11):
        self.x = init_coord
        self.P = 1.0
        self.Q = process_noise
        self.R = measurement_noise

    def update(self, raw_coord):
        x_prior = self.x
        P_prior = self.P + self.Q
        K = P_prior / (P_prior + self.R)
        self.x = x_prior + K*(raw_coord-x_prior)
        self.P = (1.0 - K) * P_prior
        return self.x