import time
import definitions as DEF
from definitions import ang_diff
from math import pi as PI
from math import cos as cos

class PIDF:
    def __init__(self, gains, Ts):
        self.Kp = gains[0]
        self.Ki = gains[1]
        self.Kd = gains[2]
        self.N = gains[3]
        self.Ts = Ts

        self.yi = 0 # This is the total value of the integral term, which must persist between calls
        self.prev_err = 0
        self.yd = 0 # last filtered derviative value
        self.last_time = time.perf_counter()

    # Update the PID with new reference input errors
    def update(self, xn, heading=False):
        now = time.perf_counter()
        dt = now - self.last_time
        self.last_time = now
        Kp, Ki, Kd = self.Kp, self.Ki, self.Kd
        Ts, N = self.Ts, self.N
        Ts = max(0.001, min(dt, self.Ts/1000*2.0)) # integral buildup could be too large, this caps it
        yp = self.Kp*xn
        self.yi = self.yi + Kp*Ki*Ts*xn
        # Ensure yi saturates to avoid integral windup
        if self.yi < DEF.MAX_NEG_THRUST/2:
            self.yi = DEF.MAX_NEG_THRUST/2
        elif self.yi > DEF.MAX_POS_THRUST/2:
            self.yi = DEF.MAX_POS_THRUST/2

        err_diff = xn-self.prev_err
        if heading: # Account for shortest path
            err_diff = ang_diff(xn, self.prev_err)
        self.yd = (Kp*Kd*N*(err_diff) + self.yd)/(N*Ts+1)
        self.prev_err = xn

        result = yp+self.yi+self.yd
        result = self.saturate(result)
        return result
    

    def saturate(self, thrust):
        if thrust > DEF.MAX_POS_THRUST:
            return DEF.MAX_POS_THRUST
        elif thrust < DEF.MAX_NEG_THRUST:
            return DEF.MAX_NEG_THRUST
        return thrust
    
    def reset(self):
        self.yi = 0.0
        self.yd = 0.0
        self.prev_err = 0.0

    def updateGains(self, gains):
        self.Kp = gains[0]
        self.Ki = gains[1]
        self.Kd = gains[2]
        self.N = gains[3]