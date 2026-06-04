from __future__ import annotations
from typing import TYPE_CHECKING
from arduino.app_utils import *
from definitions import FSM
from definitions import ERRS
from definitions import NEUTRAL
import definitions as DEF
import PID_ as pid
import time
from math import cos as cos
from math import pi as PI

if TYPE_CHECKING:
    from boat_communication import BoatParams

ESC_RANGE = 500 # us

class BoatVars:
    def __init__(self, params : BoatParams, LOOP_TIME : float):
        self.gains_head = [10.0, 0.05, 0.3, 50.0]
        self.gains_pos = [5.0, 0.05, 1.0, 30.0]
        self.state = FSM.MOTORS_OFF
        self.pid_head = pid.PIDF([0, 0, 0, 0], LOOP_TIME)
        self.pid_pos = pid.PIDF([0, 0, 0, 0], LOOP_TIME)
        self.move_dir = -1
        self.pwr_lim = 10
        self.l_thrust = 0.0
        self.r_thrust = 0.0
        self.params = params
        self.last_time = time.perf_counter()*1000
        self.prev_h_err = 0.0
        self.prev_p_err = 0.0
        self.update_gains(self.gains_head, self.gains_pos)

    def update_gains(self, gains_head, gains_pos):
        self.gains_head = gains_head
        self.gains_pos = gains_pos
        self.pid_head.updateGains(self.scale_gains(gains_head, 0.01))
        self.pid_pos.updateGains(gains_pos)
        print("Heading Gains:",gains_head)
        print("Position Gains:", gains_pos)


    def scale_gains(self, gains, multiple):
        for i in range(len(gains)):
            gains[i] = gains[i]*multiple
        return gains
    
    def update_pids(self, h_err, p_err):
        h_hErr_start = 6 # deg
        h_pErr_start = DEF.MIN_DIST # m
        h_norm = abs(h_err)/h_hErr_start
        d_norm = abs(p_err)/h_pErr_start
        h_total_err = min(h_norm, d_norm)
        h_scale = h_total_err**2/(h_total_err**2+0.1)

        p_pErr_start = DEF.MIN_DIST # m
        p_total_err = abs(p_err)/p_pErr_start
        p_scale = p_total_err**4/(p_total_err**4+0.1)

        turn_thrust = self.pid_head.update(h_err, True)
        axial_cos_factor = cos(h_err*PI/180.0)**3
        axial_thrust = self.pid_pos.update(axial_cos_factor*p_err)

        axial_thrust = axial_thrust*(1.0-abs(turn_thrust)/DEF.MAX_POS_THRUST)*p_scale
        turn_thrust *= h_scale
        print("Position Error =", axial_cos_factor**3*p_err)
        print("Thrusts")
        print("\tTurn Thrust =", turn_thrust)
        print("\tAxial Thrust =", axial_thrust)

        # Need to ensure total thrust is not greater than motors can produce
        l_thrust = turn_thrust + axial_thrust
        l_thrust = self.saturate_thrusts(l_thrust)
        r_thrust = -turn_thrust + axial_thrust
        r_thrust = self.saturate_thrusts(r_thrust)
        return (l_thrust, r_thrust)

    def saturate_thrusts(self, thrust):
        if thrust > DEF.MAX_POS_THRUST:
            thrust = DEF.MAX_POS_THRUST
        elif thrust < DEF.MAX_NEG_THRUST:
            thrust = DEF.MAX_NEG_THRUST
        return thrust
    
    def update_dir(self, new_dir):
        self.move_dir = new_dir
    
    def update_pwr(self, new_pwr):
        self.pwr_lim = new_pwr

    def update_thrust_params(self, per_l, per_r):
        self.l_thrust = per_l
        self.r_thrust = per_r
        self.params.l_thrust = self.l_thrust
        self.params.r_thrust = self.r_thrust

    # Updates motor thrusts by converting from thrust force (in N) to PWM
    def update_thrust_force(self, thrusts_F):
        pwm_l = self.convert_to_pwm(thrusts_F[0])
        pwm_r = self.convert_to_pwm(thrusts_F[1])     
        self.send_thrusts(pwm_l, pwm_r)

    # Converts a thrust force (N) to PWM
    def convert_to_pwm(self, force):
        pwm = NEUTRAL
        if force > 0.5: 
            pwm = int(22.965*force+NEUTRAL)
        elif force < -0.5: # IE, for reverse thrust
            pwm = int(15.757*force+NEUTRAL)
        return pwm

    # Sets ESC pwm from % thrust
    def update_thrust_percent(self, percents):
        pwms = []
        for thrust_per in percents:
                # ESC Mapping: 1000us - max, 1500us - min
                pwms.append(int(NEUTRAL+thrust_per*ESC_RANGE))
        self.send_thrusts(pwms[0], pwms[1])

    def pwm_to_percent(self, pwm):
        return (pwm - NEUTRAL)/ESC_RANGE

    def send_thrusts(self, pwm_l, pwm_r):
        print("\tSending Thrusts:", pwm_l, pwm_r, )
        try:
            Bridge.notify("set_thrusts", pwm_l, pwm_r) 
            self.params.clr_error(ERRS.BRIDGE_ERR)
            self.params.l_thrust = self.pwm_to_percent(pwm_l)
            self.params.r_thrust = self.pwm_to_percent(pwm_r)
            return True
        except Exception as e:
            print("Failure to send thrusts", e)
            self.params.set_err(ERRS.BRIDGE_ERR)
            time.sleep(0.5)

    def set_idle_thrust(self):
        self.send_thrusts(NEUTRAL, NEUTRAL)
                            
    # Motor values from webserver are specified in terms of percent. 
    # -1: Motors Idle
    # 1: Motors FWD
    # 2: Motors BCK
    # 3: Motors LEFT
    # 4: Motors RIGHT
    def manual_thrust_update(self):
        if self.move_dir == -1:
            self.send_thrusts(NEUTRAL, NEUTRAL)
            return
        per = self.pwr_lim/100
        if self.move_dir == 1:
            self.update_thrust_percent([per, per])
        elif self.move_dir == 2:
           self.update_thrust_percent([-per, -per])
        elif self.move_dir == 3:
            self.update_thrust_percent([-per, per])
        elif self.move_dir == 4:
            self.update_thrust_percent([per, -per])
        
    def reset_pids(self):
        self.pid_head.reset()
        self.pid_pos.reset()
