# from arduino.app_utils import *
import time
import boat_state
from definitions import FSM
from definitions import ERRS
import boat_communication
import nav as NAV
from arduino.app_utils import *

LOOP_TIME = 100 # ms, how often the PID occurs

print("Awaiting Arduino....")
time.sleep(8) # ensure ESCs are not spinning and give time for the Arduino side to boot

def get_time_ms():
    return time.perf_counter()*1000

print("Initializing....")
params = boat_communication.BoatParams()
boat = boat_state.BoatVars(params, LOOP_TIME)
nav = NAV.Nav_Info(params)

parser = boat_communication.Parser(nav, params, boat)
lora = boat_communication.LoRa(parser, params)

prevGPS = None
prevState = None
stateChange = False
receiveFlag = False

def transmit_telem(currTime):
    if currTime > lora.last_receive + lora.transmission_interval/2:
        if currTime > lora.transmission_interval + lora.last_transmission:
            try:
                telem_params = Bridge.call("get_telemetry")
                params.update_telem(telem_params)
                lora.transmit()
                params.clr_error(ERRS.BRIDGE_ERR)
            except Exception as e:
                print("Telemetry failure", e)
                params.set_err(ERRS.BRIDGE_ERR)      

def receive():
    global receiveFlag
    receiveFlag = True

try:
    Bridge.provide("py_receive", receive)
    params.clr_error(ERRS.BRIDGE_ERR)
except Exception:
    params.set_err(ERRS.BRIDGE_ERR)

def state_AUTO():
    if stateChange:
        boat.reset_pids()
    if nav.get_target() is None:
        params.auto = False
        params.set_err(ERRS.NO_TARGET) # Error Code: No Target to navigate to.
        boat.state = FSM.IDLE
    else:
        params.clr_error(ERRS.NO_TARGET)
        if stateChange:
            params.auto = True
        if nav.waypoint_reached:
            next_wp = nav.pop()
            if next_wp is not None:
                nav.update_target(next_wp)
            else:
                params.auto = False
                boat.state = FSM.HOLD_POS
                return
        boat.update_thrust_force(boat.update_pids(nav.head_err, nav.calc_dist()))        

def state_HOLD(last_gps):
    if prevGPS is None or nav.hasGPS == False :
        params.hold = False
        params.set_err(ERRS.NO_TARGET) # Error Code: No Target to navigate to.
        boat.state = FSM.IDLE
        return
    params.clr_error(ERRS.NO_TARGET)
    if stateChange:
        boat.reset_pids()
        params.hold = True
        nav.update_target(last_gps)
    if not nav.hasTarget:
        boat.state = FSM.IDLE
        params.set_err(ERRS.NO_TARGET)
        params.hold = False
        return
    boat.update_thrust_force(boat.update_pids(nav.head_err, nav.calc_dist()))
    
prev_dir = -1
last_manual_cmd = 0
def state_MAN():
    global prev_dir, last_manual_cmd
    curr_time = get_time_ms()
    if stateChange:
        last_manual_cmd = curr_time
    if params.pwr == False:
        boat.state = FSM.MOTORS_OFF
        return
    '''Currently, boat stops if no coast cmd has been sent for 3s'''
    if boat.move_dir >= 0 and curr_time - last_manual_cmd < 3000: # **UPDATE** 
        if boat.move_dir == 0:
            boat.state = FSM.HOLD_POS
            return
        if (prev_dir != boat.move_dir):
            last_manual_cmd = curr_time
            boat.manual_thrust_update()
            prev_dir = boat.move_dir
        return
    elif params.hold == True:
        boat.state = FSM.HOLD_POS
    elif params.auto == True:
        boat.state = FSM.AUTO_NAV
    else:
        boat.state = FSM.IDLE
    prev_dir = -1

# Inhibits motors from running. 
def state_PWR():
    if stateChange:
        boat.set_idle_thrust()
        boat.reset_pids()
        try:
            Bridge.notify("set_pwr", False)
            params.clr_error(ERRS.BRIDGE_ERR)
        except Exception as e:
            params.set_err(ERRS.BRIDGE_ERR)
            print("Failed to send thrusts", e)

def state_IDLE():
    global prev_dir
    # Need to ensure that the ESCs return to neutral
    if stateChange:
        prev_dir = -1
        boat.set_idle_thrust()

# Returns the boat along the same waypoints back to the original waypoint.
def state_RETURN():
    if stateChange:
        boat.reset_pids()
        params.Return = True
        params.hold = False
        params.auto = False
        next_wp = nav.revpop()
        if next_wp is not None:
            nav.update_target(next_wp)
        else:
            params.Return = False
            boat.state = FSM.HOLD_POS
            return
    elif nav.waypoint_reached:
        next_wp = nav.revpop()
        if next_wp is not None:
            nav.update_target(next_wp)
        else:
            params.Return = False
            boat.state = FSM.HOLD_POS
            return
    boat.update_thrust_force(boat.update_pids(nav.head_err, nav.calc_dist()))        
    
next_loop_time = get_time_ms() + LOOP_TIME
while True:
    if receiveFlag:
        lora.receive()
        receiveFlag = False
    curr_state = boat.state
    stateChange = False
    if curr_state != prevState:
        stateChange = True
        print("***STATE CHANGE***")
        print("    ", prevState, "->", curr_state)
    start_time = get_time_ms()

    transmit_telem(start_time)
    nav.update_info()
    
    if nav.hasGPS == False:
        params.set_err(ERRS.NO_GPS)
        params.hold = False
        params.auto = False 

    # Routinely update GPS position unless hold mode is on
    if curr_state != FSM.HOLD_POS:
        prevGPS = nav.get_lat_lng()

    # Actual FSM state logic
    if curr_state == FSM.MOTORS_OFF or params.pwr == False:
        state_PWR()
    elif curr_state == FSM.MANUAL_CNTRL:
        state_MAN()
    elif curr_state == FSM.IDLE:
        state_IDLE()
    else:
        if nav.hasGPS:
            params.clr_error(ERRS.NO_GPS)
            if curr_state == FSM.RETURN:
                state_RETURN()    
            elif curr_state == FSM.HOLD_POS:
                state_HOLD(prevGPS)
            elif curr_state == FSM.AUTO_NAV:
                state_AUTO()
    curr_time = get_time_ms()

    # Check to see if the last received transmission was over 15s ago. If it is, we're going to activate the return state.
    if (curr_time - lora.last_receive > 15*1e3):
        if params.pwr == False:
            boat.state = FSM.MOTORS_OFF
        elif (nav.rev_len() > 1 ):
            boat.state = FSM.RETURN
        else:
            boat.state = FSM.HOLD_POS

    # print("Current State: ", curr_state)
    # print("Total Loop Time: ", get_time_ms() - start_time)
    # print("PWR, AUTO, HOLD", params.pwr, params.hold, params.auto)
    if curr_time < next_loop_time:
        time.sleep((next_loop_time - curr_time)/1000.0)
        next_loop_time += LOOP_TIME
    else:
        next_loop_time = curr_time + LOOP_TIME # Reset the clock if we fell behind
    prevState = curr_state


App.run()