#!/usr/bin/env python3
# coding: utf-8
# =========================================================================
# main.py
#
# Copyright (c) the Contributors as noted in the AUTHORS file.
# This file is part of Ingescape, see https://github.com/zeromq/ingescape.
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/.
# =========================================================================
#
import struct
import signal
import getopt
import time
import xpc
from pathlib import Path
import threading

from echo import *

refresh_rate = 0.1
port = 5670
agent_name = "Human_Autopilot"
device = "wlo1"
verbose = False
is_interrupted = False
start_heading = None

## def datarefs string
iasDref = "sim/cockpit2/gauges/indicators/airspeed_kts_pilot"
pitchDref = "sim/cockpit2/gauges/indicators/pitch_AHARS_deg_pilot"
altitudeDref = "sim/cockpit2/gauges/indicator/altitude_ft_pilot"
thrustDref = "sim/flightmodel/engine/ENGN_thro_override"
headingDref = "sim/cockpit2/gauges/indicators/heading_AHARS_deg_mag_pilot" # 0 to 1, override because it is a double instead of an array
rollDref = "sim/cockpit2/gauges/indicators/roll_AHARS_deg_pilot"
parkBrakeDref = "sim/cockpit2/controls/parking_brake_ratio"
verticalSpeedDref = "sim/cockpit2/gauges/indicators/vvi_fpm_pilot"
rudderDref = "sim/cockpit2/controls/yoke_heading_ratio"
elevatorDref = "sim/cockpit2/controls/yoke_pitch_ratio"
aileronDref = "sim/cockpit2/controls/yoke_roll_ratio"


#Define the states
class State:
    def __init__(self, name):
        self.name = name

    def __repr__(self):
        return self.name

# Define the transitions
class Transition:
    def __init__(self, current_state, next_state, action, condition=None):
        self.current_state = current_state
        self.next_state = next_state
        self.action = action
        self.condition = condition
        
    def is_condition_met(self):
        if self.condition is None:
            return True
        return self.condition()

    def __repr__(self):
        return f"{self.current_state} -> {self.next_state} : {self.action}"

# Define the Finite State Machine
class FiniteStateMachine:
    def __init__(self, initial_state):
        self.current_state = initial_state
        self.transitions = []

    def add_transition(self, transition):
        self.transitions.append(transition)

    def run(self):
        print(f"Starting state: {self.current_state}")
        while (not is_interrupted) and igs.is_started():
            for transition in self.transitions:
                if transition.current_state == self.current_state and transition.is_condition_met():
                    print(f"Transitioning from {transition.current_state} to {transition.next_state}")
                    transition.action()
                    self.current_state = transition.next_state
                    transition_found = True
                    break  # Exit the for loop after executing a valid transition

            time.sleep(refresh_rate)  # Sleep to control the refresh rate

        if igs.is_started():
            igs.stop()


# Define the actions

def wait_for_start_action():
    print("Waiting for start.")
    
def is_agent_on_and_aircraft_ready():
    return agent.on_off_i and get_dref(iasDref) <= 0


def acceleration_action():
    print("Full throttle! let's takeoff!")
    set_to_thrust()
    # Start keep_aircraft_aligned() in a new thread
    alignment_thread = threading.Thread(target=keep_aircraft_aligned)
    alignment_thread.daemon = True  # Set as a daemon thread to exit when the main program exits
    alignment_thread.start()

def is_vr_reached():
    return get_dref(iasDref) >= agent.int_vr

def is_vs_positive():
    return get_dref(verticalSpeedDref) > 20

def rotation_action():
    print("VR reached. Rotate!")
    rotation_thread = threading.Thread(target=rotate_aircraft)
    rotation_thread.daemon = True
    rotation_thread.start()

def climb_action():
    print("Climbing to safe altitude.")

# Create the states
wait_for_start = State("WaitForstart")
acceleration = State("Acceleration")
rotation = State("Rotation")
climb = State("Climb")

# Create the transitions
transition1 = Transition(wait_for_start, acceleration, acceleration_action, condition=is_agent_on_and_aircraft_ready)
transition2 = Transition(acceleration, rotation, rotation_action, condition=is_vr_reached)
transition3 = Transition(rotation_action, climb, climb_action, condition=is_vs_positive)

# Create the FSM and add transitions
fsm = FiniteStateMachine(wait_for_start)
fsm.add_transition(transition1)
fsm.add_transition(transition2)
fsm.add_transition(transition3)

def return_io_value_type_as_str(value_type):
    if value_type == igs.INTEGER_T:
        return "Airspeed"
    elif value_type == igs.DOUBLE_T:
        return "Double"
    elif value_type == igs.BOOL_T:
        return "Bool"
    elif value_type == igs.STRING_T:
        return "String"
    elif value_type == igs.IMPULSION_T:
        return "Impulsion"
    elif value_type == igs.DATA_T:
        return "Data"
    else:
        return "Unknown"

def return_event_type_as_str(event_type):
    if event_type == igs.PEER_ENTERED:
        return "PEER_ENTERED"
    elif event_type == igs.PEER_EXITED:
        return "PEER_EXITED"
    elif event_type == igs.AGENT_ENTERED:
        return "AGENT_ENTERED"
    elif event_type == igs.AGENT_UPDATED_DEFINITION:
        return "AGENT_UPDATED_DEFINITION"
    elif event_type == igs.AGENT_KNOWS_US:
        return "AGENT_KNOWS_US"
    elif event_type == igs.AGENT_EXITED:
        return "AGENT_EXITED"
    elif event_type == igs.AGENT_UPDATED_MAPPING:
        return "AGENT_UPDATED_MAPPING"
    elif event_type == igs.AGENT_WON_ELECTION:
        return "AGENT_WON_ELECTION"
    elif event_type == igs.AGENT_LOST_ELECTION:
        return "AGENT_LOST_ELECTION"
    else:
        return "UNKNOWN"

def signal_handler(signal_received, frame):
    global is_interrupted
    print("\n", signal.strsignal(signal_received), sep="")
    is_interrupted = True


def on_agent_event_callback(event, uuid, name, event_data, my_data):
    agent_object = my_data
    assert isinstance(agent_object, Echo)
    # add code here if needed


def on_freeze_callback(is_frozen, my_data):
    agent_object = my_data
    assert isinstance(agent_object, Echo)
    # add code here if needed


# inputs

def bool_input_callback(io_type, name, value_type, value, my_data):
    igs.info(f"Input and output {name} written to {value}")
    agent_object = my_data
    assert isinstance(agent_object, Echo)
    agent_object.on_off_i = value
    igs.output_set_bool("On_Off", value)
    
def integer_input_callback(io_type, name, value_type, value, my_data):
    igs.info(f"Input {name} written to {value}")
    agent_object = my_data
    assert isinstance(agent_object, Echo)
    if name == "V1":
        agent_object.int_v1 = value
    elif name == "VR":
        agent_object.int_vr = value
    elif name == "V2":
        agent_object.int_v2 = value
    elif name == "Safe_Alt":
        agent_object.int_alt = value

def double_input_callback(io_type, name, value_type, value, my_data):
    igs.info(f"Input {name} written to {value}")
    agent_object = my_data
    assert isinstance(agent_object, Echo)
    if name == "Pitch":
        agent_object.double_pitch = value

def get_dref(arg, is_double=False):
    with xpc.XPlaneConnect() as client:
        #get a dref
        dref = arg
        myValue = client.getDREF(dref)
        rounded_value = round(myValue[0], 1)
        return rounded_value

def set_to_thrust():
    global start_heading
    with xpc.XPlaneConnect() as client:
        # Stow landing gear using a dataref
        start_heading = get_dref(headingDref)
        print(f"Start heading: {start_heading}")
        time.sleep(0.1)
        print("Setting T/O thrust : ")
        client.sendDREF(thrustDref, 1)
        time.sleep(1)
        client.sendDREF(parkBrakeDref, 0)
        
def keep_aircraft_aligned():
    global start_heading
    ky = 0.05
    kyc = 0.012
    kyic = 0.028
    kyi = 0.1
    yaw_control_old = 0
    roll_control_old = 0
    old_roll = 0
    time_old = time.time()
    old_heading = start_heading
    with xpc.XPlaneConnect() as client:
        while not is_interrupted:
            current_heading = get_dref(headingDref)
            diffYaw = current_heading - start_heading
            deltayaw = current_heading - old_heading
            time_current = time.time()
            dt = time_current - time_old
            time_old = time_current
            deltaControlYaw = ky * deltayaw + kyi * diffYaw * dt
            yaw_control = yaw_control_old + deltaControlYaw
            client.sendDREF(rudderDref, - yaw_control)
            old_heading = current_heading
            yaw_control_old = yaw_control
            time.sleep(0.05)
            
            current_roll = get_dref(rollDref)
            diffRoll = current_roll - 0
            deltaRoll = current_roll - old_roll
            time_current = time.time()
            dt = time_current - time_old
            time_old = time_current
            deltaControlRoll = kyc * deltaRoll + kyic * diffRoll * dt
            roll_control = roll_control_old + deltaControlRoll
            client.sendDREF(aileronDref, - roll_control)
            old_roll = current_roll
            roll_control_old = roll_control
            time.sleep(0.05)
            
def rotate_aircraft():
    ky = 0.06
    kyi = 0.14
    pitch_control_old = 0
    old_pitch = 0
    target_pitch = agent.double_pitch
    time_old = time.time()
    with xpc.XPlaneConnect() as client:
        while not is_interrupted:
            current_pitch = get_dref(pitchDref)
            diffPitch = current_pitch - target_pitch
            deltaPitch = current_pitch - old_pitch
            time_current = time.time()
            dt = time_current - time_old
            time_old = time_current
            deltaControlPitch = ky * deltaPitch + kyi * diffPitch * dt
            pitch_control = pitch_control_old + deltaControlPitch
            client.sendDREF(elevatorDref, - pitch_control)
            old_pitch = current_pitch
            pitch_control_old = pitch_control
            time.sleep(0.1)


# catch SIGINT handler before starting agent
signal.signal(signal.SIGINT, signal_handler)

igs.agent_set_name(agent_name)
igs.definition_set_version("1.0")
igs.log_set_console(verbose)
igs.log_set_file(True, None)
igs.log_set_stream(verbose)
igs.set_command_line(sys.executable + " " + " ".join(sys.argv))

if device is None:
    # we have no device to start with: try to find one
    list_devices = igs.net_devices_list()
    list_addresses = igs.net_addresses_list()
    if len(list_devices) == 1:
        device = list_devices[0].decode('utf-8')
        igs.info("using %s as default network device (this is the only one available)" % str(device))
    elif len(list_devices) == 2 and (list_addresses[0] == "127.0.0.1" or list_addresses[1] == "127.0.0.1"):
        if list_addresses[0] == "127.0.0.1":
            device = list_devices[1].decode('utf-8')
        else:
            device = list_devices[0].decode('utf-8')
        print("using %s as de fault network device (this is the only one available that is not the loopback)" % str(device))
    else:
        if len(list_devices) == 0:
            igs.error("No network device found: aborting.")
        else:
            igs.error("No network device passed as command line parameter and several are available.")
            print("Please use one of these network devices:")
            for device in list_devices:
                print("	", device)
        exit(1)

agent = Echo()

igs.observe_agent_events(on_agent_event_callback, agent)
igs.observe_freeze(on_freeze_callback, agent)

igs.input_create("On_Off", igs.BOOL_T, None)
igs.input_create("V1", igs.INTEGER_T, None)
igs.input_create("VR", igs.INTEGER_T, None)
igs.input_create("V2", igs.INTEGER_T, None)
igs.input_create("Pitch", igs.DOUBLE_T, None)
igs.input_create("Safe_Alt", igs.INTEGER_T, None)

igs.output_create("On_Off", igs.BOOL_T, None)

igs.observe_input("On_Off", bool_input_callback, agent)
igs.observe_input("V1", integer_input_callback, agent)
igs.observe_input("VR", integer_input_callback, agent)
igs.observe_input("V2", integer_input_callback, agent)
igs.observe_input("Pitch", double_input_callback, agent)
igs.observe_input("Safe_Alt", integer_input_callback, agent)

igs.log_set_console(True)
igs.log_set_console_level(igs.LOG_INFO)

igs.start_with_device(device, port)
# catch SIGINT handler after starting agent
signal.signal(signal.SIGINT, signal_handler)

fsm.run()