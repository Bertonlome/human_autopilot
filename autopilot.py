import sys
import xpc
import PID
import signal
from datetime import datetime, timedelta
#import pyqtgraph as pg
#from pyqtgraph.Qt import QtCore, QtGui
import time
import numpy as np
from echo import *

SPEED = 90 # this is the speed of the plane in XPlane, used to determine if we are on ground or not
refresh_rate = 0.1
port = 5670
agent_name = "Human_Autopilot"
device = "wlo1"
verbose = False
is_interrupted = False
start_heading = None

def normalize(value, min=-1, max=1):
    # if value = 700, and max = 20, return 20
    # if value = -200, and min = -20, return -20
    if (value > max):
        return max
    elif (value < min):
        return min  
    else:
        return value

update_interval = 0.025 # seconds, 0.05 = 20 Hz
start = datetime.now()
last_update = start

# defining the initial PID values
P_pitch = 0.1 # PID library default = 0.2
P_roll = 0.01
P_skid = 0.01
I_pitch = P_pitch/10 # default = 0
I_roll = P_roll/10
I_skid = P_skid/10
D = 0 # default = 0

# initializing PID controllers
roll_PID = PID.PID(P_roll, I_roll, D)
pitch_PID = PID.PID(P_pitch, I_pitch, D)
altitude_PID = PID.PID(P_pitch, I_pitch, D)
skid_PID = PID.PID(P_skid, I_skid, D)
rudder_PID = PID.PID(P_skid, I_skid, D)



# setting the desired values
# roll = 0 means wings level
# pitch = 2 means slightly nose up, which is required for level flight
desired_roll = 0
desired_pitch = 10
desired_altitude = 4500
desired_skid = 0


# # setting the PID set points with our desired values
pitch_PID.SetPoint = desired_pitch
altitude_PID.SetPoint = desired_altitude
skid_PID.SetPoint = desired_skid

# x_axis_counters = [] #0, 1, 2, 3, etc. just basic x-axis values used for plotting
# roll_history = []
# pitch_history = []
# altitude_history = []
# roll_setpoint_history = []
# pitch_setpoint_history = []
# altitude_setpoint_history = []
# plot_array_max_length = 300 # how many data points to hold in our arrays and graph
# \

DREFs = ["sim/cockpit2/gauges/indicators/airspeed_kts_pilot",
        "sim/cockpit2/gauges/indicators/heading_electric_deg_mag_pilot",
        "sim/flightmodel/failures/onground_any",
        "sim/flightmodel/misc/h_ind",
        "sim/flightmodel/controls/parkbrake",
        "sim/cockpit2/engine/actuators/throttle_ratio_all"
        ]

def monitor():
    agent.on_off_i = False
    while(agent.on_off_i == False):
        time.sleep(0.1)
        
    global i
    global last_update
    start = True
    desired_roll = 0
    hwg = True
    with xpc.XPlaneConnect() as client:
        while True:
                
            if (datetime.now() > last_update + timedelta(milliseconds = update_interval * 1000)):
                last_update = datetime.now()
                #print(f"loop start - {datetime.now()}")

                posi = client.getPOSI();
                ctrl = client.getCTRL();
                multi_DREFs = client.getDREFs(DREFs)

                current_roll = posi[4]
                #current_roll = agent.roll_i
                #print(f"roll : {current_roll}")
                #current_hdg = agent.heading_i
                #print(f"hdg : {current_hdg}")
                #current_pitch = agent.pitch_i
                #print(f"pitch : {current_pitch}")
                #current_altitude = agent.int_alt
                #print(f"alt : {current_altitude}")
                #current_asi = agent.airspeed_i
                #print(f"airspeed : {current_asi}")
                
                
                current_pitch = posi[3]
                current_hdg = multi_DREFs[1][0]
                current_altitude = multi_DREFs[3][0]
                current_asi = multi_DREFs[0][0]
                onground = multi_DREFs[2][0]
                current_rudder = ctrl[2]
                current_skid = client.getDREF("sim/cockpit2/gauges/indicators/slip_deg")[0]

                print("Rudder position:", current_rudder)
                print("Skid position:", current_skid)
                
                # if the plane is on ground, set the park brake and throttle to 0
                if (start):
                    print("start")
                    #agent.control_thrust_o = 1
                    #igs.output_set_double("Thrust", 1)
                    #time.sleep(5)
                    #igs.output_set_double("Parking_Brake", 0)
                    #agent.control_parking_brake_o = 0
                    client.sendDREF("sim/flightmodel/controls/parkbrake", 0.0)
                    time.sleep(5)
                    client.sendDREF("sim/cockpit2/engine/actuators/throttle_ratio_all", 1.0)
                    time.sleep(5)
                    start = False
                    
                #print(current_asi)
                if(current_asi > SPEED):  
                    marge_derreur = 2 # 5 degrees of error
                    #print(f"Agent heading input = {agent.heading_i}")
                    user_requested_heading = agent.heading_t_i #roll_PID.user_requested_heading
                    current_hdg
                    # Calcul de la différence minimale d'angle
                    if user_requested_heading is not None:
                        bearing_difference = (user_requested_heading - current_hdg + 180) % 360 - 180
                        if(bearing_difference < 0):
                            desired_roll = -30 
                        elif(bearing_difference > 0):
                            desired_roll = 30
                        else:
                            desired_roll = 0  
                        if(np.abs(bearing_difference) < marge_derreur):
                            desired_roll = 0 

                        roll_PID.SetPoint = desired_roll
                    
                    # update outer loops first
                    #print (agent.int_alt)
                    if agent.int_alt_target is not None:
                        altitude_PID.SetPoint = agent.int_alt_target
                    altitude_PID.update(current_altitude)
                    #skid_PID.update(current_skid)
                    # if alt=12000, setpoint = 10000, the error is 2000. if P=0.1, output will be 2000*0.1=200
                    pitch_PID.SetPoint = normalize(altitude_PID.output, min=-15, max=10)
                    #rudder_PID.SetPoint = normalize(skid_PID.output)
                    
                    # update PIDs
                    roll_PID.update(current_roll)
                    pitch_PID.update(current_pitch)
                    #rudder_PID.update(current_rudder)
                    #skid_PID.update(current_skid)

                    # update control outputs
                    new_ail_ctrl = normalize(roll_PID.output)
                    new_ele_ctrl = normalize(pitch_PID.output)
                    new_rudder_ctrl = normalize(rudder_PID.output,min=-1,max=1)

                    #sending actual control values to XPlane
                    #igs.output_set_double("Control_Pitch", new_ele_ctrl)
                    #igs.output_set_double("Control_Roll", new_ail_ctrl)
                    #agent.control_roll_o = new_ail_ctrl
                    #agent.control_yaw_o = new_rudder_ctrl
                    ctrl = [new_ele_ctrl, new_ail_ctrl, new_rudder_ctrl , -998] # -998 pour ne pas changer le moteur
                    client.sendCTRL(ctrl)


                    output = f"current values --    roll: {current_roll: 0.3f},  pitch: {current_pitch: 0.3f}"
                    output = output + "\n" + f"PID outputs    --    roll: {roll_PID.output: 0.3f},  pitch: {pitch_PID.output: 0.3f}"
                    output = output + "\n"
                    #print(output)


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
    #igs.info(f"Input and output {name} written to {value}")
    agent_object = my_data
    assert isinstance(agent_object, Echo)
    if name == "On_Off":
        agent_object.on_off_i = value
        print(f" Agent on_off input = {agent.on_off_i}")
    if name == "On_Off_Takeoff":
        agent.on_off_i_takeoff = value
        print(agent.on_off_i_takeoff)
    if name == "On_Off_Cap":
        agent.on_off_i_cap = value
        print(agent.on_off_i_cap)

def integer_input_callback(io_type, name, value_type, value, my_data):
    igs.info(f"Input {name} written to {value}")
    agent_object = my_data
    assert isinstance(agent_object, Echo)

def double_input_callback(io_type, name, value_type, value, my_data):
    #igs.info(f"Input {name} written to {value}")
    agent_object = my_data
    assert isinstance(agent_object, Echo)
    if name == "Pitch":
        agent_object.pitch_i = value
    if name == "Pitch_Target":
        agent_object.pitch_t_i = value
    if name == "Heading":
        agent_object.heading_i = value
    if name == "Headin_Target":
        agent_object.heading_t_i = value
    if name == "Altitude":
        agent_object.int_alt = value
    if name == "Altitude_Target":
        agent_object.int_alt_target = value
    if name == "Airspeed":
        agent_object.airspeed_i = value
    if name == "Airspeed_Target":
        agent_object.airspeed_t_i = value
    if name == "Vertical_Speed":
        agent_object.vertical_speed_i = value
    if name == "Vertical_Speed_Target":
        agent_object.vertical_speed_t_i = value
    if name == "Roll":
        agent_object.roll_i = value
    if name == "Roll_Target":
        agent_object.roll_t_i = value


if __name__ == "__main__":
    
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
    #anes
    igs.input_create("Heading", igs.DOUBLE_T, None)
    igs.input_create("Heading_Target", igs.DOUBLE_T, None)
    igs.input_create("Airspeed", igs.DOUBLE_T, None)
    igs.input_create("Airspeed_Target", igs.DOUBLE_T, None)
    igs.input_create("Altitude", igs.DOUBLE_T, None)
    igs.input_create("Altitude_Target", igs.DOUBLE_T, None)
    igs.input_create("Vertical_Speed", igs.DOUBLE_T, None)
    igs.input_create("Vertical_Speed_Target", igs.DOUBLE_T, None)
    igs.input_create("Roll", igs.DOUBLE_T, None)
    igs.input_create("Roll_Target", igs.DOUBLE_T, None)
    igs.input_create("Pitch", igs.DOUBLE_T, None)
    igs.input_create("Pitch_Target", igs.DOUBLE_T, None)
    #keep_safe_alt prend Safe_Alt entree


    igs.output_create("Status", igs.STRING_T, None)
    igs.output_create("Control_Pitch", igs.DOUBLE_T, None)
    #anes
    igs.output_create("Control_Roll", igs.DOUBLE_T, None)
    igs.output_create("Control_Yaw", igs.DOUBLE_T, None)
    igs.output_create("Thrust", igs.DOUBLE_T, None)
    igs.output_create("Parking_Brake", igs.DOUBLE_T, None)


    igs.observe_input("On_Off", bool_input_callback, agent)
    igs.observe_input("Pitch", double_input_callback, agent)
    #anes
    igs.observe_input("Heading", double_input_callback, agent)
    igs.observe_input("Altitude", double_input_callback, agent)
    igs.observe_input("Airspeed", double_input_callback, agent)
    igs.observe_input("Vertical_Speed", double_input_callback, agent)
    igs.observe_input("Roll",double_input_callback,agent)


    igs.log_set_console(True)
    igs.log_set_console_level(igs.LOG_INFO)

    igs.start_with_device(device, port)
    # catch SIGINT handler after starting agent
    signal.signal(signal.SIGINT, signal_handler)
    
    monitor()
    