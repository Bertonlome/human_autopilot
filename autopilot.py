import sys
import xpc
import PID
import signal
from datetime import datetime, timedelta
import time
import numpy as np
from echo import *

SPEED = 90 # this is the speed of the plane in XPlane, used to determine if we are on ground or not
refresh_rate = 0.1
port = 5670
agent_name = "Human_Autopilot"
device = "A7500_NETGEAR" 
verbose = False
is_interrupted = False
start_heading = None

def on_agent_event_callback(event, uuid, name, event_data, my_data):
    agent_object = my_data
    assert isinstance(agent_object, Echo)
    # add code here if needed

def on_freeze_callback(is_frozen, my_data):
    agent_object = my_data
    assert isinstance(agent_object, Echo)
    # add code here if needed

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
P_pitch = 0.07 # PID library default = 0.2
#P_roll = 0.01
P_roll = 0.01
P_yaw = 0.025
I_pitch = P_pitch/10 # default = 0
#I_pitch = 0
#I_roll = 0
I_roll = P_roll/10
I_yaw = P_yaw/10
D = 0 # default = 0

# initializing PID controllers
roll_PID = PID.PID(P_roll, I_roll, D)
pitch_PID = PID.PID(P_pitch, I_pitch, D)
altitude_PID = PID.PID(P_pitch, I_pitch, D)
yaw_PID = PID.PID(P_yaw, I_yaw, D)

# setting the desired values
# roll = 0 means wings level
# pitch = 2 means slightly nose up, which is required for level flight
desired_roll = 0
desired_pitch = 10
desired_altitude = 1500
desired_skid = 0

# # setting the PID set points with our desired values
pitch_PID.SetPoint = desired_pitch
altitude_PID.SetPoint = desired_altitude
yaw_PID.SetPoint = desired_skid

DREFs = ["sim/cockpit2/gauges/indicators/airspeed_kts_pilot",
        "sim/cockpit2/gauges/indicators/heading_electric_deg_mag_pilot",
        "sim/flightmodel/failures/onground_any",
        "sim/flightmodel/misc/h_ind",
        "sim/flightmodel/controls/parkbrake",
        "sim/cockpit2/engine/actuators/throttle_ratio_all"
        ]

def main():
    global agent
    agent.on_off_i = False
    while(agent.on_off_i == False):
        time.sleep(0.1)
        
    global i
    global last_update
    start = True
    global desired_roll 
    igs.output_set_double("controlPitch", 0)
    igs.output_set_double("controlRoll", 0)
    igs.output_set_double("controlYaw", 0)
    while True:
        if (datetime.now() > last_update + timedelta(milliseconds = update_interval * 1000)):

            last_update = datetime.now()
            

            #posi = client.getPOSI();
            #ctrl = client.getCTRL();
            #multi_DREFs = client.getDREFs(DREFs)

            #current_roll = posi[4]
            current_roll = agent.roll_i
            #print(f"roll : {current_roll}")
            current_hdg = agent.heading_i
            #print(f"hdg : {current_hdg}")
            current_pitch = agent.pitch_i
            #print(f"pitch : {current_pitch}")
            current_altitude = agent.int_alt
            #print(f"alt : {current_altitude}")
            current_asi = agent.airspeed_i
            #print(f"airspeed : {current_asi}")
            #current_pitch = posi[3]
            #current_hdg = multi_DREFs[1][0]
            #current_altitude = multi_DREFs[3][0]
            #current_asi = multi_DREFs[0][0]
            #onground = multi_DREFs[2][0]
            #current_rudder = client.getDREF("sim/joystick/yoke_heading_ratio")[0]
            #current_skid = client.getDREF("sim/cockpit2/gauges/indicators/slip_deg")[0]
            current_rudder = agent.control_yaw_o if agent.control_yaw_o is not None else -998
            current_skid = agent.skid_i if agent.skid_i is not None else -998
            print(f"\rRudder position: {current_rudder:8.3f} | Current skid: {current_skid:8.3f}", end='', flush=True)
            
            # if the plane is on ground, set the park brake and throttle to 0
            if (start):
                print("start")
                igs.output_set_double("thrust", 1)
                time.sleep(8)
                igs.output_set_double("parkingBrake", 0)
                #client.sendDREF("sim/cockpit2/engine/actuators/throttle_ratio_all", 1.0)
                #time.sleep(10)
                #client.sendDREF("sim/flightmodel/controls/parkbrake", 0.0)
                user_requested_heading = agent.heading_i
                print(f"runway heading: {user_requested_heading}")
                start = False
                
            #print(current_asi)
            if(current_asi > SPEED):  
                #print("speed > 90; roll PID is active")
                marge_derreur = 2 # 5 degrees of error
                #print(f"Agent heading input = {agent.heading_i}")
                if agent.heading_t_i is not None:
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
                altitude_PID.SetPoint = agent.int_alt_target if agent.int_alt_target is not None else desired_altitude
                altitude_PID.update(current_altitude)
                #skid_PID.update(current_skid)
                # if alt=12000, setpoint = 10000, the error is 2000. if P=0.1, output will be 2000*0.1=200
                pitch_PID.SetPoint = normalize(altitude_PID.output, min=-15, max=10)
                #rudder_PID.SetPoint = normalize(skid_PID.output)
                
                # update PIDs
                roll_PID.update(current_roll)
                pitch_PID.update(current_pitch)
                if (current_skid > 0.7 or current_skid < -0.7):
                    yaw_PID.update(current_skid)
                    new_rudder_ctrl = normalize(yaw_PID.output,min=-1,max=1)
                else:
                    new_rudder_ctrl = current_rudder
                
                #print(f"new rudder control: {new_rudder_ctrl}")

                # update control outputs
                new_ail_ctrl = normalize(roll_PID.output)
                new_ele_ctrl = normalize(pitch_PID.output)

                #sending actual control values to XPlane
                #igs.output_set_double("Control_Pitch", new_ele_ctrl)
                #igs.output_set_double("Control_Roll", new_ail_ctrl)
                #agent.control_roll_o = new_ail_ctrl
                #agent.control_yaw_o = new_rudder_ctrl
                #ctrl = [new_ele_ctrl, new_ail_ctrl, new_rudder_ctrl , -998] # -998 pour ne pas changer le moteur
                agent.control_pitch_o = new_ele_ctrl
                agent.control_roll_o = new_ail_ctrl
                agent.control_yaw_o = new_rudder_ctrl
                igs.output_set_double("controlPitch", new_ele_ctrl)
                igs.output_set_double("controlRoll", new_ail_ctrl)
                igs.output_set_double("controlYaw", new_rudder_ctrl)
                #client.sendCTRL(ctrl)
                output = f"current values --    roll: {current_roll: 0.3f},  pitch: {current_pitch: 0.3f}"
                output = output + "\n" + f"PID outputs    --    roll: {roll_PID.output: 0.3f},  pitch: {pitch_PID.output: 0.3f}"
                output = output + "\n"
                #print(output)

def signal_handler(signal_received, frame):
    global is_interrupted
    print("\n", signal.strsignal(signal_received), sep="")
    is_interrupted = True

def bool_input_callback(io_type, name, value_type, value, my_data):
    #igs.info(f"Input and output {name} written to {value}")
    agent_object = my_data
    assert isinstance(agent_object, Echo)
    if name == "on_off":
        agent_object.on_off_i = value

def integer_input_callback(io_type, name, value_type, value, my_data):
    igs.info(f"Input {name} written to {value}")
    agent_object = my_data
    assert isinstance(agent_object, Echo)

def double_input_callback(io_type, name, value_type, value, my_data):
    #igs.info(f"Input {name} written to {value}")
    agent_object = my_data
    assert isinstance(agent_object, Echo)
    if name == "pitch":
        agent_object.pitch_i = value
    if name == "pitchTarget":
        agent_object.pitch_t_i = value
    if name == "heading":
        agent_object.heading_i = value
    if name == "headingTarget":
        agent_object.heading_t_i = value
    if name == "altitude":
        agent_object.int_alt = value
    if name == "altitudeTarget":
        print(f"altitude target input: {value}")
        agent_object.int_alt_target = value
    if name == "airspeed":
        agent_object.airspeed_i = value
    if name == "airspeedTarget":
        agent_object.airspeed_t_i = value
    if name == "verticalSpeed":
        agent_object.vertical_speed_i = value
    if name == "verticalSpeedTarget":
        agent_object.vertical_speed_t_i = value
    if name == "roll":
        agent_object.roll_i = value
    if name == "rollTarget":
        agent_object.roll_t_i = value
    if name == "skid":
        agent_object.skid_i = value


# catch SIGINT handler before starting agent
signal.signal(signal.SIGINT, signal_handler)

igs.agent_set_name(agent_name)
igs.definition_set_version("1.0")
igs.log_set_console(verbose)
igs.log_set_file(True, None)
igs.log_set_stream(verbose)
igs.set_command_line(sys.executable + " " + " ".join(sys.argv))

agent = Echo()

igs.observe_agent_events(on_agent_event_callback, agent)
igs.observe_freeze(on_freeze_callback, agent)

igs.input_create("on_off", igs.BOOL_T, None)
igs.input_create("heading", igs.DOUBLE_T, None)
igs.input_create("headingTarget", igs.INTEGER_T, None)
igs.input_create("airspeed", igs.DOUBLE_T, None)
igs.input_create("airspeedTarget", igs.INTEGER_T, None)
igs.input_create("altitude", igs.DOUBLE_T, None)
igs.input_create("altitudeTarget", igs.INTEGER_T, None)
igs.input_create("verticalSpeed", igs.DOUBLE_T, None)
igs.input_create("verticalSpeedTarget", igs.INTEGER_T, None)
igs.input_create("roll", igs.DOUBLE_T, None)
igs.input_create("rollTarget", igs.INTEGER_T, None)
igs.input_create("pitch", igs.DOUBLE_T, None)
igs.input_create("pitchTarget", igs.INTEGER_T, None)
igs.input_create("skid", igs.DOUBLE_T, None)

igs.output_create("controlPitch", igs.DOUBLE_T, None)
igs.output_create("controlRoll", igs.DOUBLE_T, None)
igs.output_create("controlYaw", igs.DOUBLE_T, None)
igs.output_create("thrust", igs.DOUBLE_T, None)
igs.output_create("parkingBrake", igs.DOUBLE_T, None) # tbdeleted

igs.observe_input("on_off", bool_input_callback, agent)
igs.observe_input("pitch", double_input_callback, agent)
igs.observe_input("heading", double_input_callback, agent)
igs.observe_input("altitude", double_input_callback, agent)
igs.observe_input("airspeed", double_input_callback, agent)
igs.observe_input("verticalSpeed", double_input_callback, agent)
igs.observe_input("roll", double_input_callback, agent)
igs.observe_input("pitchTarget", double_input_callback, agent)
igs.observe_input("headingTarget", double_input_callback, agent)
igs.observe_input("airspeedTarget", double_input_callback, agent)
igs.observe_input("altitudeTarget", double_input_callback, agent)
igs.observe_input("verticalSpeedTarget", double_input_callback, agent)
igs.observe_input("rollTarget", double_input_callback, agent)
igs.observe_input("skid", double_input_callback, agent)

igs.log_set_console(True)
igs.log_set_console_level(igs.LOG_INFO)

igs.start_with_device(device, port)
# catch SIGINT handler after starting agent
signal.signal(signal.SIGINT, signal_handler)

main()