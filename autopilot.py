import sys
import xpc
import PID
from datetime import datetime, timedelta
#import pyqtgraph as pg
#from pyqtgraph.Qt import QtCore, QtGui
import time
import numpy as np

SPEED = 90 # this is the speed of the plane in XPlane, used to determine if we are on ground or not

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
    global i
    global last_update
    start = True
    desired_roll = 0
    hwg = True
    with xpc.XPlaneConnect() as client:
        while True:
            
            if (datetime.now() > last_update + timedelta(milliseconds = update_interval * 1000)):
                last_update = datetime.now()
                print(f"loop start - {datetime.now()}")

                posi = client.getPOSI();
                ctrl = client.getCTRL();
                multi_DREFs = client.getDREFs(DREFs)

                current_roll = posi[4]
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
                    client.sendDREF("sim/flightmodel/controls/parkbrake", 0.0)
                    client.sendDREF("sim/cockpit2/engine/actuators/throttle_ratio_all", 1.0)
                    time.sleep(15)
                    start = False
                    
                #print(current_asi)
                if(current_asi > SPEED):  
                    if(current_altitude > 1900 and hwg): #and bearing difference > 30  #and input igs through roll_PID.user_request_a_roll
                        marge_derreur = 2 # 5 degrees of error
                        user_requested_heading = 80 #roll_PID.user_requested_heading
                        current_heading = current_hdg
                        # Calcul de la différence minimale d'angle
                        bearing_difference = (user_requested_heading - current_hdg + 180) % 360 - 180
                        if(bearing_difference < 0):
                            desired_roll = -30 
                        elif(bearing_difference > 0):
                            desired_roll = 30
                        else:
                            desired_roll = 0  
                        if(np.abs(bearing_difference) < marge_derreur):
                            desired_roll = 0 
                            hwg = False                   
                    
                    
                    roll_PID.SetPoint = desired_roll
                    
                    # update outer loops first
                    altitude_PID.update(current_altitude)
                    skid_PID.update(current_skid)
                    # if alt=12000, setpoint = 10000, the error is 2000. if P=0.1, output will be 2000*0.1=200
                    pitch_PID.SetPoint = normalize(altitude_PID.output, min=-15, max=10)
                    rudder_PID.SetPoint = normalize(skid_PID.output)
                    
                    # update PIDs
                    roll_PID.update(current_roll)
                    pitch_PID.update(current_pitch)
                    rudder_PID.update(current_rudder)
                    #skid_PID.update(current_skid)

                    # update control outputs
                    new_ail_ctrl = normalize(roll_PID.output)
                    new_ele_ctrl = normalize(pitch_PID.output)
                    new_rudder_ctrl = normalize(rudder_PID.output,min=-1,max=1)

                    # sending actual control values to XPlane
                    
                    ctrl = [new_ele_ctrl, new_ail_ctrl, new_rudder_ctrl , -998] # -998 pour ne pas changer le moteur
                    client.sendCTRL(ctrl)


                    output = f"current values --    roll: {current_roll: 0.3f},  pitch: {current_pitch: 0.3f}"
                    output = output + "\n" + f"PID outputs    --    roll: {roll_PID.output: 0.3f},  pitch: {pitch_PID.output: 0.3f}"
                    output = output + "\n"
                    #print(output)
if __name__ == "__main__":
    monitor()