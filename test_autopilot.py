import sys
import xpc
import PID
from datetime import datetime, timedelta

update_interval = 0.100 # seconds
start = datetime.now()
last_update = start

# defining the initial PID values
P = 0.1 # PID library default = 0.2
I = P/10 # default = 0
D = 0 # default = 0

# initializing both PID controllers
roll_PID = PID.PID(P, I, D)
pitch_PID = PID.PID(P, I, D)

# setting the desired values
# roll = 0 means wings level
# pitch = 2 means slightly nose up, which is required for level flight
desired_roll = 0
desired_pitch = 2

# setting the PID set points with our desired values
roll_PID.SetPoint = desired_roll
pitch_PID.SetPoint = desired_pitch

def monitor():
    global last_update
    with xpc.XPlaneConnect() as client:
        while True:
            if (datetime.now() > last_update + timedelta(milliseconds = update_interval * 1000)):
                last_update = datetime.now()
                print(f"loop start - {datetime.now()}")

                posi = client.getPOSI();
                ctrl = client.getCTRL();

                current_roll = posi[4]
                current_pitch = posi[3]

                roll_PID.update(current_roll)
                pitch_PID.update(current_pitch)

                new_ail_ctrl = roll_PID.output
                new_ele_ctrl = pitch_PID.output

                ctrl = [new_ele_ctrl, new_ail_ctrl, 0.0, -998] # ele, ail, rud, thr. -998 means don't change
                client.sendCTRL(ctrl)

                output = f"current values --    roll: {current_roll: 0.3f},  pitch: {current_pitch: 0.3f}"
                output = output + "\n" + f"PID outputs    --    roll: {roll_PID.output: 0.3f},  pitch: {pitch_PID.output: 0.3f}"
                output = output + "\n"
                print(output)

if __name__ == "__main__":
    monitor()