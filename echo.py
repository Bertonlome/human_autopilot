# coding: utf-8

# =========================================================================
# echo_example.py
#
# Copyright (c) the Contributors as noted in the AUTHORS file.
# This file is part of Ingescape, see https://github.com/zeromq/ingescape.
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/.
# =========================================================================


import ingescape as igs
import sys


class Singleton(type):
    _instances = {}
    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]


class Echo(metaclass=Singleton):
    def __init__(self):
        # inputs
        self.on_off_i = None
        self.int_alt = None
        self.int_alt_target = None
        self.heading_t_i = None

        self.airspeed_i = None
        self.airspeed_t_i = None
        self.heading_i = None
        self.vertical_speed_i = None
        self.vertical_speed_t_i = None
        #anes
        self.on_off_i_takeoff = None
        self.on_off_i_cap = None
        self.roll_i = None
        self.roll_t_i = None 
        self.pitch_i = None
        self.pitch_t_i = None
        self.skid_i = None
        
        
        # outputs
        self._on_off_o = None
        self.control_pitch_o = None
        self.control_yaw_o = None
        self.control_roll_o = None
        self.control_thrust_o = None
        self.control_parking_brake_o = None
        self.status_o = None

        # Add setters and getters for outputs
        @property
        def control_pitch_o(self):
            return self._control_pitch_o

        @control_pitch_o.setter
        def control_pitch_o(self, value):
            self._control_pitch_o = value
            if self._control_pitch_o is not None:
                igs.output_set_double("controlPitch", self._control_pitch_o)

        @property
        def control_yaw_o(self):
            return self._control_yaw_o

        @control_yaw_o.setter
        def control_yaw_o(self, value):
            self._control_yaw_o = value
            if self._control_yaw_o is not None:
                igs.output_set_double("controlYaw", self._control_yaw_o)

        @property
        def control_roll_o(self):
            return self._control_roll_o

        @control_roll_o.setter
        def control_roll_o(self, value):
            self._control_roll_o = value
            if self._control_roll_o is not None:
                igs.output_set_double("controlRoll", self._control_roll_o)

        @property
        def control_thrust_o(self):
            return self._control_thrust_o

        @control_thrust_o.setter
        def control_thrust_o(self, value):
            self._control_thrust_o = value
            if self._control_thrust_o is not None:
                print("Applying Thrust !!!")
                igs.output_set_double("thrust", self._control_thrust_o)

        @property
        def control_parking_brake_o(self):
            return self._control_parking_brake_o

        @control_parking_brake_o.setter
        def control_parking_brake_o(self, value):
            self._control_parking_brake_o = value
            if self._control_parking_brake_o is not None:
                igs.output_set_bool("parkingBrake", self._control_parking_brake_o)

    # services
    def receive_values(self, sender_agent_name, sender_agent_uuid, boolV, integer, double, string, data, token, my_data):
        igs.info(f"Service receive_values called by {sender_agent_name} ({sender_agent_uuid}) with argument_list {boolV, integer, double, string, data} and token '{token}''")

    def send_values(self, sender_agent_name, sender_agent_uuid, token, my_data):
        print(f"Service send_values called by {sender_agent_name} ({sender_agent_uuid}), token '{token}' sending values : {self.on_off_o, self.integerO, self.doubleO, self.stringO, self.dataO}")
        igs.info(sender_agent_uuid, "receive_values", (self.on_off_o, self.integerO, self.doubleO, self.stringO, self.dataO), token)