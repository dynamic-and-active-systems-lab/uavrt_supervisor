#!/usr/bin/env python3
#
# Codebase for the Supervisor package used within the UAV-RT architecture.
# Copyright (C) 2022 Dynamic and Active Systems Lab
#
# This program is free software: you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation, either version 3 of the License, or (at your option) any later
# version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of  MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License along with
# this program.  If not, see <http://www.gnu.org/licenses/>

# https://docs.python.org/3/library/enum.html
from enum import Enum

class SystemControl(Enum):
    STOP_ALL_SUBPROCESS = 0

class DiagnosticStatusIndiceControl(Enum):
    DIAGNOSTIC_STATUS = 0

class KeyValueIndicesControl(Enum):
    CENTER_FREQUENCY = 0
    SAMPLE_RATE = 1

class NetcatAirspyhfSubprocessDictionary(Enum):
    CENTER_FREQUENCY = 0
    SAMPLE_RATE = 1
    NETCAT_AIRSPYHF_SUBPROCESS = 2

class AirspyhfChannelizeSubprocessDictionary(Enum):
    AIRSPYHF_CHANNELIZE_SUBPROCESS = 0
