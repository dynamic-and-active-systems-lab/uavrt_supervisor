#!/usr/bin/env python3
#
# Codebase for the Supervisor package used within the UAV-RT architecture.
# Copyright (C) 2023 Dynamic and Active Systems Lab
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


class SubprocessConstants(Enum):
    # Queue size is a required QoS (quality of service) setting that limits the
    # amount of queued messages if a subscriber is not receiving them fast enough.
    QUEUE_SIZE = 10
    # Rate at which status timer messages will be checked and published in seconds.
    STATUS_TIMER_MESSAGE_PUBLISH_RATE = .5
    # Sampling rate used by airspy_rx to determine the rate of which samples
    # are collected.
    RADIO_SAMPLING_RATE = 3000000
    # Default channelizer sampling rate
    CHANNELIZER_SAMPLING_RATE = 375000
    # Default channelizer decimation rate
    CHANNELIZER_DECIMATION_RATE = 100


class TunerOutputConstants(Enum):
    RADIO_CENTER_FREQENCY = 0
    CENTER_CHANNEL_FREQUENCIES = 1
    TAG_CHANNEL_NUMBER = 2


class DiagnosticStatusIndicesControl(Enum):
    DIAGNOSTIC_STATUS = 0


class KeyValueIndicesControl(Enum):
    CENTER_FREQUENCY = 0
    DETECTOR_CONFIG_PATH = 1


class AirspyCSDRNetcatComponentSubprocessDictionary(Enum):
    AIRSPY_CSDR_NETCAT_SUBPROCESS = 0


class ChannelizerSubprocessDictionary(Enum):
    CHANNELIZER_SUBPROCESS = 0


class DetectorSubprocessDictionary(Enum):
    DETECTOR_SUBPROCESS = 0
