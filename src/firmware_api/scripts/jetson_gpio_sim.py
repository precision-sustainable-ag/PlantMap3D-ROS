#!/usr/bin/env python3

#Daniel Yanke

import builtins

BOARD = 10
HIGH = 1
LOW = 0

OUT = 0
IN = 1

_PUD_OFFSET = 20
PUD_OFF = 0 + _PUD_OFFSET
PUD_DOWN = 1 + _PUD_OFFSET
PUD_UP = 2 + _PUD_OFFSET

mode_set = False
channelsall = {}

def setmode(mode):
    global mode_set
    print("Running using simulation mode")
    mode_set = True

def setup(channel, direction):
    global mode_set, channelsall
    if not mode_set:
        raise RuntimeError("Pinmode not set")
    else:
        channelsall[channel]=direction

def input(channel):
    global mode_set, channelsall
    if not mode_set or channelsall[channel] != IN:
        raise RuntimeError("Pinmode not set")
    else:
        raw_value = "a"
        while raw_value not in "01":
            raw_value = builtins.input("Enter boolean (0 or 1) value of pin {:d} ".format(channel))
        return bool(int(raw_value))

def output(channels,values):
    global channelsall
    if type(channels) != list: channels = [channels]
    if type(values) != list: values = [values]
    def format_bool(input_char):
        if input_char == 0: return "False"
        else: return "True"
    if len(channels) != len(values):
        raise RuntimeError("Number of output pins not equal to number of values to write")
    if any(channelsall[channel] != OUT for channel in channels):
        raise RuntimeError("Attempted to write to a pin not configured as output. ")
    for channel,value in zip(channels,values):
        print("Assigned pin "+str(channel)+" to "+format_bool(value))