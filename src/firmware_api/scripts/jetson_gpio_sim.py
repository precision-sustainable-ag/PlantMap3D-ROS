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
channels = {}

def setmode(mode):
    global mode_set
    print("Running using simulation mode")
    mode_set = True

def setup(channel, direction):
    global mode_set, channels
    if not mode_set:
        raise RuntimeError("Pinmode not set")
    else:
        channels[channel]=direction

def input(channel):
    global mode_set, channels
    if not mode_set:
        raise RuntimeError("Pinmode not set")
    else:
        raw_value = "a"
        while raw_value not in "01":
            raw_value = builtins.input("Enter boolean (0 or 1) value of pin {:d} ".format(channel))
        return bool(int(raw_value))
