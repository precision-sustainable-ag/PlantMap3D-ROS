#!/usr/bin/env python3

#Daniel Yanke
#9/19/23 best Docker command
#sudo docker run --name test -it --device /dev/gpiochip0 -v /sys:/sys -v /proc:/proc 362b9486c282
#nano /usr/local/lib/python3.8/dist-packages/Jetson/GPIO/gpio_pin_data.py
#add "return JETSON_ORIN" on line 546 

simulator = False
import platform
if platform.system()=="Linux" and "tegra" in platform.release():
    import Jetson.GPIO as GPIO
else:
    import jetson_gpio_sim as GPIO
    simulator = True
import os
import time

class SMB_Interface():
    def __init__(self, parameter_path):
        #There are empty holders for 6 pin numbers because there are a total of 6
        #pins in this interface. In theory, it could be necessary to swap directions. 
        self.read_pins, self.write_pins = [-1,-1,-1,-1,-1,-1],[-1,-1,-1,-1,-1,-1]
        self.parameter_directory = os.path.abspath(os.path.join(os.path.dirname(__file__),parameter_path))
        self.read_parameters()
        GPIO.setmode(GPIO.BOARD)
        self.initialize_GPIO()
    def read_parameters(self):
        self.parameter_files = os.listdir(self.parameter_directory)
        self.parameter_dictionary = {}
        for each in self.parameter_files:
            file = open(os.path.join(self.parameter_directory,each))
            dictionary_holder = {}
            #Filter out comments from parameter file (lines with "#" near the beginning
            #of the line.)
            raw_file = [x.strip() for x in file.readlines() if "#" not in x[0:3]]
            for line in raw_file:
                parsing_holder = line.split(",")
                if each == "pins.csv" and parsing_holder[1].isnumeric():
                    dictionary_holder[parsing_holder[0]]=int(parsing_holder[1])
                elif each == "pins.csv":
                    print("Skipping invalid pins parameter line \""+line+"\"")
                else:
                    dictionary_holder[parsing_holder[0]]=parsing_holder[1]
            self.parameter_dictionary[each.split(".")[0]]=dictionary_holder
        self.in_bin_masks,self.in_bin_states = self.format_states("in_states")
        self.out_bin_masks,self.out_bin_states = self.format_states("out_states")
    def initialize_GPIO(self):
        def get_pin_holder_index(pin_name):
            counter = 0
            indexes = []
            numeric_holder = ""
            for index in range(1,len(pin_name)):
                character = pin_name[index]
                if character.isnumeric():
                    counter+=1
                    indexes.append(int(character))
                    numeric_holder += character
            indexes.sort()
            if (indexes[-1]-indexes[0]>=counter):
                print("Error reading pin name. Are there multiple numeric designations?")
                return 0
            else:
                return int(numeric_holder)
        pins_to_assign = self.parameter_dictionary.get("pins")
        if pins_to_assign is not None:
            for pin_name in pins_to_assign.keys():
                #Since pin direction names are defined from the perspective of the Pico,
                #the "IN" pins need to be output pins on the Jetson side. 
                if "OUT" in pin_name:
                    GPIO.setup(int(pins_to_assign[pin_name]),GPIO.IN)
                    self.read_pins[get_pin_holder_index(pin_name)]=pins_to_assign[pin_name]
                elif "IN" in pin_name:
                    GPIO.setup(int(pins_to_assign[pin_name]),GPIO.OUT)
                    self.write_pins[get_pin_holder_index(pin_name)]=pins_to_assign[pin_name]
            self.write_pin_status = [0 for x in self.write_pins if x != -1]
        else:
            print("Error. No pins found for assignment. ")
    def state_from_raw(self,state_with_ignore):
        holder = ""
        input_state = str(state_with_ignore)
        for char in input_state:
            if char == "*":
                holder += "0"
            else:
                holder += char
        return holder
    def mask_from_raw(self,state_with_ignore):
        holder = ""
        input_state = str(state_with_ignore)
        for char in input_state:
            if char == "*":
                holder += "0"
            elif (char == "0") or (char == "1"):
                holder += "1"
            else:
                holder += char
        return holder
    def format_states(self,target_file):
        defined_states = self.parameter_dictionary.get(target_file)
        binary_states = {}
        binary_masks = {}
        if defined_states is not None:
            for state in defined_states.keys():
                binary_states[int(self.state_from_raw(state),2)] = defined_states[state]
                binary_masks[int(self.mask_from_raw(state),2)] = defined_states[state]
        return binary_masks,binary_states
    def read_state(self,simulator=False):
        raw_pin_data = []
        if simulator:
            print("Jetson Input Pins are " + str([x for x in self.read_pins if x != -1][::-1]) + " MSB to LSB")
        #The output of read_pins is ordered from [LSB,...,MSB],
        #so reverse the array first to get a standard binary arrangement
        #from MSB on the left to LSB on the right and request from MSB to LSB (if in sim mode).
        for pin in self.read_pins[::-1]:
            if pin != -1:
                raw_pin_data.append(str(int(GPIO.input(int(pin)))))
        current_state = int("".join(raw_pin_data),2)
        self.status = ""
        status_options = []
        for mask in self.in_bin_masks.keys():
            masked_state = current_state & mask
            status_holder = self.in_bin_states.get(masked_state)
            if status_holder is not None:
                status_options.append(status_holder)
        if len(status_options)>1:
            print("WARNING! Multiple State Conditions Found. Choosing First: " + status_options[0])
        if len(status_options) != 0:
            board.status = status_options[0]
        else:
            board.status = "Unknown"

    def write_to_pins(self):
        holder = list(zip(self.write_pins[len(self.write_pin_status)-1::-1],self.write_pin_status))
        for pin_assignment in holder:
            GPIO.output(int(pin_assignment[0]),int(pin_assignment[1]))

    def lights(self,value):
        light_state = [x for x in self.out_bin_states.keys() if "light" in self.out_bin_states.get(x).lower()]
        if len(light_state)>1:
            raise ValueError("Multiple pin states defined as light controls")
        light_state = light_state[0] | int("".join([str(x) for x in self.write_pin_status]))
        light_state = "{:3b}".format(light_state)
        for i in range(0,len(self.write_pin_status)):
            self.write_pin_status[i] = int(light_state[i])
        self.write_to_pins()

    def shutdown(self):
        sd_state = [x for x in self.out_bin_states.keys() if "shutdown" in self.out_bin_states.get(x).lower()]
        if len(sd_state)>1:
            raise ValueError("Multiple pin states defined as light controls")
        sd_state = sd_state[0] | int("".join([str(x) for x in self.write_pin_status]))
        sd_state = "{:3b}".format(sd_state)
        for i in range(0,len(self.write_pin_status)):
            self.write_pin_status[i] = int(sd_state[i])
        self.write_to_pins()

if __name__ == "__main__":
    board = SMB_Interface("../params/")
    board.write_to_pins()
    board.shutdown()
    board.lights(True)
    board.read_state(simulator)
    print(board.status)