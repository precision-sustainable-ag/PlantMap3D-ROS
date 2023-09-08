#!/usr/bin/env python3

import platform
if platform.system()=="Linux" and "20.04" in platform.release():
    import Jetson.GPIO as GPIO
else:
    import jetson_gpio_sim as GPIO
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
        self.format_states()
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
                    GPIO.setup(pins_to_assign[pin_name],GPIO.IN)
                    self.read_pins[get_pin_holder_index(pin_name)]=pins_to_assign[pin_name]
                elif "IN" in pin_name:
                    GPIO.setup(pins_to_assign[pin_name],GPIO.OUT)
                    self.write_pins[get_pin_holder_index(pin_name)]=pins_to_assign[pin_name]
        else:
            print("Error. No pins found for assignment. ")
    def format_states(self):
        def state_from_raw(state_with_ignore):
            holder = ""
            input_state = str(state_with_ignore)
            for char in input_state:
                if char == "*":
                    holder += "0"
                else:
                    holder += char
            return holder
        def mask_from_raw(state_with_ignore):
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
        defined_states = self.parameter_dictionary.get("states")
        self.binary_states = {}
        self.binary_masks = {}
        if defined_states is not None:
            for state in defined_states.keys():
                self.binary_states[int(state_from_raw(state),2)] = defined_states[state]
                self.binary_masks[int(mask_from_raw(state),2)] = defined_states[state]
    def read_state(self):
        raw_pin_data = []
        counter = 0
        for pin in self.read_pins:
            if pin != -1:
                raw_pin_data.append(str(int(GPIO.input(int(pin)))))
            counter += 1
        #The output of this is ordered from [LSB,...,MSB],
        #so reverse the array to get a standard binary arrangement
        #from MSB on the left to LSB on the right. 
        raw_pin_data.reverse()
        current_state = int("".join(raw_pin_data),2)
        self.command = ""
        for mask in self.binary_masks.keys():
            masked_state = current_state & mask
            self.command = self.binary_states.get(masked_state)



if __name__ == "__main__":
    board = SMB_Interface("../params/")
    board.initialize_GPIO()
    board.format_states()
    board.read_state()
    print(board.command)