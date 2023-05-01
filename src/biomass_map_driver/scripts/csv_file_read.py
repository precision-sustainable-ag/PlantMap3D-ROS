#!/usr/bin/env python3
import roslib; roslib.load_manifest('oakd_camera_driver')
roslib.load_manifest('camera_trigger_driver')
import rospy
import rospkg
import datetime, pytz
from os import path

import csv 
rospy.init_node('testing')
rospack_testset = rospkg.RosPack()
#ADD the actual path here, but keep the date-based filename
__biomass_summary_path= rospack_testset.get_path('data_saver_driver') + '/biomass_estimation/biomass_save_summary/'
__summary_filename = "test_file.csv"
summary_complete_path_name = __biomass_summary_path + __summary_filename

if(not path.exists(summary_complete_path_name)):
    biomass_header = ["im_filename", "latitude", "longitude", "grass_pixels", "grass_biomass", "clover_pixels", "clover_biomass", "brassica_pixels", "brassica_biomass", "weed_pixels", "weed_biomass", "total_vegetation_pixels", "total_biomass"]
    fd = open(summary_complete_path_name,'w', newline='')
    fd.write((';'.join(map(str, biomass_header)))+'\r\n')
    fd.close()

im_filename = "test.jpg"
latitude = -39.1
longitude = 71.2
grass_pixels = 1
grass_biomass = 2
clover_pixels = 3
clover_biomass = 4
brassica_pixels = 5
brassica_biomass = 6
weed_pixels = 7
weed_biomass = 8
total_vegetation_pixels = 9
total_biomass = 10


biomass_data = [im_filename, latitude, longitude, grass_pixels, grass_biomass, clover_pixels, clover_biomass, brassica_pixels, brassica_biomass, weed_pixels, weed_biomass, total_vegetation_pixels, total_biomass]
fd = open(summary_complete_path_name,'a', newline='')
fd.write((';'.join(map(str, biomass_data)))+'\r\n')
fd.close()

biomass_data = [im_filename, latitude + 10, longitude + 10, grass_pixels + 10, grass_biomass + 10, clover_pixels, clover_biomass, brassica_pixels, brassica_biomass, weed_pixels, weed_biomass, total_vegetation_pixels, total_biomass]
fd = open(summary_complete_path_name,'a', newline='')
fd.write((';'.join(map(str, biomass_data)))+'\r\n')
fd.close()

with open(summary_complete_path_name,'r') as csvfile:
    contents = csvfile.read()

last_line = contents.strip().split('\n')[-1]

last_rows = list(csv.reader([last_line]))[0]

print(last_rows)