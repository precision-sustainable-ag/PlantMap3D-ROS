# -*- coding: utf-8 -*-
"""
Created on Fri Apr 28 19:53:57 2023

@author: skovsen
"""
import datetime, pytz
from os import path
current_time = datetime.datetime.now(pytz.timezone('US/Eastern'))

summary_directory = "" #ADD the actual path here, but keep the date-based filename
summary_filename = "biomass_data_" + str(current_time.year) + "-" + str(current_time.month) + "-" + str(current_time.day) + ".csv"
summary_path_and_name = summary_directory + "/" + summary_filename 

if(not path.exists(summary_path_and_name)):
    biomass_header = ["im_filename", "latitude", "longitude", "grass_pixels", "grass_biomass", "clover_pixels", "clover_biomass", "brassica_pixels", "brassica_biomass", "weed_pixels", "weed_biomass", "total_vegetation_pixels", "total_biomass"]
    fd = open(summary_path_and_name,'w', newline='')
    fd.write((';'.join(map(str, biomass_header)))+'\r\n')
    fd.close()
# Test input data directly below - should be replaced with something from ros
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
fd = open(summary_path_and_name,'a', newline='')
fd.write((';'.join(map(str, biomass_data)))+'\r\n')
fd.close()