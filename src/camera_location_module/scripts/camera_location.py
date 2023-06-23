#!/usr/bin/env python3
import yaml
import utm
import numpy as np
from math import cos, sin, radians
import os
os.chdir(os.path.dirname(os.path.abspath(__file__)))

def camera_vector_heading_correction(r_c_t_t: np.array, heading: float) -> np.array:
    heading_trans = radians(-1*heading)
    rotation_matrix = np.array([[cos(heading_trans), -1*sin(heading_trans)], [sin(heading_trans), cos(heading_trans)]])
    r_c_t_w = rotation_matrix.dot(np.asarray(r_c_t_t))
    return r_c_t_w

def translate_gps_to_camera(gps_coordinates_latlon: list, r_c_t_w: np.array) -> list:
    easting, northing, zone_number, zone_letter = utm.from_latlon(gps_coordinates_latlon[0], gps_coordinates_latlon[1])
    easting += r_c_t_w[0]
    northing += r_c_t_w[1]
    lat, long = utm.to_latlon(easting, northing, zone_number, zone_letter)

    return [float(lat), float(long)]

# Use this function.
def find_camera_gps_coordinates(gps_coordinates: list, heading: float, camera_id: int, cam_config_path:str) -> list:
    with open(cam_config_path, 'r') as file:
        camera_data = yaml.safe_load(file)
    r_c_t_t = np.array([camera_data['camera_displacement_right'][camera_id - 1], camera_data['camera_displacement_forward']])
    r_c_t_w = camera_vector_heading_correction(r_c_t_t, heading)
    return translate_gps_to_camera(gps_coordinates, r_c_t_w)

