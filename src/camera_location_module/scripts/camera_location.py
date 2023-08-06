#!/usr/bin/env python3
import yaml
import utm
import numpy as np
from math import cos, sin, radians

# Convert the vector from the tractor to the camera from a tractor reference frame into a world/utm reference frame by
# rotating the vector based on the heading of the tractor.
def camera_vector_heading_correction(r_c_t_t: np.array, heading: float) -> np.array:
    # Negate the heading value, since a positive heading change from North goes clockwise, however a positive right-hand
    # rule rotation about the z-axis is anti-clockwise.
    heading_trans = radians(-1*heading)

    # Implementation of a 2D rotation matrix: https://en.wikipedia.org/wiki/Rotation_matrix
    rotation_matrix = np.array([[cos(heading_trans), -1*sin(heading_trans)], [sin(heading_trans), cos(heading_trans)]])

    # Applying this rotation matrix on the vector to the camera from the tractor.
    r_c_t_w = rotation_matrix.dot(np.asarray(r_c_t_t))
    return r_c_t_w

# A function to move the gps coordiates by a distance in meters
def translate_gps_to_camera(gps_coordinates_latlon: list, r_c_t_w: np.array) -> list:
    # Convert the lat-long coordinates into the utm coordinate system
    easting, northing, zone_number, zone_letter = utm.from_latlon(gps_coordinates_latlon[0], gps_coordinates_latlon[1])

    # Add the vector to the camera from the tractor to the gps coordinates in the same utm world reference frame.
    easting += r_c_t_w[0]
    northing += r_c_t_w[1]

    # Convert the camera location back into lat-long coordinate system
    lat, long = utm.to_latlon(easting, northing, zone_number, zone_letter)
    return [float(lat), float(long)]

# Use this function.
# How to interpret 4 character variables used in this function:
# The first letter r just refers to this variable being a vector. Not sure why the convention is r rather than v.
# The second letter is what the vector is pointing to: I chose c to stand for camera in this case.
# The third letter is where the vector starts: I chose t to stand for tractor in this case.
# The fourth letter is the reference frame that the vector is in: tractor space or world/utm space.
#
# To further explain the reference frame:
# Tractor space has its origin at the GPS unit on the tractor, or some other arbitrary point on the tractor. The x-axis
# of this reference frame points out the right window of the tractor. No matter how the tractor moves, the vector
# from the tractor to a camera will never change (unless the boom is raised or lowered). The world/utm space moves the
# origin point off the tractor to some arbitrary point on the farm...perhaps the beginning of the first row and will
# never move. The x- and y-axis are aligned with East and North respectively in alignment with the utm global coordinate
# system.
def find_camera_gps_coordinates(gps_coordinates: list, heading: float, camera_id: int, cam_config_path:str) -> list:
    with open(cam_config_path, 'r') as file:
        camera_data = yaml.safe_load(file)

    # Get the vector from the tractor to the camera w.r.t. the tractor
    r_c_t_t = np.array([camera_data['camera_displacement_right'][camera_id], camera_data['camera_displacement_forward']])

    # Transform the vector from the tractor to the camera into world coordinates, which will rotate the vector depending
    # on the heading of the tractor from North.
    r_c_t_w = camera_vector_heading_correction(r_c_t_t, heading)

    # shift the lat-long coordinates of the gps receiver unit to the camera.
    return translate_gps_to_camera(gps_coordinates, r_c_t_w)

