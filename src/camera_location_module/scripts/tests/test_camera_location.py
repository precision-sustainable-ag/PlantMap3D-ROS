from unittest import TestCase
import numpy as np
from camera_location import find_camera_gps_coordinates, translate_gps_to_camera, camera_vector_heading_correction




class TestCameraLocation(TestCase):

    def test_camera_vector_function_should_return_nparray_with_2_npfloats(self):
        result = camera_vector_heading_correction(np.array([0.0, 0.0]), 0.0)
        self.assertEqual(type(result), np.ndarray)
        self.assertEqual(type(result[0]), np.float64)
        self.assertEqual(type(result[1]), np.float64)
        self.assertEqual(len(result), 2)

    def test_camera_vector_function_should_return_input_if_heading_0(self):
        result = camera_vector_heading_correction(np.array([1.0, 2.0]), 0.0)
        self.assertAlmostEqual(result[0], 1.0, 1)
        self.assertAlmostEqual(result[1], 2.0, 1)

    def test_camera_vector_function_should_return_90_degree_rotation(self):
        result = camera_vector_heading_correction(np.array([0.0, -1.0]), 90.0)
        self.assertAlmostEqual(result[0], -1.0, 1)
        self.assertAlmostEqual(result[1], 0.0, 1)

    def test_find_camera_gps_coordinates_should_retun_2_floats_in_a_list(self):
        pin1 = [39.338800, -96.830578]
        #pin2 = [39.338200, -96.830578]
        #true_distance = 66.72  # meters
        gps_coordinates = pin1
        heading = 0
        camera_coord = find_camera_gps_coordinates(gps_coordinates=gps_coordinates, heading=heading, camera_id=0, cam_config_path="test_config.yaml")
        self.assertEqual(type(camera_coord[0]), float)
        self.assertEqual(type(camera_coord[1]), float)
        self.assertEqual(type(camera_coord), list)

    def test_translate_gps_to_camera_should_return_2_floats_in_a_list(self):
        camera_latlong = translate_gps_to_camera(gps_coordinates_latlon=[51.2, 7.5], r_c_t_w=np.array([0, 0]))
        self.assertEqual(type(camera_latlong[0]), float)
        self.assertEqual(type(camera_latlong[1]), float)
        self.assertEqual(type(camera_latlong), list)

    def test_should_show_camera_66m_behind_is_point0006_degrees_away(self):
        pin1 = [39.338800, -96.830578]
        pin2 = [39.338200, -96.830578]
        camera_coord = find_camera_gps_coordinates(gps_coordinates=pin1, heading=0, camera_id=0,
                                                   cam_config_path="test_config.yaml")
        self.assertAlmostEqual(camera_coord[0], pin2[0], 4)
        self.assertAlmostEqual(camera_coord[1], pin2[1], 4)




