from unittest import TestCase
from biomass_map import BiomassMap
import PIL
import numpy as np

class TestBiomassMap(TestCase):

    def test_run(self):
        test_input_1 = np.array([0.0, 123.45, 234.56, 0.0, 0.0, 0.0])
        test_input_2 = (3.456, 4.567)
        bio_map = BiomassMap(biomass_estimates=test_input_1, gps_location=test_input_2)
        map_results = bio_map.run()
        print(type(map_results))
        self.assertEqual(type(map_results), PIL.Image.Image)
