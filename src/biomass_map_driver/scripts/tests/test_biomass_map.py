from unittest import TestCase
from biomass_map import BiomassMap
import PIL

class TestBiomassMap(TestCase):

    def test_run(self):
        test_input_1 = {'AMAPA': 0.0, 'AMATU': 123.45, 'ECHCG': 234.56, 'PROLO': 0.0, 'ZEAMX': 0.0, 'GLXMA': 0.0}
        test_input_2 = (3.456, 4.567)
        bio_map = BiomassMap(biomass_estimates=test_input_1, gps_location=test_input_2)
        map_results = bio_map.run()
        print(type(map_results))
        self.assertEqual(type(map_results), PIL.Image.Image)
