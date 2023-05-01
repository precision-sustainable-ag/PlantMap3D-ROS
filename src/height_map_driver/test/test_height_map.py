from unittest import TestCase
import numpy as np
from height_map import HeightMap

class TestHeightMap(TestCase):
    def test_run(self):
        test_input_1 = np.ones([2, 3])
        test_input_2 = 1
        hm = HeightMap(depth_map=test_input_1, boom_height=test_input_2)
        hm_results = hm.run()
        self.assertEqual(hm_results[0][0], 2.0)
