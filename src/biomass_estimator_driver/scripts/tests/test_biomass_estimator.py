from unittest import TestCase
import numpy as np
from biomass_estimator import BiomassEstimator

class TestBiomassEstimator(TestCase):

    def test_run(self):
        test_input_1 = np.ones([2, 3])
        test_input_2 = np.ones([2, 3])
        bio_est = BiomassEstimator(semantic_array=test_input_1, heights_array=test_input_2)
        biomass_results = bio_est.run()
        self.assertEqual(biomass_results['AMATU'], 123.45)
