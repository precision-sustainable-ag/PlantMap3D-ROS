from unittest import TestCase
import numpy as np
from biomass_estimator import BiomassEstimator

class TestBiomassEstimator(TestCase):

    def setUp(self):
        self.test_input_1 = np.ones([2, 3])
        self.test_input_2 = np.ones([2, 3])

    def test_should_output_a_dict(self):
        bio_est = BiomassEstimator(semantic_array=self.test_input_1, heights_array=self.test_input_2)
        biomass_results = bio_est.run()
        self.assertEqual(type(biomass_results), dict)

    def test_should_sum_to_five_AMATU(self):
        self.test_input_1 = np.ones([2, 3])
        self.test_input_1[0][0] = 0
        bio_est = BiomassEstimator(semantic_array=self.test_input_1, heights_array=self.test_input_2)
        biomass_results = bio_est.run()
        self.assertEqual(biomass_results['AMATU'], 5)

