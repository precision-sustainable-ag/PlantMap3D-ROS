import numpy as np
from typing import Dict

class BiomassEstimator:

    def __init__(self, semantic_array: np.array, heights_array: np.array):
        self.semantic_array = semantic_array
        self.heights_array = heights_array
        self.biomass_estimates = {'AMAPA': 0.0, 'AMATU': 0.0, 'ECHCG': 0.0, 'PROLO': 0.0, 'ZEAMX': 0.0, 'GLXMA': 0.0}

    def run(self) -> Dict:
        self.biomass_estimates['AMATU'] = 123.45
        self.biomass_estimates['ECHCG'] = 234.56
        return self.biomass_estimates


