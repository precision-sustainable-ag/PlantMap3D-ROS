import numpy as np
from typing import Dict

class BiomassEstimator:

    def __init__(self, semantic_array: np.array, heights_array: np.array):
        self.semantic_array = semantic_array
        self.heights_array = heights_array
        self.biomass_estimates = {'AMAPA': 0, 'AMATU': 0, 'ECHCG': 0, 'PROLO': 0, 'ZEAMX': 0, 'GLXMA': 0}

    def run(self) -> Dict:
        i = 0
        for key in self.biomass_estimates:
            self.biomass_estimates[key] = np.sum(self.semantic_array == i)
            i += 1
        return self.biomass_estimates
