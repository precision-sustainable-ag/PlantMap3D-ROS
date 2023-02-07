import numpy as np
from typing import Dict

class BiomassEstimator:

    def __init__(self, semantic_array: np.array, heights_array: np.array):
        self.semantic_array = semantic_array
        self.heights_array = heights_array
        self.biomass_estimates = {'AMAPA': 0, 'AMATU': 0, 'ECHCG': 0, 'PROLO': 0, 'ZEAMX': 0, 'GLXMA': 0}

    def run(self) -> Dict:

        for i in range(np.shape(self.semantic_array)[0]):
            for j in range(np.shape(self.semantic_array)[1]):
                if self.semantic_array[i][j] == 0:
                    self.biomass_estimates['AMAPA'] += 1
                elif self.semantic_array[i][j] == 1:
                    self.biomass_estimates['AMATU'] += 1
                elif self.semantic_array[i][j] == 2:
                    self.biomass_estimates['ECHCG'] += 1

        return self.biomass_estimates


