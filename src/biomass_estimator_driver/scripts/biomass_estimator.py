import numpy as np
import json
import os
os.chdir(os.path.dirname(os.path.abspath(__file__)))

def biomassCorrelation(pix_count: float) -> float:
    return pix_count


class BiomassEstimator:
    
    def __init__(self, semantic_array: np.array, heights_array: np.array):
        self.semantic_array = semantic_array
        self.heights_array = heights_array
        # Opening JSON file
        with open("config/species_list.json", 'r') as species_file:
            self.species_data = json.load(species_file)
        self.num_categories = len(self.species_data["species"])
        self.biomass_estimates = np.zeros(self.num_categories)


    def run(self) -> np.array:
        for i in range(1, self.num_categories):
            self.biomass_estimates[i] = biomassCorrelation(np.sum(self.semantic_array == i))
        return self.biomass_estimates
