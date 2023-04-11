from typing import Tuple
from PIL import Image
import numpy as np
import json
import os
os.chdir(os.path.dirname(os.path.abspath(__file__)))

class BiomassMap:

    def __init__(self, biomass_estimates: np.ndarray, gps_location: Tuple[float, float],species_list_path:str):
        self.biomass_estimates = biomass_estimates
        self.gps_location = gps_location
        # Opening the species list
        with open(species_list_path, 'r') as species_file:
            self.species_data = json.load(species_file)
        self.num_categories = len(self.species_data["species"])
        self.map_image = Image.fromarray(np.zeros([1024, 1024, 3], dtype=np.uint8))
        self.map_image.convert('RGB')

    def run(self) -> Image.Image:
        return self.map_image
