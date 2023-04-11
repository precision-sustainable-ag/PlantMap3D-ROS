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
        self._species_data = species_list_path
        try :

            if not os.path.exists(self._species_data):
                raise FileNotFoundError(f"Species list not found at path : {self._species_data}")
            else:
                with open(self._species_data, 'r') as species_file:
                    self._species_data = json.load(species_file)

        except FileNotFoundError as e:
                print(e) 
     
        self.num_categories = len(self._species_data["species"])
        self.map_image = Image.fromarray(np.zeros([1024, 1024, 3], dtype=np.uint8))
        self.map_image.convert('RGB')
     

    def run(self) -> Image.Image:
        return self.map_image
