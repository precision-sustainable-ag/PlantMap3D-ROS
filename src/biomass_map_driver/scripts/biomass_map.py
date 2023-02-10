from typing import Tuple
from PIL import Image
import numpy as np

class BiomassMap:

    def __init__(self, biomass_estimates: np.ndarray, gps_location: Tuple[float, float]):
        self.biomass_estimates = biomass_estimates
        self.gps_location = gps_location
        self.map_image = Image.fromarray(np.zeros([1024, 1024, 3], dtype=np.uint8))
        self.map_image.convert('RGB')

    def run(self) -> Image.Image:
        return self.map_image
