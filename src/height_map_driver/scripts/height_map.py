import numpy as np

class HeightMap:

    def __init__(self, depth_map: np.array, boom_height: int):
        self.depth_map = depth_map
        self.boom_height = boom_height

    def run(self) -> str:
        return self.depth_map + self.boom_height
