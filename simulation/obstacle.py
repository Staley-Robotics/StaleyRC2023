import numpy as np


class Obstacle:
    geometry: np.array

    def __init__(self, geometry: np.array):
        self.geometry = geometry
