from typing import List


class Obstacle:
    geometry: List[float]

    def __init__(self, geometry: List[float]):
        self.geometry = geometry
