from typing import List

from apriltag import AprilTag


class Map:
    apriltags: List[AprilTag]

    def __init__(self):
        # Field measurements are in inches
        self.apriltags = [
            AprilTag(1, 610.77, 42.19, 18.22, True),
            AprilTag(2, 610.77, 108.19, 18.22, True),
            AprilTag(3, 610.77, 174.19, 18.22, True),
            AprilTag(4, 636.96, 265.74, 27.38, True),
            AprilTag(5, 14.25, 265.74, 27.38, False),
            AprilTag(6, 40.45, 174.19, 18.22, False),
            AprilTag(7, 40.45, 108.19, 18.22, False),
            AprilTag(8, 40.45, 42.19, 18.22, False)
        ]
