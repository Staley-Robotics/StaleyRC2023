class AprilTag:
    tag_id: int
    x: float
    y: float
    z: float
    face: bool

    def __init__(self, tag_id: int, x: float, y: float, z: float, face: bool) -> None:
        self.tag_id = tag_id
        self.x = x
        self.y = y
        self.z = z
        self.face = face

