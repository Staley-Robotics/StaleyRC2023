import ctre


class Drivetrain:

    EASE_VALUE = 0.0175
    last_speed = 0
    last_rotation = 0

    def __init__(self):
        self.front_left = ctre.WPI_VictorSPX(1)
        self.back_left = ctre.WPI_VictorSPX(2)
        self.back_right = ctre.WPI_VictorSPX(3)
        self.front_right = ctre.WPI_VictorSPX(4)

    def drive(self, speed: float, rotation: float):
        speed *= 0.5
        if speed > self.last_speed + self.EASE_VALUE and self.last_speed < speed:
            speed = self.last_speed + self.EASE_VALUE
        if speed < self.last_speed - self.EASE_VALUE and self.last_speed > speed:
            speed = self.last_speed - self.EASE_VALUE
        self.last_speed = speed
        rotation *= 0.2
        # if rotation > self.last_rotation + self.EASE_VALUE and self.last_rotation < rotation:
        #     rotation = self.last_rotation + self.EASE_VALUE
        # if rotation < self.last_rotation - self.EASE_VALUE and self.last_rotation > rotation:
        #     rotation = self.last_rotation - self.EASE_VALUE
        # self.last_rotation = rotation
        self.front_left.set(rotation - speed)
        self.front_right.set(rotation + speed)
        self.back_left.set(rotation - speed)
        self.back_right.set(rotation + speed)
