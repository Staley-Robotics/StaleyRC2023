import ctre
from wpilib import MotorControllerGroup


class Drivetrain:
    EASE_VALUE = 0.0175
    last_speed = 0
    last_rotation = 0

    def __init__(self):
        self.left = MotorControllerGroup(ctre.WPI_VictorSPX(1), ctre.WPI_VictorSPX(2))
        self.right = MotorControllerGroup(ctre.WPI_VictorSPX(3), ctre.WPI_VictorSPX(4))
        # self.mod = 0

    def drive(self, speed: float, rotation: float, variableSpeed: float):
        # if abs(variableSpeed - 1) > 1:
        #     self.mod = 1
        # else:
        #     self.mod = self.mod
        # speed *= self.mod

        speed *= 0.8

        if speed > self.last_speed + self.EASE_VALUE and self.last_speed < speed:
            speed = self.last_speed + self.EASE_VALUE
        if speed < self.last_speed - self.EASE_VALUE and self.last_speed > speed:
            speed = self.last_speed - self.EASE_VALUE
        self.last_speed = speed

        rotation *= 0.4

        self.left.set(rotation - speed)
        self.right.set(rotation + speed)
