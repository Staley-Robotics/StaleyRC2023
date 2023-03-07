import math
from enum import IntEnum

from ctre import *
from ctre.sensors import *

from drivetrain.chassis import Chassis
from tools import *


class Motor(IntEnum):
    BRT = 1
    BRD = 2
    BLT = 3
    BLD = 4
    FRT = 5
    FRD = 6
    FLT = 7
    FLD = 8


class Swerve(Chassis):

    speed: float
    rotation: float
    direction: float
    drive_left: tuple[WPI_TalonFX, WPI_TalonFX]
    drive_right: tuple[WPI_TalonFX, WPI_TalonFX]
    swivels: tuple[WPI_TalonFX, WPI_TalonFX, WPI_TalonFX, WPI_TalonFX]
    swivel_encoders: tuple[WPI_CANCoder, WPI_CANCoder, WPI_CANCoder, WPI_CANCoder]
    swivel_offset: tuple[int, int, int, int] = (214.453, 245.391, 223.242, 223.066)
    full_rotation: Tuple[int, int, int, int] = (225, 315, 135, 45)
    k_gains: Gains = Gains(0.03, 0.0, 0.0, 0.0, 0, 1.0)

    def __init__(self, pipeline: PipelineManager):
        super().__init__(pipeline)

        self.speed = 0.0
        self.rotation = 0.0
        self.direction = 0.0

        self.talon_fx_resolution *= 10

        self.drive_left = (WPI_TalonFX(Motor.BLT, "canivore"), WPI_TalonFX(Motor.FLT, "canivore"))

        self.drive_right = (WPI_TalonFX(Motor.BRT, "canivore"), WPI_TalonFX(Motor.FRT, "canivore"))

        self.swivels = (WPI_TalonFX(Motor.BRD, "canivore"), WPI_TalonFX(Motor.BLD, "canivore"),
                        WPI_TalonFX(Motor.FRD, "canivore"), WPI_TalonFX(Motor.FLD, "canivore"))

        self.swivel_encoders = (WPI_CANCoder(Motor.FLD + 10, "canivore1"), WPI_CANCoder(Motor.FRD + 10, "canivore1"),
                                WPI_CANCoder(Motor.BLD + 10, "canivore1"), WPI_CANCoder(Motor.BRD + 10, "canivore1"))

        self.drive_right[0].setInverted(True)
        self.drive_right[1].setInverted(True)

        for p in range(len(self.swivels)):
            pinion: WPI_TalonFX = self.swivels[p]
            encoder: WPI_CANCoder = self.swivel_encoders[p]
            pinion.configFactoryDefault()
            encoder.configFactoryDefault()
            encoder.setPosition(encoder.getAbsolutePosition() - self.swivel_offset[p])

    def goto(self, x: float, y: float) -> bool:
        self.pipeline.drive_constant(0.5)
        self.pipeline.direction_x_constant(x)
        self.pipeline.direction_y_constant(y)
        return x - 0.5 < self.position[0] < x + 0.5 and y - 0.5 < self.position[1] < y + 0.5

    def updatePosition(self):
        avg_throttle = (self.drive_left[0].get() + self.drive_left[1].get() + self.drive_right[0].get() +
                        self.drive_right[1].get()) / 4
        # MAY NEED TO SQUARE AVG DISTANCE
        self.position[0] += self.velocity_to_distance(avg_throttle) * math.cos(self.head * math.pi) * math.cos(
            self.direction * math.pi)
        self.position[1] += self.velocity_to_distance(avg_throttle) * math.sin(self.head * math.pi) * math.cos(
            self.direction * math.pi)

    def drive(self):
        self.speed = self.pipeline.drive() * self.throttle_multiplier
        self.rotation = self.pipeline.rotation()
        for pinion in self.drive_left:
            pinion.set(self.speed)
        for pinion in self.drive_right:
            pinion.set(self.speed)

        if pow(self.pipeline.direction_x(), 2) + pow(self.pipeline.direction_y(), 2) > 0.16:
            self.direction = math.atan2(self.pipeline.direction_x(), -self.pipeline.direction_y()) / math.pi * 360

        for p in range(len(self.swivels)):
            pinion_direction = self.full_rotation[p] * self.rotation + self.direction * (1 - abs(self.rotation))
            sensor = (self.swivel_encoders[p].getPosition() / self.talon_fx_resolution) - 1
            diff = ((pinion_direction / 360) - sensor) - 2 * math.floor(((pinion_direction / 360) - sensor + 1) / 2)
            self.swivels[p].set(ControlMode.PercentOutput, diff * self.direction_multiplier)
