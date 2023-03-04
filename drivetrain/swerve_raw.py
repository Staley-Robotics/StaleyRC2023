import math

from ctre import *

from drivetrain.chassis import Chassis
from tools import *


class Swerve(Chassis):

    speed: float
    rotation: float
    direction: float
    drive_left: tuple[WPI_TalonFX, WPI_TalonFX]
    drive_right: tuple[WPI_TalonFX, WPI_TalonFX]
    swivels: tuple[WPI_TalonFX, WPI_TalonFX, WPI_TalonFX, WPI_TalonFX]
    full_rotation: tuple[int, int, int, int] = (-0.75, -0.25, 0.25, 0.75)

    def __init__(self, pipeline: PipelineManager):
        super().__init__(pipeline)

        self.speed = 0.0
        self.rotation = 0.0
        self.direction = 0.0

        self.talon_fx_resolution *= 10

        self.drive_left = (WPI_TalonFX(5, "canivore1"), WPI_TalonFX(1, "canivore1"))
        self.drive_right = (WPI_TalonFX(3, "canivore1"), WPI_TalonFX(7, "canivore1"))
        self.swivels = (WPI_TalonFX(6, "canivore1"), WPI_TalonFX(2, "canivore1"),
                        WPI_TalonFX(4, "canivore1"), WPI_TalonFX(8, "canivore1"))

        self.drive_left[0].setInverted(True)
        self.drive_left[1].setInverted(True)

        for pinion in self.swivels:
            pinion.configFactoryDefault()

            pinion.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, PID_loop_idx, k_timeout)

            pinion.configNominalOutputForward(0, k_timeout)
            pinion.configNominalOutputReverse(0, k_timeout)
            pinion.configPeakOutputForward(1, k_timeout)
            pinion.configPeakOutputReverse(-1, k_timeout)

            pinion.config_kF(PID_loop_idx, k_gains.kF, k_timeout)
            pinion.config_kP(PID_loop_idx, k_gains.kP, k_timeout)
            pinion.config_kI(PID_loop_idx, k_gains.kI, k_timeout)
            pinion.config_kD(PID_loop_idx, k_gains.kD, k_timeout)

            pinion.setSelectedSensorPosition(0, PID_loop_idx, k_timeout)

    def goto(self, x: float, y: float, r: float = 0):
        self.pipeline.drive_constant(0.5)
        self.pipeline.direction_x_constant(x - r)
        self.pipeline.direction_y_constant(y - r)

    def updatePosition(self):
        avg_throttle = (self.drive_left[0].get() + self.drive_left[1].get() + self.drive_right[0].get() + self.drive_right[1].get()) / 4
        # MAY NEED TO SQUARE AVG DISTANCE
        self.position[0] += self.velocity_to_distance(avg_throttle) * math.cos(self.head * math.pi) * math.cos(self.direction * math.pi)
        self.position[1] += self.velocity_to_distance(avg_throttle) * math.sin(self.head * math.pi) * math.cos(self.direction * math.pi)

    def drive(self):
        self.speed = self.pipeline.drive() * self.throttle_multiplier
        self.rotation = self.pipeline.rotation()
        for pinion in self.drive_left:
            pinion.set(self.speed)
        for pinion in self.drive_right:
            pinion.set(self.speed)

        if pow(self.pipeline.direction_x(), 2) + pow(self.pipeline.direction_y(), 2) > 0.04:
            self.direction = math.atan2(self.pipeline.direction_x(), -self.pipeline.direction_y()) / math.pi
        for p in range(len(self.swivels)):
            pinion_direction = self.full_rotation[p] * self.rotation + self.direction * (1 - abs(self.rotation))
            sensor = (self.swivels[p].getSelectedSensorPosition()) / self.talon_fx_resolution

            if pinion_direction - sensor > 0:
                self.swivels[p].set((abs(pinion_direction - sensor) % 1) * self.direction_multiplier)
            elif pinion_direction - sensor < 0:
                self.swivels[p].set(-(abs(pinion_direction - sensor) % 1) * self.direction_multiplier)
            else:
                self.swivels[p].set(0)
