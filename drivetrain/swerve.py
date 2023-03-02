import math

from ctre import *

from drivetrain.chassis import Chassis
from tools import *


# fr: 6
# bl: 2
# br: 4
# fl: 8


class Swerve(Chassis):

    speed: float
    rotation: float
    direction: float
    throttle_left_pinions: Tuple[WPI_TalonFX, WPI_TalonFX]
    throttle_right_pinions: Tuple[WPI_TalonFX, WPI_TalonFX]
    rotation_pinions: Tuple[WPI_TalonFX, WPI_TalonFX, WPI_TalonFX, WPI_TalonFX]
    rotation_pinion_offsets: Tuple[int, int, int, int] = (1374, 1690, 222, 78)
    full_rotation: Tuple[int, int, int, int] = (-0.75, -0.25, 0.25, 0.75)

    def __init__(self, pipeline: PipelineManager):
        super().__init__(pipeline)

        self.speed = 0.0
        self.rotation = 0.0
        self.direction = 0.0

        self.talon_fx_resolution *= 10

        self.throttle_left_pinions = (WPI_TalonFX(1, "canivore1"), WPI_TalonFX(3, "canivore1"))
        self.throttle_right_pinions = (WPI_TalonFX(5, "canivore1"), WPI_TalonFX(7, "canivore1"))
        self.rotation_pinions = (WPI_TalonFX(2, "canivore1"), WPI_TalonFX(4, "canivore1"),
                                 WPI_TalonFX(6, "canivore1"), WPI_TalonFX(8, "canivore1"))

        self.throttle_right_pinions[0].setInverted(True)
        self.throttle_left_pinions[1].setInverted(True)

        for pinion in self.rotation_pinions:
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

    def drive(self):
        self.speed = self.pipeline.throttle() * self.throttle_multiplier
        self.rotation = self.pipeline.rotation()
        for pinion in self.throttle_left_pinions:
            pinion.set(self.speed)
        for pinion in self.throttle_right_pinions:
            pinion.set(self.speed)

        if pow(self.pipeline.direction_x(), 2) + pow(self.pipeline.direction_y(), 2) > 0.04:
            self.direction = math.atan2(self.pipeline.direction_x(), -self.pipeline.direction_y()) / math.pi
        for p in range(len(self.rotation_pinions)):
            pinion_direction = self.full_rotation[p] * self.rotation + self.direction * (1 - abs(self.rotation))
            sensor = (self.rotation_pinions[p].getSelectedSensorPosition()) / self.talon_fx_resolution
            self.rotation_pinions[p].set((pinion_direction - sensor) * self.direction_multiplier)
