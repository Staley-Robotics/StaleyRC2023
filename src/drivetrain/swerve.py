import math

from ctre import *

from chassis import Chassis
from src.tools import *


class Swerve(Chassis):

    speed: float
    rotation: float
    direction: float
    throttle_left_pinions: Tuple[WPI_TalonFX, WPI_TalonFX]
    throttle_right_pinions: Tuple[WPI_TalonFX, WPI_TalonFX]
    rotation_pinions: Tuple[WPI_TalonFX, WPI_TalonFX, WPI_TalonFX, WPI_TalonFX]

    def __init__(self, pipeline: PipelineManager):
        super().__init__(pipeline)

        self.speed = 0.0
        self.rotation = 0.0
        self.direction = 0.0

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
        self.rotation = self.pipeline.rotation() * self.rotation_multiplier
        if pow(self.pipeline.direction_x(), 2) + pow(self.pipeline.direction_y(), 2) > 0.04:
            self.direction = math.atan2(self.pipeline.direction_x(), -self.pipeline.direction_y()) / math.pi

        for pinion in self.throttle_left_pinions:
            pinion.set(self.speed)

        for pinion in self.throttle_right_pinions:
            pinion.set(self.speed)

        for pinion in self.rotation_pinions:
            sensor = (pinion.getSelectedSensorPosition() / self.talon_fx_resolution)
            if self.direction - sensor <= 1:
                pinion.set((self.direction - sensor) * self.direction_multiplier)
            else:
                pinion.set(((1 + self.direction) + (1 - sensor)) * self.direction_multiplier)
