import math

from ctre import *
from ctre.sensors import *

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
    rotation_pinion_offset: tuple[int, int, int, int] = (47.373, -154.492+180, 15.996, -158.115+180)
    full_rotation: Tuple[int, int, int, int] = (-0.75, -0.25, 0.25, 0.75)
    k_gains: Gains = Gains(0.03, 0.0, 0.0, 0.0, 0, 1.0)
    turning_encoders: tuple[WPI_CANCoder, WPI_CANCoder, WPI_CANCoder, WPI_CANCoder]

    def __init__(self, pipeline: PipelineManager):
        super().__init__(pipeline)

        self.speed = 0.0
        self.rotation = 0.0
        self.direction = 0.0

        self.talon_fx_resolution *= 10

        self.throttle_right_pinions = (WPI_TalonFX(1, "canivore1"), WPI_TalonFX(5, "canivore1"))
        self.throttle_left_pinions = (WPI_TalonFX(7, "canivore1"), WPI_TalonFX(3, "canivore1"))
        self.rotation_pinions = (WPI_TalonFX(6, "canivore1"), WPI_TalonFX(2, "canivore1"),
                                 WPI_TalonFX(4, "canivore1"), WPI_TalonFX(8, "canivore1"))
        self.turning_encoders = (WPI_CANCoder(16, "canivore1"), WPI_CANCoder(12, "canivore1"),
                                 WPI_CANCoder(14, "canivore1"), WPI_CANCoder(18, "canivore1"))

        self.throttle_right_pinions[0].setInverted(True)
        self.throttle_right_pinions[1].setInverted(True)

        for p in range(len(self.rotation_pinions)):
            pinion: WPI_TalonFX = self.rotation_pinions[p]
            encoder: WPI_CANCoder = self.turning_encoders[p]
            pinion.configFactoryDefault()
            encoder.configFactoryDefault()

            encoder.configMagnetOffset(self.rotation_pinion_offset[p])
            encoder.setPositionToAbsolute()

            # self.rotation_pinions[p].configRemoteFeedbackFilter(self.turning_encoder, 0)
            # self.rotation_pinions[p].configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, PID_loop_idx, k_timeout)

            pinion.configNominalOutputForward(0, k_timeout)
            pinion.configNominalOutputReverse(0, k_timeout)
            pinion.configPeakOutputForward(1, k_timeout)
            pinion.configPeakOutputReverse(-1, k_timeout)

            pinion.configClosedLoopPeriod(0, 20)

            pinion.config_kF(PID_loop_idx, k_gains.kF, k_timeout)
            pinion.config_kP(PID_loop_idx, k_gains.kP, k_timeout)
            pinion.config_kI(PID_loop_idx, k_gains.kI, k_timeout)
            pinion.config_kD(PID_loop_idx, k_gains.kD, k_timeout)

            pinion.setInverted(True)

            # encoder.setPosition(encoder.getAbsolutePosition() - self.rotation_pinion_offset[p])

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
            pinion_direction = self.direction
            # pinion_direction = self.full_rotation[p]
            # pinion_direction = self.full_rotation[p] * self.rotation + self.direction * (1 - abs(self.rotation))
            sensor = self.turning_encoders[p].getPosition() / 180 - 1
            if pinion_direction - sensor > 0:
                self.rotation_pinions[p].set(ControlMode.PercentOutput, (abs(pinion_direction - sensor) % 1) * self.direction_multiplier)
            elif pinion_direction - sensor < 0:
                self.rotation_pinions[p].set(ControlMode.PercentOutput, -(abs(pinion_direction - sensor) % 1) * self.direction_multiplier)
            else:
                self.rotation_pinions[p].set(ControlMode.PercentOutput, 0)
