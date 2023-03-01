import math
from typing import Tuple

import wpilib
from wpilib import *
from ctre import *
from ntcore import *

from drivetrain.chassis import Chassis


class Gains:
    def __init__(self, kP, kI, kD, kF, kIzone, kPeakOutput):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.kF = kF
        self.kIzone = kIzone
        self.kPeakOutput = kPeakOutput


class Swerve(Chassis):

    speed: float
    rotation: float
    direction: float
    throttle_left_pinions: Tuple[WPI_TalonFX, WPI_TalonFX]
    throttle_right_pinions: Tuple[WPI_TalonFX, WPI_TalonFX]
    rotation_pinions: Tuple[WPI_TalonFX, WPI_TalonFX, WPI_TalonFX, WPI_TalonFX]

    kTimeoutMs = 20
    kPIDLoopIdx = 0
    kSlotIdx = 0
    kGains = Gains(0.1, 0.0, 1.0, 0.0, 0, 1.0)
    kSensorPhase = False
    kMotorInvert = False

    def __init__(self, inherited_state: NetworkTableInstance, controller: XboxController):
        super().__init__(inherited_state, controller)

        # TODO: Test swerve configuration
        self.config = self.state.getTable("swerve")
        self.speed = 0.0
        self.rotation = 0.0
        self.direction = 0.0

        self.throttle_left_pinions = (
            WPI_TalonFX(self.config.getNumber("modules/throttle/0", 1), "canivore1"),
            WPI_TalonFX(self.config.getNumber("modules/throttle/1", 3), "canivore1")
        )
        self.throttle_right_pinions = (
            WPI_TalonFX(self.config.getNumber("modules/throttle/2", 5), "canivore1"),
            WPI_TalonFX(self.config.getNumber("modules/throttle/3", 7), "canivore1")
        )
        self.rotation_pinions = (
            WPI_TalonFX(self.config.getNumber("modules/rotation/0", 2), "canivore1"),
            WPI_TalonFX(self.config.getNumber("modules/rotation/1", 4), "canivore1"),
            WPI_TalonFX(self.config.getNumber("modules/rotation/2", 6), "canivore1"),
            WPI_TalonFX(self.config.getNumber("modules/rotation/3", 8), "canivore1")
        )

        self.throttle_right_pinions[0].setInverted(True)
        self.throttle_left_pinions[1].setInverted(True)

        for pinion in self.rotation_pinions:
            pinion.configFactoryDefault()

            pinion.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, self.kPIDLoopIdx, self.kTimeoutMs)

            pinion.configNominalOutputForward(0, self.kTimeoutMs)
            pinion.configNominalOutputReverse(0, self.kTimeoutMs)
            pinion.configPeakOutputForward(1, self.kTimeoutMs)
            pinion.configPeakOutputReverse(-1, self.kTimeoutMs)

            pinion.config_kF(self.kPIDLoopIdx, self.kGains.kF, self.kTimeoutMs)
            pinion.config_kP(self.kPIDLoopIdx, self.kGains.kP, self.kTimeoutMs)
            pinion.config_kI(self.kPIDLoopIdx, self.kGains.kI, self.kTimeoutMs)
            pinion.config_kD(self.kPIDLoopIdx, self.kGains.kD, self.kTimeoutMs)

            pinion.setSelectedSensorPosition(0, self.kPIDLoopIdx, self.kTimeoutMs)

    def drive(self):
        self.speed = self.controller.getLeftY() * self.throttle_multiplier
        self.rotation = self.controller.getLeftX() * self.rotation_multiplier
        if pow(self.controller.getRightX(), 2) + pow(self.controller.getRightY(), 2) > 0.04:
            self.direction = (-math.atan2(self.controller.getRightX(), self.controller.getRightY()) / math.pi)

        for pinion in self.throttle_left_pinions:
            pinion.set(self.speed)

        for pinion in self.throttle_right_pinions:
            pinion.set(self.speed)

        for pinion in self.rotation_pinions:
            sensor = (pinion.getSelectedSensorPosition() / (self.talon_fx_revolution_steps * 10)) % 1
            pinion.set((abs(self.direction) - abs(sensor)) * self.direction_multiplier)
