import math
from typing import Tuple

from wpilib import *
from ctre import *
from ntcore import *

from chassis.drivetrain import Drivetrain


class Gains:
    def __init__(self, kP, kI, kD, kF, kIzone, kPeakOutput):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        self.kF = kF
        self.kIzone = kIzone
        self.kPeakOutput = kPeakOutput


class Swerve(Drivetrain):

    throttle_pinions: MotorControllerGroup
    rotation_pinions: Tuple[WPI_TalonFX, WPI_TalonFX, WPI_TalonFX, WPI_TalonFX]

    kTimeoutMs = 20
    kPIDLoopIdx = 0
    kSlotIdx = 0
    kGains = Gains(0.1, 0.0, 1.0, 0.0, 0, 1.0)
    kSensorPhase = True
    kMotorInvert = False

    def __init__(self, inherited_state: NetworkTableInstance):
        super().__init__(inherited_state)

        self.drivetrain = self.state.getTable("swerve")

        self.throttle_pinions = MotorControllerGroup(
            WPI_TalonFX(self.drivetrain.getNumber("modules/throttle/0", 1), "rio"),
            WPI_TalonFX(self.drivetrain.getNumber("modules/throttle/1", 3), "rio"),
            WPI_TalonFX(self.drivetrain.getNumber("modules/throttle/2", 5), "rio"),
            WPI_TalonFX(self.drivetrain.getNumber("modules/throttle/3", 7), "rio")
        )
        self.rotation_pinions = (
            WPI_TalonFX(self.drivetrain.getNumber("modules/rotation/0", 2), "rio"),
            WPI_TalonFX(self.drivetrain.getNumber("modules/rotation/1", 4), "rio"),
            WPI_TalonFX(self.drivetrain.getNumber("modules/rotation/2", 6), "rio"),
            WPI_TalonFX(self.drivetrain.getNumber("modules/rotation/3", 8), "rio")
        )

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
        speed = self.in_LY.get() * self.throttle_multiplier
        rotation = ((-math.atan2(self.in_RX.get(), self.in_RY.get())+(math.pi/4)) / math.pi) * self.talon_fx_revolution_steps
        self.throttle_pinions.set(speed)
        for pinion in self.rotation_pinions:
            pinion.set(ControlMode.Position, rotation)
