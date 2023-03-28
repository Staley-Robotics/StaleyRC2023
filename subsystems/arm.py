from ctre import *

from wpilib import *
from . import Subsystems
from math import pi


class Gains:
    def __init__(self, _kP, _kI, _kD, _kF, _kIzone, _kPeakOutput):
        self.kP = _kP
        self.kI = _kI
        self.kD = _kD
        self.kF = _kF
        self.kIzone = _kIzone
        self.kPeakOutput = _kPeakOutput


k_timeout: int = 20
PID_loop_idx: int = 0
k_slot_idx: int = 0
k_gains: Gains = Gains(0.1, 0.0, 0.0, 0.0, 0, 1.0)
k_Egains: Gains = Gains(1.0, 0.0, 0.0, 0.0, 0, 1.0)


def deg(ticks: float) -> float:
    return ticks * (2048 / 360 * 200)


def inch(ticks: float) -> float:
    return ticks * (pi * 4096 / 360 * 1)


class Arm(Subsystems):
    BAY = 0
    LOW = 1
    MID = 2
    TOP = 3

    pivot_stages: list[float] = [deg(10.0), deg(20.0), deg(85.0), deg(60.0)]
    extend_stages: list[float] = [deg(0.0), deg(6.0), deg(24.0), deg(46.0)]

    arm_r = WPI_TalonFX(9, "rio")
    arm_e = WPI_TalonSRX(31)

    op2: XboxController
    pivot: float
    extend: float

    def initVariables(self):
        self.op2 = XboxController(1)

        self.arm_r.configFactoryDefault()
        self.arm_r.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, PID_loop_idx, k_timeout)
        self.arm_r.setSensorPhase(True)
        self.arm_r.setInverted(False)
        self.arm_r.configNominalOutputForward(0, k_timeout)
        self.arm_r.configNominalOutputReverse(0, k_timeout)
        self.arm_r.configPeakOutputForward(0.5, k_timeout)
        self.arm_r.configPeakOutputReverse(-0.5, k_timeout)
        self.arm_r.configAllowableClosedloopError(PID_loop_idx, 0, k_timeout)
        self.arm_r.config_kF(PID_loop_idx, k_gains.kF, k_timeout)
        self.arm_r.config_kP(PID_loop_idx, k_gains.kP, k_timeout)
        self.arm_r.config_kI(PID_loop_idx, k_gains.kI, k_timeout)
        self.arm_r.config_kD(PID_loop_idx, k_gains.kD, k_timeout)

        self.arm_e.configFactoryDefault()
        self.arm_e.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, PID_loop_idx, k_timeout)
        self.arm_e.setSensorPhase(False)
        self.arm_e.setInverted(False)
        self.arm_e.configNominalOutputForward(0, k_timeout)
        self.arm_e.configNominalOutputReverse(0, k_timeout)
        self.arm_e.configPeakOutputForward(1.0, k_timeout)
        self.arm_e.configPeakOutputReverse(-1.0, k_timeout)
        self.arm_e.configAllowableClosedloopError(PID_loop_idx, 0, k_timeout)
        self.arm_e.config_kF(PID_loop_idx, k_Egains.kF, k_timeout)
        self.arm_e.config_kP(PID_loop_idx, k_Egains.kP, k_timeout)
        self.arm_e.config_kI(PID_loop_idx, k_Egains.kI, k_timeout)
        self.arm_e.config_kD(PID_loop_idx, k_Egains.kD, k_timeout)
        self.arm_e.setSelectedSensorPosition(0.0)
        self.arm_e.selectProfileSlot(0, 0)

    def run(self):
        self.set_mode([
            self.op2.getAButtonPressed(),
            self.op2.getBButtonPressed(),
            self.op2.getXButtonPressed(),
            self.op2.getYButtonPressed()
        ].index(True))
        self.offset(
            self.op2.getRightY(),
            self.op2.getRightTriggerAxis(),
            self.op2.getLeftTriggerAxis(),
            self.op2.getLefttBumperPressed()
        )
        self.apply()

    def set_mode(self, mode: int):
        self.pivot = self.pivot_stages[mode]
        self.extend = self.extend_stages[mode]

    def offset(self, out: float, up: float, down: float, slow: bool):
        speed = 2.0
        if slow:
            speed = 0.9

        self.pivot += deg(up * speed)
        self.pivot -= deg(down * speed)
        self.extend = inch(out)
        self.extend = max(min(self.extend, self.extend_stages[-1]), self.extend_stages[0])

    def apply(self):
        print(self.pivot, self.extend)
        self.arm_r.set(ControlMode.Position, self.pivot)
        self.arm_e.set(ControlMode.Position, self.extend)

    # def extension(self, goal):
    #     print(self.arm_e.getSelectedSensorPosition())
    #     return
    #     current = self.target_pos
    #     toChange = goal * -410
    #     target = current + toChange
    #     if target > 15000:
    #         target = 15000
    #     elif target < 0:
    #         target = 0
    #
    #     print(current, toChange, target, self.arm_e.getSelectedSensorPosition())
    #     self.target_pos = target
    #     self.arm_e.set(ControlMode.Position, int(target))
