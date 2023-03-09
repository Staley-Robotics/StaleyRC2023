# vehemently vomiting violent vulgarisms
# placidly placing purple plaques
# sadistically spacing sanguine serpents
# tremulously tracking tramping truant tripe
import wpilib
from ctre import *

from tools import *
from math import pi


class Arm:

    pipeline: PipelineManager

    shaft_ratio: int = 50
    mag_encoder: int = 4096
    circumference: float = pi
    inch_to_ticks: int = circumference * mag_encoder/360 * shaft_ratio
    shaft_floor: float = 4 * inch_to_ticks  # resting on floor
    shaft_bay: float = 0.0  # needs tuning
    shaft_low: float = 6 * inch_to_ticks  # needs tuning
    shaft_mid: float = 24 * inch_to_ticks  # needs tuning
    shaft_top: float = 46 * inch_to_ticks  # needs tuning

    integrated_encoder: int = 2048
    pivot_ratio: int = 200
    degrees_to_ticks = integrated_encoder/360 * pivot_ratio
    pivot_bay: float = 10 * degrees_to_ticks
    pivot_low: float = 20.0 * degrees_to_ticks
    pivot_mid: float = 75.0 * degrees_to_ticks
    pivot_top: float = 60.0 * degrees_to_ticks

    arm_r = WPI_TalonFX(9, "rio")
    arm_e = WPI_TalonFX(31, "rio")

    target_pos: float = 0

    pivot_atm: float = 0

    def __init__(self, pipeline: PipelineManager):
        self.pipeline = pipeline
        self.arm_r.configFactoryDefault()

        self.arm_r.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, PID_loop_idx, k_timeout)
        self.arm_r.setSensorPhase(k_sensor_phase)
        self.arm_r.setInverted(k_motor_invert)

        self.arm_r.configNominalOutputForward(0, k_timeout)
        self.arm_r.configNominalOutputReverse(0, k_timeout)
        self.arm_r.configPeakOutputForward(1, k_timeout)
        self.arm_r.configPeakOutputReverse(-1, k_timeout)

        self.arm_r.configAllowableClosedloopError(0, PID_loop_idx, k_timeout)

        self.arm_r.config_kF(PID_loop_idx, k_gains.kF, k_timeout)
        self.arm_r.config_kP(PID_loop_idx, k_gains.kP, k_timeout)
        self.arm_r.config_kI(PID_loop_idx, k_gains.kI, k_timeout)
        self.arm_r.config_kD(PID_loop_idx, k_gains.kD, k_timeout)

        self.arm_e.setSelectedSensorPosition(0, PID_loop_idx, k_timeout)

        self.arm_e.configFactoryDefault()

        self.arm_e.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, PID_loop_idx, k_timeout)
        self.arm_e.setSensorPhase(k_sensor_phase)
        self.arm_e.setInverted(k_motor_invert)

        self.arm_e.configNominalOutputForward(0, k_timeout)
        self.arm_e.configNominalOutputReverse(0, k_timeout)
        self.arm_e.configPeakOutputForward(1, k_timeout)
        self.arm_e.configPeakOutputReverse(-1, k_timeout)

        self.arm_e.configAllowableClosedloopError(0, PID_loop_idx, k_timeout)

        self.arm_e.config_kF(PID_loop_idx, k_gains.kF, k_timeout)
        self.arm_e.config_kP(PID_loop_idx, k_gains.kP, k_timeout)
        self.arm_e.config_kI(PID_loop_idx, k_gains.kI, k_timeout)
        self.arm_e.config_kD(PID_loop_idx, k_gains.kD, k_timeout)

        self.arm_e.setSelectedSensorPosition(0, PID_loop_idx, k_timeout)

        self.arm_e.selectProfileSlot(0, 0)

        # for list pivot function
        self.possiblePivotPos = [self.pivot_bay, self.pivot_low, self.pivot_mid, self.pivot_top]
        self.possibleExtendPos = [self.shaft_bay, self.shaft_low, self.shaft_mid, self.shaft_top]
        self.index: int = 0
        self.pivotAd = 0
        self.extendAd = 0
        self.curPos = 0
        self.target = 0
        self.log = -1

    def run_checks(self):
        self.extend_stickler()
        self.rotate_stickler()
        # self.extend()

    def rotate_stickler(self):
        self.pivot_atm += self.pipeline.pivot_axis() * 0.5 * self.degrees_to_ticks
        self.pivot_atm -= self.pipeline.pivot_negative_axis() * 0.5 * self.degrees_to_ticks
        # self.pipeline.pivot_axis() * 90 * self.degrees_to_ticks BAD CODE DON'T USE
        self.arm_r.set(ControlMode.Position, self.pivot_atm)

    def extend_stickler(self):
        target = self.pipeline.shaft_axis() * -48 * self.inch_to_ticks
        if target < -48 * self.inch_to_ticks:
            target = -48 * self.inch_to_ticks
        self.arm_e.set(ControlMode.Position, target)

    def pivot(self):
        if self.pipeline.point_1():
            if self.pivot_bay >= self.target > 0:
                self.target -= 1 * self.degrees_to_ticks
            elif self.target <= 0:
                self.target = 0
            else:
                self.target = self.pivot_bay
        elif self.pipeline.point_2():
            self.target = self.pivot_low
        elif self.pipeline.point_3():
            self.target = self.pivot_mid
        elif self.pipeline.point_4():
            self.target = self.pivot_top

        self.arm_r.set(ControlMode.Position, self.target)
        print("Encoder " + str(self.arm_r.getSelectedSensorPosition()))
        print("Target " + str(self.target))
        print("Screw Up Amount " + str(abs(self.target - self.arm_r.getSelectedSensorPosition())))

    def listPivot(self):
        if self.pipeline.point_2:
            if self.index < len(self.possiblePivotPos)-1 and self.log > self.pipeline.time.get()-20:
                self.index += 1
                self.pivotAd = 0
                self.log = self.pipeline.time.get()
        elif self.pipeline.point_3 and self.log > self.pipeline.time.get()-20:
            if self.index > 0:
                self.index -= 1
                self.pivotAd = 0
                self.log = self.pipeline.time.get()
        print(self.index)
        target = self.possiblePivotPos[int(self.index)]
        if self.pipeline.point_1:
            self.pivotAd += 1 * self.degrees_to_ticks
        elif self.pipeline.point_4:
            self.pivotAd -= 1 * self.degrees_to_ticks
        target += self.pivotAd
        self.curPos = self.possiblePivotPos[int(self.index)]
        self.arm_r.set(ControlMode.Position, target)

    def extend(self):
        target = 0.0
        if self.pipeline.point_1():
            target = self.shaft_bay
        elif self.pipeline.point_2():
            target = self.shaft_low
        elif self.pipeline.point_3():
            target = self.shaft_mid
        elif self.pipeline.point_4():
            target = self.shaft_top
        if target > 48 * self.inch_to_ticks:
            target = 48 * self.inch_to_ticks
        self.arm_e.set(ControlMode.Position, target)
        print("Encoder " + str(self.arm_e.getSelectedSensorPosition()))
        print("Target " + str(target))
        print("Screw Up Amount " + str(abs(target - self.arm_e.getSelectedSensorPosition())))

    def listExtend(self):
        if self.pipeline.point_2:
            if self.index < len(self.possibleExtendPos) - 1 and self.log > self.pipeline.time.get() - 20:
                self.index += 1
                self.extendAd = 0
                self.log = self.pipeline.time.get()
        elif self.pipeline.point_3 and self.log > self.pipeline.time.get() - 20:
            if self.index > 0:
                self.index -= 1
                self.extendAd = 0
                self.log = self.pipeline.time.get()
        print(self.index)
        target = self.possibleExtendPos[int(self.index)]
        if self.pipeline.point_1:
            self.extendAd += 1 * self.degrees_to_ticks
        elif self.pipeline.point_4:
            self.extendAd -= 1 * self.degrees_to_ticks
        target += self.extendAd
        self.curPos = self.possibleExtendPos[int(self.index)]
        self.arm_e.set(ControlMode.Position, target)
