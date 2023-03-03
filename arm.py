# vehemently vomiting violent vulgarisms
# placidly placing purple plaques
# sadistically spacing sanguine serpents
# tremulously tracking tramping truant tripe
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
    pivot_bay: float = 0.0  # needs tuning
    pivot_low: float = 1024.0  # needs tuning
    pivot_mid: float = 6228.5  # needs tuning
    pivot_top: float = 13481.0  # needs tuning

    arm_r = WPI_TalonFX(9, "rio")
    arm_e = WPI_TalonFX(18, "rio")

    target_pos: float = 0

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

    def run_checks(self):
        self.pivot()
        # self.extend()

    def stickler(self):
        self.arm_r.set()

    def pivot(self):
        target = 0.0
        if self.pipeline.point_1():
            target = self.pivot_bay
        elif self.pipeline.point_2():
            target = self.pivot_low
        elif self.pipeline.point_3():
            target = self.pivot_mid
        elif self.pipeline.point_4():
            target = self.pivot_top

        self.arm_r.set(ControlMode.Position, target)
        print("Encoder " + str(self.arm_r.getSelectedSensorPosition()))
        print("Target " + str(target))
        print("Screw Up Amount " + str(abs(target - self.arm_r.getSelectedSensorPosition())))

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
