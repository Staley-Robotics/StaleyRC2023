from ctre import *

from src.tools import *


class Arm:

    pipeline: PipelineManager

    shaft_bay: float = 0.0  # needs tuning
    shaft_low: float = -6127.0  # needs tuning
    shaft_mid: float = -12423.0  # needs tuning
    shaft_top: float = -30974.0  # needs tuning

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
        self.extend()

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
        self.arm_e.set(ControlMode.Position, target)
        print("Encoder " + str(self.arm_e.getSelectedSensorPosition()))
        print("Target " + str(target))
        print("Screw Up Amount " + str(abs(target - self.arm_e.getSelectedSensorPosition())))
