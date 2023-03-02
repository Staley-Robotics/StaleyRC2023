from ctre import *
from wpilib import XboxController

from src.tools import *


class Arm:

    controller: XboxController

    shaft_bay: float = 0.0
    shaft_low: float = -6127.0
    shaft_mid: float = -12423.0
    shaft_top: float = -30974.0

    pivot_bay: float = 0.0
    pivot_low: float = 1024.0
    pivot_mid: float = 6228.5
    pivot_top: float = 13481.0

    arm_r = WPI_TalonFX(9, "rio")
    arm_e = WPI_TalonFX(18, "rio")

    target_pos: float = 0

    def __init__(self, controller: XboxController):
        self.controller = controller
        self.arm_r.configFactoryDefault()

        self.arm_r.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, PID_loop_idx, k_timeout)
        self.arm_r.setSensorPhase(k_sensor_phase)
        self.arm_r.setInverted(k_motor_invert)

        self.arm_r.configNominalOutputForward(0, k_timeout)
        self.arm_r.configNominalOutputReverse(0, k_timeout)
        self.arm_r.configPeakOutputForward(1, k_timeout)
        self.arm_r.configPeakOutputReverse(-1, k_timeout)

        self.arm_r.configAllowableClosedloopError(0,PID_loop_idx, k_timeout)

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
        self.step()
        self.extend_stick()
        self.pivot()

    def pivot(self):
        target = 0.0
        if self.controller.getAButtonPressed():
            target = self.pivot_bay
        elif self.controller.getBButtonPressed():
            target = self.pivot_low
        elif self.controller.getYButtonPressed():
            target = self.pivot_mid
        elif self.controller.getXButtonPressed():
            target = self.pivot_top

        self.arm_r.set(ControlMode.Position, target)
        print("Encoder " + str(self.arm_r.getSelectedSensorPosition()))
        print("Target " + str(target))
        print("Screw Up Amount " + str(abs(target - self.arm_r.getSelectedSensorPosition())))

    def extend_stick(self,):
        target = self.controller.getLeftY() * -30974
        print(self.arm_e.getSelectedSensorPosition())
        # self.armR.set(ControlMode.Position, target)
        self.arm_e.set(self.controller.getLeftY())

    # FIXME: Inputs interfere with Arm.pivot
    # def extend(self):
    #     target = 0
    #     if self.controller.getAButtonPressed():
    #         target = self.shaft_bay
    #     elif self.controller.getBButtonPressed():
    #         target = self.shaft_low
    #     elif self.controller.getYButtonPressed():
    #         target = self.shaft_mid
    #     elif self.controller.getXButtonPressed():
    #         target = self.shaft_top
    #     self.arm_e.set(ControlMode.Position, target)
    #     print("Encoder " + str(self.arm_e.getSelectedSensorPosition()))
    #     print("Target " + str(target))
    #     print("Screw Up Amount " + str(abs(target - self.arm_e.getSelectedSensorPosition())))

    def step(self):
        if self.controller.getRightBumperPressed():
            self.target_pos += 1024
        if self.controller.getLeftBumperPressed():
            self.target_pos -= 1024
        self.arm_e.set(ControlMode.Position, float(self.target_pos))
        print("target; " + str(self.target_pos))
        print("position; " + str(self.arm_e.getSelectedSensorPosition()))
