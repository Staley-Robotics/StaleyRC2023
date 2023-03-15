from ctre import *

from wpilib import *
from wpimath import applyDeadband
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
k_sensor_phase = True
k_motor_invert = False

class Arm(Subsystems):

    shaft_ratio: int = 1
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
    arm_e = WPI_TalonSRX(31) #, "rio")

    target_pos: float = 2500
    extend_pos: float = 0.0
    pivot_atm: float = 0

    op2: XboxController

    def initVariables(self):
        self.op2 = XboxController(1)

        self.arm_r.configFactoryDefault()

        self.arm_r.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, PID_loop_idx, k_timeout)
        self.arm_r.setSensorPhase(k_sensor_phase)
        self.arm_r.setInverted(k_motor_invert)

        self.arm_r.configNominalOutputForward(0, k_timeout)
        self.arm_r.configNominalOutputReverse(0, k_timeout)
        self.arm_r.configPeakOutputForward(0.5, k_timeout)
        self.arm_r.configPeakOutputReverse(-0.5, k_timeout)

        self.arm_r.configAllowableClosedloopError(0, PID_loop_idx, k_timeout)

        self.arm_r.config_kF(PID_loop_idx, k_gains.kF, k_timeout)
        self.arm_r.config_kP(PID_loop_idx, k_gains.kP, k_timeout)
        self.arm_r.config_kI(PID_loop_idx, k_gains.kI, k_timeout)
        self.arm_r.config_kD(PID_loop_idx, k_gains.kD, k_timeout)

        self.arm_e.configFactoryDefault()

        self.arm_e.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, PID_loop_idx, k_timeout)
        self.arm_e.setSensorPhase(False) #k_sensor_phase)
        self.arm_e.setInverted(False) #k_motor_invert)

        self.arm_e.configNominalOutputForward(0, k_timeout)
        self.arm_e.configNominalOutputReverse(0, k_timeout)
        self.arm_e.configPeakOutputForward(1.0, k_timeout)
        self.arm_e.configPeakOutputReverse(-1.0, k_timeout)

        self.arm_e.configAllowableClosedloopError(PID_loop_idx, 0, k_timeout)

        self.arm_e.config_kF(PID_loop_idx, k_Egains.kF, k_timeout)
        self.arm_e.config_kP(PID_loop_idx, k_Egains.kP, k_timeout)
        self.arm_e.config_kI(PID_loop_idx, k_Egains.kI, k_timeout)
        self.arm_e.config_kD(PID_loop_idx, k_Egains.kD, k_timeout)

        #self.arm_e.setSelectedSensorPosition(0, PID_loop_idx, k_timeout)
        absPos = self.arm_e.getSensorCollection().getPulseWidthPosition()
        offset = 5000
        self.arm_e.setSelectedSensorPosition( offset )

        self.arm_e.selectProfileSlot(0, 0)

        self.possiblePivotPos = [self.pivot_bay, self.pivot_low, self.pivot_mid, self.pivot_top]
        self.possibleExtendPos = [self.shaft_bay, self.shaft_low, self.shaft_mid, self.shaft_top]
        self.index: int = 0
        self.pivotAd = 0
        self.extendAd = 0
        self.curPos = 0
        self.target = 0
        self.log = -1
        self.target = 0

    def getInputs(self) -> tuple[float, float, float, bool, bool, bool, bool]:
        point_1 = False#self.op2.getAButtonPressed()
        point_2 = self.op2.getBButtonPressed()
        point_3 = self.op2.getXButtonPressed()
        point_4 = self.op2.getYButtonPressed()
        pivot_axis = self.op2.getRightTriggerAxis()
        pivot_negative_axis = self.op2.getLeftTriggerAxis()
        shaft_axis = applyDeadband( self.op2.getRightY(), 0.1, 1.0 )
        return pivot_axis, pivot_negative_axis, shaft_axis, point_1, point_2, point_3, point_4

    def run(self):
        self.extend_stickler(self.getInputs()[2])
        self.rotate_stickler(self.getInputs()[0], self.getInputs()[1])
        # self.extend()

    def rotate_stickler(self, up, down):
        self.pivot_atm += up * 1.0 * self.degrees_to_ticks
        self.pivot_atm -= down * 1.0 * self.degrees_to_ticks
        self.arm_r.set(ControlMode.Position, self.pivot_atm)

    def extend_stickler(self, goal):
        pass
        current = self.target_pos #self.arm_e.getSelectedSensorPosition()
        toChange = goal * -410 #-48 * self.inch_to_ticks
        target = current + toChange
        #if target < -48 * self.inch_to_ticks:
        #    target = -48 * self.inch_to_ticks
        if target > 15000:
            target = 15000
        elif target < 0:
            target = 0

        print( current, toChange, target, self.arm_e.getSelectedSensorPosition() )
        self.target_pos = target
        self.arm_e.set(ControlMode.Position, int(target))

    def pivot(self, bay, low, mid, top):
        if bay:
            if self.pivot_bay >= self.target > 0:
                self.target -= 1 * self.degrees_to_ticks
            elif self.target <= 0:
                self.target = 0
            else:
                self.target = self.pivot_bay
        elif low:
            self.target = self.pivot_low
        elif mid:
            self.target = self.pivot_mid
        elif top:
            self.target = self.pivot_top

        self.arm_r.set(ControlMode.Position, self.target)
        # print("Encoder " + str(self.arm_r.getSelectedSensorPosition()))
        # print("Target " + str(self.target))
        # print("Screw Up Amount " + str(abs(self.target - self.arm_r.getSelectedSensorPosition())))

    def extend(self):
        target = 0.0
        if self.getInputs()[3]:
            target = self.shaft_bay
        elif self.getInputs()[4]:
            target = self.shaft_low
        elif self.getInputs()[5]:
            target = self.shaft_mid
        elif self.getInputs()[6]:
            target = self.shaft_top
        if target > 48 * self.inch_to_ticks:
            target = 48 * self.inch_to_ticks
        self.arm_e.set(ControlMode.Position, target)
        #print("Encoder " + str(self.arm_e.getSelectedSensorPosition()))
        #print("Target " + str(target))
        #print("Screw Up Amount " + str(abs(target - self.arm_e.getSelectedSensorPosition())))

    def listExtend(self):
        if self.getInputs()[4]:
            if self.index < len(self.possibleExtendPos) - 1 and self.log > self.time.get() - 20:
                self.index += 1
                self.extendAd = 0
                self.log = self.pipeline.time.get()
        elif self.getInputs()[5] and self.log > self.pipeline.time.get() - 20:
            if self.index > 0:
                self.index -= 1
                self.extendAd = 0
                self.log = self.pipeline.time.get()
        #print(self.index)
        target = self.possibleExtendPos[int(self.index)]
        if self.getInputs()[3]:
            self.extendAd += 1 * self.degrees_to_ticks
        elif self.getInputs()[6]:
            self.extendAd -= 1 * self.degrees_to_ticks
        target += self.extendAd
        self.curPos = self.possibleExtendPos[int(self.index)]
        self.arm_e.set(ControlMode.Position, target)
