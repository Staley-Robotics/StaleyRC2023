from ctre import *
from tools import *


class ArmedRotation:

    def __init__(self):
        # Variables for use later in code or init; Listed here for easier editing
        self.BayPos = 0
        self.lowPos = 1024
        self.midPos = (13481 - 1024) / 2
        self.topPos = 13481.0
        self.kTimeoutMs = 20
        self.kPIDLoopIdx = 0
        self.kSlotIdx = 0
        self.kGains = Gains(0.1, 0.0, 1.0, 0.0, 0, 1.0)
        self.kSensorPhase = True
        self.kMotorInvert = False

        self.armR = WPI_TalonFX(9, 'rio')  # arm rotation motor number 9
        self.armR.configFactoryDefault()

        # set sensor
        self.armR.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, self.kPIDLoopIdx,
                                               self.kTimeoutMs)
        # correct the motor encoder just in case
        self.armR.setSensorPhase(self.kSensorPhase)
        self.armR.setInverted(self.kMotorInvert)

        # set peak and nominal outputs
        self.armR.configNominalOutputForward(0, self.kTimeoutMs)
        self.armR.configNominalOutputReverse(0, self.kTimeoutMs)
        self.armR.configPeakOutputForward(1, self.kTimeoutMs)
        self.armR.configPeakOutputReverse(-1, self.kTimeoutMs)

        # allowed closed loop error, it will be neutral within this range
        self.armR.configAllowableClosedloopError(0, self.kPIDLoopIdx, self.kTimeoutMs)

        # setting the PID settings
        self.armR.config_kF(self.kPIDLoopIdx, self.kGains.kF, self.kTimeoutMs)
        self.armR.config_kP(self.kPIDLoopIdx, self.kGains.kP, self.kTimeoutMs)
        self.armR.config_kI(self.kPIDLoopIdx, self.kGains.kI, self.kTimeoutMs)
        self.armR.config_kD(self.kPIDLoopIdx, self.kGains.kD, self.kTimeoutMs)

        # Set the quadrature(relative) sensor to match absolutes
        self.armR.setSelectedSensorPosition(0, self.kPIDLoopIdx, self.kTimeoutMs)

    # def loop(self, leftYStick):
    #     # 10 Rotations * 4096 u/rev in either direction
    #     targetPositionRotations = leftYStick * 4096 * 10
    #     self.armR.set(ControlMode.Position, targetPositionRotations)
    #     print(self.armR.getSelectedSensorPosition())

    def loop(self, pos):
        if pos == 0:  # A
            targetPositionRotations = self.BayPos
        elif pos == 1:  # B
            targetPositionRotations = self.lowPos
        elif pos == 2:  # Y
            targetPositionRotations = self.midPos
        elif pos == 3:  # X
            targetPositionRotations = self.topPos

        self.armR.set(ControlMode.Position, targetPositionRotations)
        print("Encoder " + str(self.armR.getSelectedSensorPosition()))
        print("Target " + str(targetPositionRotations))
        print("Screw Up Amount " + str(abs(targetPositionRotations - self.armR.getSelectedSensorPosition())))