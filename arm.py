import ctre
import wpilib
import wpimath.controller
import math

from ctre import FeedbackDevice, ControlMode


# vehemently vomiting violent vulgarisms
# placidly placing purple plaques
# sadistically spacing sanguine serpents
# tremulously tracking tramping truant tripe


class ArmedExtension:
    liftMotor: ctre.WPI_VictorSPX = None
    # liftMotor: ctre.WPI_TalonFX = None
    extendMotor: ctre.VictorSPX = None
    liftSensors: ctre.TalonFXSensorCollection = None
    startPos = None
    armMath = None

    def __init__(self, lMotor, eMotor):
        self.liftMotor = lMotor
        self.extendMotor = eMotor
        self.liftSensors = self.liftMotor.getSensorCollection()
        self.startPos = self.liftSensors.getIntegratedSensorPosition()
        self.armMath = wpimath.controller.ArmFeedforward(kS=0, kG=0, kV=0, kA=0)

    def update(self, controller2: wpilib.XboxController = None):
        pass

    def lift(self, direction):
        __posValue__ = self.liftSensors.getIntegratedSensorPosition()
        __rValue__ = math.radians(__posValue__)
        if direction > 0:
            __rValue__ = math.radians(45)
        elif direction < 0:
            __rValue__ = math.radians(self.startPos)
        volts = self.armMath.calculate(angle=__rValue__, velocity=direction)
        self.liftMotor.set(volts)

    def extend(self):
        pass

    def setLiftPhysics(self, kS=0, kG=0, kV=0, kA=0):
        self.armMath.kS = kS
        self.armMath.kG = kG
        self.armMath.kV = kV
        self.armMath.kA = kA


class ArmedRotation:
    """
    TODO
        implement arm move function
    """

    def __init__(self):
        # Variables for use later in code or init; Listed here for easier editing
        self.BayPos = 0
        self.topPos = 4096*3
        self.midPos = 4096*7
        self.lowPos = 4096*10
        self.kTimeoutMs = 20
        self.kPIDLoopIdx = 0
        self.kSlotIdx = 0
        self.kGains = Gains(0.0312, 0.0, 1.0, 0.0, 0, 1.0)
        self.kSensorPhase = True
        self.kMotorInvert = False

        self.armR = ctre.WPI_TalonFX(7, 'canivore1')  # arm rotation motor number
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

        '''
        # Grab the 360 degree position of the MagEncoder's absolute position, and initially set the relative sensor
        # to match
        absolutePosition = self.armR.getSensorCollection().getIntegratedSensorAbsolutePosition()

        # Mask out overflows, keep bottom 12 bits
        absolutePosition &= 0xFFF
        if self.kSensorPhase:
            absolutePosition *= -1
        if self.kMotorInvert:
            absolutePosition *= -1
            '''
        # Set the quadrature(relative) sensor to match absolutes
        self.armR.setSelectedSensorPosition(0, self.kPIDLoopIdx, self.kTimeoutMs)
        
    '''def loop(self, leftYStick):
        # 10 Rotations * 4096 u/rev in either direction
        targetPositionRotations = leftYStick * 4096 * 10
        # targetPositionRotations = 0
        self.armR.set(ControlMode.Position, targetPositionRotations)
        print(targetPositionRotations)'''

    def loop(self, pos):
        if pos == 0:
            targetPositionRotations = self.BayPos
        elif pos == 1:
            targetPositionRotations = self.lowPos
        elif pos == 2:
            targetPositionRotations = self.midPos
        elif pos == 3:
            targetPositionRotations = self.topPos
        self.armR.set(ControlMode.Position, targetPositionRotations)
        print(self.armR.getSelectedSensorPosition())


class Gains:
    def __init__(self, _kP, _kI, _kD, _kF, _kIzone, _kPeakOutput):
        self.kP = _kP
        self.kI = _kI
        self.kD = _kD
        self.kF = _kF
        self.kIzone = _kIzone
        self.kPeakOutput = _kPeakOutput
