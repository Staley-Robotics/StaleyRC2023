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
        self.topPos = 245
        self.midPos = 260
        self.lowPos = 300
        self.kTimeoutMs = 20
        self.kPIDLoopIdx = 0
        self.kSlotIdx = 0
        self.kGains = Gains(0.15, 0.0, 1.0, 0.0, 0, 1.0)
        self.kSensorPhase = True
        self.kMotorInvert = False

        # self.armR = ctre.WPI_TalonFX(14)  # arm rotation motor number
        self.armR = ctre.WPI_TalonSRX(31)
        self.armR.configFactoryDefault()
        # set stuff
        self.armR.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, self.kPIDLoopIdx,
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
        # Grab the 360 degree position of the MagEncoder's absolute position, and initially set the relative sensor
        # to match
        absolutePosition = self.armR.getSensorCollection().getPulseWidthPosition()

        # Mask out overflows, keep bottom 12 bits
        absolutePosition &= 0xFFF
        if self.kSensorPhase:
            absolutePosition *= -1
        if self.kMotorInvert:
            absolutePosition *= -1
        # Set the quadrature(relative) sensor to match absolute
        self.armR.setSelectedSensorPosition(absolutePosition, self.kPIDLoopIdx, self.kTimeoutMs)

        self.armRE = wpilib.Encoder(14)
        self.armRE.reset()

        self.lastButton1 = False

    def loop(self, button1, button2, leftYstick):

        if not (self.lastButton1 & button1):
            # 10 Rotations * 4096 u/rev in either direction
            targetPositionRotations = leftYstick * 10.0 * 4096
            self.armR.set(ControlMode.Position, targetPositionRotations)
            print(targetPositionRotations)

        # When button 2 is held, just straight drive
        if button2:
            # Percent Output
            self.armR.set(ControlMode.PercentOutput, leftYstick)
        _lastButton1 = button1


class Gains:
    def __init__(self, _kP, _kI, _kD, _kF, _kIzone, _kPeakOutput):
        self.kP = _kP
        self.kI = _kI
        self.kD = _kD
        self.kF = _kF
        self.kIzone = _kIzone
        self.kPeakOutput = _kPeakOutput
