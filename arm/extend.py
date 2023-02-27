from ctre import WPI_TalonFX, VictorSPX, TalonFXSensorCollection
import wpilib
import wpimath.controller
import math


class Extend:

    liftMotor: WPI_TalonFX = None
    extendMotor: VictorSPX = None
    liftSensors: TalonFXSensorCollection = None
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
