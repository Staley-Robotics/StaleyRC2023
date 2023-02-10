import math

import ctre
import wpilib
import wpilib.drive
import wpimath
import wpimath.controller
import wpimath.trajectory

from drivetrain import Drivetrain
from arm import ArmedExtension
from pcm import Pcm
from claw import Claw


class Robot(wpilib.TimedRobot):
    time: wpilib.Timer
    controller1: wpilib.XboxController
    controller2: wpilib.XboxController
    drivetrain: Drivetrain
    arm: ArmedExtension
    pcm: Pcm
    claw: Claw

    extendMotor: ctre.WPI_TalonSRX
    extendPID: wpimath.controller.PIDController
    extendFeedForward: wpimath.controller.SimpleMotorFeedforwardMeters

    def robotInit(self):

        self.time = wpilib.Timer()
        self.controller1 = wpilib.XboxController(0)
        self.controller2 = wpilib.XboxController(1)
        self.drivetrain = Drivetrain()
        self.arm = ArmedExtension(ctre.WPI_TalonFX(11), ctre.WPI_VictorSPX(9))
        self.pcm = Pcm(wpilib.PneumaticsControlModule(0))
        self.claw = Claw(self.pcm.getSolendoid(1))

        try:
            pdp = wpilib.PowerDistribution(0, wpilib.PowerDistribution.ModuleType.kCTRE)
            faults = pdp.getStickyFaults()
            print(f'{faults.print()}')
            pdp.clearStickyFaults()
        except Exception as e:
            print(e)

    def autonomousInit(self):
        self.time.reset()
        self.time.start()

    def autonomousPeriodic(self):
        pass

    def autonomousExit(self):
        self.time.stop()

    def teleopInit(self):
        self.time.start()

    def teleopPeriodic(self):

        self.drivetrain.update()
        self.arm.update()
        self.pcm.update()

        if self.controller1.getAButtonPressed():
            self.claw.update()

    def teleopExit(self):
        self.time.stop()

    def testInit(self):

        self.extendMotor = ctre.WPI_TalonSRX(31)
        self.extendMotor.configSelectedFeedbackSensor(ctre.FeedbackDevice.QuadEncoder, 0, 0)
        self.extendMotor.getSensorCollection()
        self.extendMotor.setSelectedSensorPosition(0, 0, 0)

        self.extendMotor.configForwardSoftLimitThreshold(16384, 0)
        self.extendMotor.configForwardSoftLimitEnable(True, 0)
        self.extendMotor.configReverseSoftLimitThreshold(-1, 0)
        self.extendMotor.configReverseSoftLimitEnable(True, 0)

        profile = wpimath.trajectory.TrapezoidProfile.Constraints(math.pi, 2 * math.pi)
        self.extendPID = wpimath.controller.ProfiledPIDController(0.5, 0, 0, profile)
        # self.extendPID.setTolerance(0.1, 0.1)
        # self.extendPID.setIntegratorRange(-0.25, 0.25)
        # self.extendPID.setSetpoint()

    def testPeriodic(self):

        count = self.extendMotor.getSelectedSensorPosition()

        if self.controller1.getLeftBumperPressed() and count < 16384:

            # self.extendMotor.set(self.controller1.getLeftY())
            print(f"counts = {count}")
            Volts = self.extendPID.calculate(count, 8 * math.pi)
            self.extendMotor.setVoltage(min(Volts, 1))
            print(Volts)

        elif self.controller1.getRightBumperPressed() and count > 0:

            print(f"counts = {count}")
            Volts = self.extendPID.calculate(count, 0)
            self.extendMotor.setVoltage(max(Volts, -1))
            print(Volts)


if __name__ == "__main__":
    wpilib.run(Robot)