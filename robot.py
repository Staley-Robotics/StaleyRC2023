import math

from ctre import WPI_TalonFX, WPI_VictorSPX, WPI_TalonSRX, FeedbackDevice
import wpilib
import wpilib.drive
import wpimath
import wpimath.controller
import wpimath.trajectory

from drivetrain import Drivetrain
from arm import ArmedExtension, ArmedRotation
from pcm import Pcm
from claw import Claw


class Robot(wpilib.TimedRobot):
    time: wpilib.Timer
    controller1: wpilib.XboxController
    controller2: wpilib.XboxController
    drivetrain: Drivetrain
    arm_ext: ArmedExtension
    arm_rot: ArmedRotation
    pcm: Pcm
    claw: Claw

    extendMotor: WPI_TalonSRX
    extendPID: wpimath.controller.PIDController
    extendFeedForward: wpimath.controller.SimpleMotorFeedforwardMeters

    def robotInit(self):

        self.time = wpilib.Timer()
        self.controller1 = wpilib.XboxController(0)
        self.controller2 = wpilib.XboxController(1)
        self.drivetrain = Drivetrain()
        # self.arm = ArmedExtension(WPI_TalonFX(11), WPI_VictorSPX(9))
        self.arm_rot = ArmedRotation()
        self.arm_ext = ArmedExtension()
        # self.pcm = Pcm(wpilib.PneumaticsControlModule(0))
        # self.claw = Claw(self.pcm.getSolendoid(1))

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
        self.pos = 0

    def testPeriodic(self):
        # self.arm_rot.loop(self.controller1.getLeftY())
        # self.arm_ext.extendLStick(self.controller1.getLeftY())
        # self.arm_ext.stepExtend(self.controller1.getLeftBumperPressed(), self.controller1.getRightBumperPressed())

        if self.controller1.getAButtonPressed():
            self.pos = 0
        elif self.controller1.getBButtonPressed():
            self.pos = 1
        elif self.controller1.getYButtonPressed():
            self.pos = 2
        elif self.controller1.getXButtonPressed():
            self.pos = 3
        self.arm_rot.loop(self.pos)


if __name__ == "__main__":
    wpilib.run(Robot)
