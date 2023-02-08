import ctre
import wpilib
import wpilib.drive

from drivetrain import Drivetrain
from arm import Arm
from pcm import Pcm
from claw import Claw


class Robot(wpilib.TimedRobot):
    time: wpilib.Timer
    controller1: wpilib.XboxController
    controller2: wpilib.XboxController
    drivetrain: Drivetrain
    arm: Arm
    pcm: Pcm
    claw: Claw
    testMotor: ctre.WPI_TalonSRX

    def robotInit(self):

        self.time = wpilib.Timer()
        self.controller1 = wpilib.XboxController(0)
        self.controller2 = wpilib.XboxController(1)
        self.drivetrain = Drivetrain()
        self.arm = Arm(ctre.WPI_TalonFX(11), ctre.WPI_VictorSPX(9))
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

        # calculationForSwerveDrive()

        if self.controller1.getAButtonPressed():
            self.claw.update()

    def teleopExit(self):
        self.time.stop()

    def testInit(self):
        self.testMotor = ctre.WPI_TalonSRX(31)
        self.testMotor.configSelectedFeedbackSensor(ctre._ctre.FeedbackDevice.QuadEncoder, 0, 0)

    def testPeriodic(self):
        self.testMotor.set(self.controller1.getLeftY() * 0.5)
        # count = self.testMotor.getQuadraturePosition()
        # print(count)
        # ctre.SensorCollection(motorController: ctre._ctre.BaseTalon)
        # getQuadraturePosition()


if __name__ == "__main__":
    wpilib.run(Robot)
