import wpilib
import wpilib.drive

from drivetrain import Drivetrain
from limelightSimple import Limelight


class Robot(wpilib.TimedRobot):
    time: wpilib.Timer
    drivetrain: Drivetrain
    controllerStick: wpilib.Joystick
    # controller1: wpilib.XboxController
    # controller2: wpilib.XboxController
    limelight: Limelight

    def robotInit(self):
        self.time = wpilib.Timer()
        self.drivetrain = Drivetrain()
        self.controllerStick = wpilib.Joystick(0)
        # self.controller1 = wpilib.XboxController(0)
        # self.controller2 = wpilib.XboxController(1)
        # wpilib.CameraServer.launch("limelight.py:launch")
        self.limelight = Limelight()

    def testInit(self):
        pass

    def testPeriodic(self):
        self.drivetrain.drive(-self.controllerStick.getY(), self.controllerStick.getX())
        # self.drivetrain.drive(self.controller1.getLeftY(), self.controller1.getLeftX())
        self.limelight.look()


if __name__ == "__main__":
    wpilib.run(Robot)
