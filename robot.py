import wpilib
import wpilib.drive

from drivetrain import Drivetrain
from intake import Intake
from limelightSimple import Limelight


class Robot(wpilib.TimedRobot):
    time: wpilib.Timer
    joystick: wpilib.Joystick
    drivetrain: Drivetrain
    intake: Intake
    limelight: Limelight

    def robotInit(self):
        self.time = wpilib.Timer()
        self.joystick = wpilib.Joystick(0)
        self.drivetrain = Drivetrain()
        self.intake = Intake()
        self.limelight = Limelight()

    def teleopInit(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        self.drivetrain.drive(-self.joystick.getY(), self.joystick.getX(), self.joystick.getZ())
        self.intake.go()
        self.limelight.look()

    def teleopExit(self) -> None:
        pass


if __name__ == "__main__":
    wpilib.run(Robot)
