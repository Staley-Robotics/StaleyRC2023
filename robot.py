import wpilib
from limelightSimple import Limelight


class Robot(wpilib.TimedRobot):
    time: wpilib.Timer
    controller1: wpilib.XboxController
    controller2: wpilib.XboxController
    limelight: Limelight

    def robotInit(self) -> None:

        self.time = wpilib.Timer()
        self.controller1 = wpilib.XboxController(0)
        self.controller2 = wpilib.XboxController(1)
        self.limelight = Limelight()

    def testInit(self) -> None:
        pass

    def testPeriodic(self) -> None:
        self.limelight.look()


if __name__ == "__main__":
    wpilib.run(Robot)
