import wpilib


class Robot(wpilib.TimedRobot):
    time: wpilib.Timer
    controller1: wpilib.XboxController
    controller2: wpilib.XboxController

    def robotInit(self):

        self.time = wpilib.Timer()
        self.controller1 = wpilib.XboxController(0)
        self.controller2 = wpilib.XboxController(1)

    def testInit(self):
        pass

    def testPeriodic(self):
        pass


if __name__ == "__main__":
    wpilib.run(Robot)
