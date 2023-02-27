from wpilib import *
from ntcore import *

from controller import Controller
from drivetrain.drivetrain import Drivetrain
from drivetrain.swerve import Swerve
from drivetrain.tank import Tank


class Robot(TimedRobot):
    time: Timer
    state: NetworkTableInstance
    controller: Controller
    drivetrain: Drivetrain

    def robotInit(self):
        self.time = Timer()
        self.state = NetworkTableInstance.getDefault()
        self.controller = Controller(0, self.state)
        self.drivetrain = Tank(self.state)

    def teleopInit(self):
        self.time.start()

    def teleopPeriodic(self):
        self.controller.update()
        self.drivetrain.drive()

    def teleopExit(self):
        self.time.stop()


if __name__ == "__main__":
    run(Robot)
