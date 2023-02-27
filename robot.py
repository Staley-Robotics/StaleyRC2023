from wpilib import *
from ntcore import *

from controller import Controller
from drivetrain.tank import Tank


class Robot(TimedRobot):
    time: Timer
    state: NetworkTableInstance
    controller: Controller
    tank: Tank

    def robotInit(self):
        self.time = Timer()
        self.state = NetworkTableInstance.getDefault()
        self.controller = Controller(0, self.state)
        self.tank = Tank(self.state)

    def teleopInit(self):
        self.time.start()

    def teleopPeriodic(self):
        self.controller.update()
        self.tank.drive()

    def teleopExit(self):
        self.time.stop()


if __name__ == "__main__":
    run(Robot)
