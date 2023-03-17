import wpilib
from ctre import WPI_Pigeon2

from . import Subsystems
from subsystems.arm import Arm
from subsystems.claw import Claw
from subsystems.swervedrive import SwerveDrive4


class Auto(Subsystems):

    timer: wpilib.Timer
    steps: tuple[list[callable], list[callable], list[callable]]
    mode: int = 1
    index: int = 0
    is_new_step: bool = True

    def __init__(self, swerve: SwerveDrive4, arm: Arm, claw: Claw):
        super().__init__()
        self.swerve = swerve
        self.arm = arm
        self.claw = claw
        self.timer = wpilib.Timer()
        self.steps = (
            [
                lambda: self.drive(0.75, 0.0, 0.0, 0.0, 0.0, 2.0)
            ],
            [
                lambda: self.arm_pivot(2),
                lambda: self.pause(2.0),
                lambda: self.claw_toggle(),
                lambda: self.pause(3.0),
                lambda: self.claw_toggle(),
                lambda: self.pause(2.0),
                lambda: self.arm_pivot(0)
            ],
            [

            ]
        )

    def initVariables(self):
        gyroId: int = int(self.ntCfgs.getNumber("gyroId", 61))
        gyroCanbus: str = self.ntCfgs.getString("gyroCanbus", "rio")
        self.gyro = WPI_Pigeon2(gyroId, gyroCanbus)

    def pause(self, delay):
        if self.is_new_step:
            self.timer.reset()
            self.timer.start()
            self.is_new_step = False
        return self.timer.get() >= delay

    def claw_toggle(self):
        if self.is_new_step:
            self.timer.reset()
            self.timer.start()
            self.is_new_step = False
        self.claw.toggle(True)
        return True

    def balance(self):
        if self.is_new_step:
            self.timer.reset()
            self.timer.start()
            self.is_new_step = False
        self.swerve.drive(self.gyro.getPitch() / 360, 0, 0)
        return abs(self.gyro.getPitch()) < 5

    def arm_pivot(self, goal):
        if self.is_new_step:
            self.timer.reset()
            self.timer.start()
            self.is_new_step = False
        self.arm.pivot(goal == 0, goal == 1, goal == 2, goal == 3)
        return True

    def arm_extend(self, goal):
        if self.is_new_step:
            self.timer.reset()
            self.timer.start()
            self.is_new_step = False
        if goal > self.arm.arm_e.getSelectedSensorPosition():
            self.arm.extend_stickler(0.2)
        elif goal < self.arm.arm_e.getSelectedSensorPosition():
            self.arm.extend_stickler(-0.2)
        return self.arm.arm_e.getSelectedSensorPosition() > goal - 10 and self.arm.arm_e.getSelectedSensorPosition() < goal + 10

    def drive(self, lx, ly, omega, rx, ry, time):
        if self.is_new_step:
            self.timer.reset()
            self.timer.start()
            self.is_new_step = False
        self.swerve.drive(-lx, ly, omega)
        return self.timer.get() >= time

    def run(self):
        if self.steps[self.mode][self.index]() and self.index < len(self.steps[self.mode]):
            self.index += 1
            self.is_new_step = True
