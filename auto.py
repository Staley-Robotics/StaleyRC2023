from subsystems import Subsystems
import wpilib


class Auto:

    timer: wpilib.Timer
    steps: tuple[list[callable], list[callable], list[callable()]]
    mode: int
    lx = 0.0
    ly = 0.0
    lt = 0.0
    rx = 0.0
    ry = 0.0
    rt = 0.0
    index: int = 0
    is_new_step: bool = True

    def __init__(self, swerve: Subsystems, arm: Subsystems, claw: Subsystems):
        self.swerve = swerve
        self.arm = arm
        self.claw = claw

    def initSubsystem(self):
        self.timer = wpilib.Timer()
        self.steps = (
            [
                lambda: self.drive(0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 5.0)
            ],
            [],
            []
        )
        self.swerve.getInputs = self.getSwerveInputs()

    def getSwerveInputs(self):
        return self.lx, self.ly, self.lt, self.rx, self.ry, self.rt

    def drive(self, lx, ly, lt, rx, ry, rt, time):
        if self.is_new_step:
            self.timer.reset()
            self.timer.start()
            self.is_new_step = False
        self.lx = lx
        self.ly = ly
        self.lt = lt
        self.rx = rx
        self.ry = ry
        self.rt = rt
        return self.timer.get() >= time

    def run(self):
        if self.steps[self.index]():
            self.steps += 1
            self.is_new_step = True
