import wpilib


class Claw:
    solenoid: wpilib.Solenoid = None

    def __init__(self, solenoid: wpilib.Solenoid):
        self.solenoid = solenoid

    def update(self):
        self.solenoid.set(not self.solenoid.get())

    def getState(self):
        return self.solenoid.get()
