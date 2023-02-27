from wpilib import Solenoid


class Claw:
    solenoid: Solenoid = None

    def __init__(self, solenoid: Solenoid):
        self.solenoid = solenoid

    def update(self):
        self.solenoid.set(not self.solenoid.get())

    def getState(self):
        return self.solenoid.get()
