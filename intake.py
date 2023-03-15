import ctre
import wpilib


class Intake:

    def __init__(self):
        self.joystick = wpilib.Joystick(0)
        self.motor = ctre.WPI_VictorSPX(5)

    def go(self):

        if self.joystick.getRawButton(2):
            self.motor.set(0.45)

        elif self.joystick.getRawButton(3):
            self.motor.set(-0.45)

        else:
            self.motor.set(0)
