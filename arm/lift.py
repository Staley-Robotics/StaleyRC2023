from ctre import WPI_TalonFX
from wpilib import Encoder, XboxController


def calc_rotations(ticks: int):
    return 0


class Lift:

    """
    TODO Figure out encoder
     get correct vals
     calcRotations() function
     find out how to get length from arm
     implement arm move function
     """

    def __init__(self):
        # variables
        self.grabbingBay = 0
        self.topPos = 245
        self.midPos = 260
        self.lowPos = 300
        self.towardBay = -1  # negative or positive one
        self.towardPlatform = 1  # negative or positive one
        self.extend = 1
        self.retract = -1

        self.bayLength = 4
        self.topLength = 20
        self.midLength = 16
        self.lowLength = 10

        self.armR = WPI_TalonFX(14)  # arm rotation motor number
        self.armL = WPI_TalonFX(13)  # arm extension motor number
        self.armRE = Encoder(14)
        self.armRE.reset()

    def test(self):
        self.armR.set(XboxController.getLeftY)
        self.armRE.get()
        print(self.armRE.get())

    def moveTo(self, pos):
        rotation = calc_rotations(
            self.armRE.tickNum)  # this line to be implemented when I figure out how to get ticks from the
        # encoder
        length = self.armL.length()  # this line to be implemented when I figure out how to get length from the arm
        if pos == "grabZone":
            if rotation > self.grabbingBay:  # the 0 value will change to reflect the smallest/lowest position
                self.armR.set(
                    self.towardBay)  # For these set() calls, I'm assuming that I'm viewing the robot from the port side
                # and CCW is negative
            else:
                self.armR.set(0)
            if length > self.bayLength:
                self.armL.set(self.retract)
            else:
                self.armL.set(0)
        elif pos == "topZone":
            if rotation > self.topPos:  # the 245 value will change to reflect the top position
                self.armR.set(self.towardBay)
            elif rotation < self.topPos:  # the 245 value will change to reflect the top position
                self.armR.set(self.towardPlatform)
            else:
                self.armR.set(0)
            if length > self.topLength:
                self.armL.set(self.retract)
            elif length < self.topLength:
                self.armL.set(self.extend)
            else:
                self.armL.set(0)
        elif pos == "midZone":
            if rotation > self.midPos:  # the 260 value will change to reflect the middle position
                self.armR.set(self.towardBay)
            elif rotation < self.midPos:  # the 260 value will change to reflect the middle position
                self.armR.set(self.towardPlatform)
            else:
                self.armR.set(0)
            if length > self.midLength:
                self.armL.set(self.retract)
            elif length < self.midLength:
                self.armL.set(self.extend)
            else:
                self.armL.set(0)
        elif pos == "lowZone":
            if rotation > self.lowPos:  # the 300 value will change to reflect the hybrid position
                self.armR.set(self.towardBay)
            elif rotation < self.lowPos:  # the 300 value will change to reflect the hybrid position
                self.armR.set(self.towardPlatform)
            else:
                self.armR.set(0)
            if length > self.lowLength:
                self.armL.set(self.retract)
            elif length < self.lowLength:
                self.armL.set(self.extend)
            else:
                self.armL.set(0)