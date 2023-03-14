from wpilib import XboxController
import ctre


class Constants:
    Pigeon2ID = ctre.WPI_Pigeon2(0, "rio")
    ZeroPigeonYaws = XboxController.Button.kA
    AddPigeonYaws = XboxController.Button.kB
