import ctre
from ctre import WPI_Pigeon2, Pigeon2_Faults
from subsystems.pigeonSubsystem import PigeonSubsystem
import numpy


class Pigeon2Subsystem(PigeonSubsystem):
    m_pigeonFaults: ctre.Pigeon2_Faults
    m_pigeon2: WPI_Pigeon2

    def __init__(self, device: WPI_Pigeon2):
        super(Pigeon2Subsystem, self).__init__(device)
        self.m_pigeon2 = device
        self.m_pigeonFaults = device

    def periodic(self):
        self.m_pigeon2.getFaults(self.m_pigeonFaults)

    def simulationPeriodic(self) -> None:
        pass

    # def getFaults(self) -> Pigeon2_Faults:
    #     return self.m_pigeonFaults.

    def getFault(self):
        return self.m_pigeonFaults.hasAnyFault()

    def getFaultMessage(self) -> str:
        if not self.m_pigeonFaults.hasAnyFault():
            return "No faults"
        retval = ""
        retval += self.m_pigeonFaults.APIError if "APIError, " else ""
        retval += self.m_pigeonFaults.AccelFault if "AccelFault, " else ""
        retval += self.m_pigeonFaults.BootIntoMotion if "BootIntoMotion, " else ""
        retval += self.m_pigeonFaults.GyroFault if "GyroFault, " else ""
        retval += self.m_pigeonFaults.HardwareFault if "HardwareFault, " else ""
        retval += self.m_pigeonFaults.MagnetometerFault if "MagnetometerFault, " else ""
        retval += self.m_pigeonFaults.ResetDuringEn if "ResetDuringEn, " else ""
        retval += self.m_pigeonFaults.SaturatedAccel if "SaturatedAccel, " else ""
        retval += self.m_pigeonFaults.SaturatedMag if "SaturatedMag, " else ""
        retval += self.m_pigeonFaults.SaturatedRotVelocity if "SaturatedRotVelocity, " else ""
        return retval

    # Whether robot is balanced or not on Charging Platform
    def getBalanced(self):
        if 8 < self.m_pigeon2.getRoll() + self.m_pigeon2.getPitch() < 352:
            return True
        return False
