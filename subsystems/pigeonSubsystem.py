import ctre.sensors
import commands2
from abc import abstractmethod


class PigeonSubsystem(commands2.SubsystemBase):
    m_basePigeon: ctre.BasePigeon

    def __init__(self, basePigeon: ctre.BasePigeon):
        super(PigeonSubsystem, self).__init__()
        self.m_basePigeon = basePigeon

    def getYaw(self) -> float:
        return self.m_basePigeon.getYaw()

    def getRoll(self) -> float:
        return self.m_basePigeon.getRoll()

    def getPitch(self) -> float:
        return self.m_basePigeon.getPitch()

    def setYaw(self, yaw: float):
        self.m_basePigeon.setYaw(yaw)

    def addYaw(self, yaw: float):
        self.m_basePigeon.addYaw(yaw, 10)

    def setYawToCompass(self):
        self.m_basePigeon.setYawToCompass(10)

    def setAccumZ(self, accumZ: float):
        self.m_basePigeon.setAccumZAngle(accumZ, 10)

    @abstractmethod
    def getFault(self):
        """Returns string containing fault message"""
        pass

    def getCompass(self) -> float:
        return self.m_basePigeon.getCompassHeading()

    # use getAccumZ()[integer] for x y or z
    def getAccumZ(self) -> float:
        self.accums = self.m_basePigeon.getAccumGyro()
        return self.accums

    # Just getting the gyros, getRawGyros()[integer] for x y or z
    def getRawGyros(self):
        self.gyrs = self.m_basePigeon.getRawGyro()
        return self.gyrs

    # Pigeon run time, 255 sec cap
    def getUpTime(self) -> int:
        return self.m_basePigeon.getUpTime()

    # Celsius
    def getTemp(self) -> float:
        return self.m_basePigeon.getTemp()

    @abstractmethod
    def getFaultMessage(self) -> str:
        pass
    """Returns string containing fault message"""
