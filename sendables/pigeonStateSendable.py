from wpiutil import Sendable
from wpiutil import SendableBuilder
from subsystems.pigeonSubsystem import PigeonSubsystem


class PigeonStateSendable(Sendable):
    m_pigeon: PigeonSubsystem

    def __init__(self, pigeon: PigeonSubsystem):
        m_pigeon = pigeon

    def initSendable(self, builder: SendableBuilder):
        builder.setSmartDashboardType("PigeonState")
        builder.addDoubleProperty("yaw", self.m_pigeon.getYaw, self.m_pigeon.setYaw)
        builder.addDoubleProperty("pitch", self.m_pigeon.getPitch, None)
        builder.addDoubleProperty("roll", self.m_pigeon.getRoll, None)
        builder.addBooleanProperty("faults", self.m_pigeon.getFault, None)
        builder.addStringProperty("fault message", self.m_pigeon.getFaultMessage, None)
        builder.addDoubleProperty("Compass Heading", self.m_pigeon.getCompass, None)
        builder.addDoubleProperty("AccumZ", self.m_pigeon.getAccumZ, None)
        builder.addDoubleArrayProperty("Raw Gyros", self.m_pigeon.getRawGyros, None)
        builder.addDoubleProperty("Up Time", self.m_pigeon.getUpTime, None)
        builder.addDoubleProperty("Temperature", self.m_pigeon.getTemp, None)



