import commands2
from subsystems.networkTables import NetworkTables


class ReportingSubsystem(commands2.SubsystemBase):
    # Placeholder __init__
    def __init__(self):
        super().__init__()
        self.networkTable = NetworkTables()

    def periodic(self):
        self.networkTable.update()

    def simulationPeriodic(self):
        pass
        # This method will be called once per scheduler run during simulation
