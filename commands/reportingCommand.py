from sendables.pigeonStateSendable import PigeonStateSendable
from subsystems.pigeonSubsystem import PigeonSubsystem
from subsystems.reportingSubsystem import ReportingSubsystem
from commands2 import CommandBase
from wpilib import SmartDashboard


class ReportingCommand(CommandBase):
    m_pigeons = []
    m_pigeonSendables = []

    def __init__(self, reportingSubsystem: ReportingSubsystem, *pigeonsubsystem: PigeonSubsystem):
        super().__init__()
        for system in pigeonsubsystem:
            self.m_pigeons.append(system)
            self.m_pigeonSendables.append(PigeonStateSendable(system))
            CommandBase.addRequirements(system)

        CommandBase.addRequirements(reportingSubsystem)

    def runsWhenDisabled(self) -> bool:
        return True

    def initialize(self) -> None:
        pass

    def execute(self) -> None:
        # Sends Pigeon updates to dashboard
        iteration = 0
        for sendable in self.m_pigeonSendables:
            iteration += 1
            SmartDashboard.putData("Pigeon " + str(iteration) + sendable)

    # Called once command should end
    def end(self, interrupted: bool) -> None:
        pass

    # Returns true when command should end.
    def isFinished(self) -> bool:
        return False
