from commands2 import Command
from wpilib import SmartDashboard
from Subsytem.AlgaeArmSubsys import AlgaeArmSubsys
from Constants import AlgaeSubsys


class AlgaeArmCommand(Command):
    def __init__(self, subsys: AlgaeArmSubsys, angle: float, isDeafultCommand: bool = False):
        self.subsys: AlgaeArmSubsys = subsys
        self.addRequirements(subsys)
        self.angle = angle
        self.isDefaultCommand = isDeafultCommand
        super().__init__()

    def initialize(self):
        return super().initialize()

    def execute(self):
        if (self.subsys.at_limit() or abs(self.subsys.get_current_degree() - self.angle) <= AlgaeSubsys.deadband):
            self.subsys.stop()
        else:
            self.subsys.motor_to_position(self.angle)
        return super().execute()

    def isFinished(self):
        return (
            not self.isDefaultCommand
            and abs(self.subsys.get_current_degree() - self.angle) <= AlgaeSubsys.deadband
        )

    def end(self, interrupted):
        self.subsys.stop()
