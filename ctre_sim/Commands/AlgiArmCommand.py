from commands2 import Command
from wpilib import SmartDashboard
from Subsytem.AlgiArmSubsys import algiArmSubsys
from Constants import AlgiSubsys


class algiArmCommand(Command):
    def __init__(
        self, subsys: algiArmSubsys, angle: float, isDeafultCommand: bool = False
    ):
        self.subsys: algiArmSubsys = subsys
        self.addRequirements(subsys)
        self.angle = angle
        self.isDeafultCommand = isDeafultCommand
        super().__init__()

    def initialize(self):
        return super().initialize()

    def execute(self):
        if abs(self.subsys.get_current_degree() - self.angle) <= AlgiSubsys.deadband:
             self.subsys.stop()
        else:
            self.subsys.motor_to_position(self.angle)
        return super().execute()

    def isFinished(self):
        return (
            not self.isDeafultCommand
            and abs(self.subsys.get_current_degree() - self.angle)
            <= AlgiSubsys.deadband
        )
        return super().isFinished()

    def end(self, interrupted):
        self.subsys.stop()
        return super().end(interrupted)
