from commands2 import Command
from Subsytem.CorralIntake import corralIntake
from Constants import CorralIntake


class corralIntakeCommand(Command):
    def __init__(
        self, subsys: corralIntake, power: float, isDeafultCommand: bool = False
    ):
        self.power = power
        self.subsys = subsys
        self.addRequirements(subsys)
        self.isDeafultCommand = isDeafultCommand
        super().__init__()

    def initialize(self):
        return super().initialize()

    def execute(self):
        self.subsys.duty_motor(self.power)
        return super().execute()

    def isFinished(self):
        return not self.isDeafultCommand and self.subsys.get_motor_current() >= 25.5
        return super().isFinished()

    def end(self, interrupted):
        self.subsys.stop()
        return super().end(interrupted)
