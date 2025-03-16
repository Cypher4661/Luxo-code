from commands2 import Command
from Subsytem.AlgiIntake import algiIntake
from Constants import AlgiIntake
from commands2.button import CommandXboxController


class algiIntakeCommand(Command):
    def __init__(
        self,
        subsys: algiIntake,
        power: float,
        isDeafultCommand: bool = False,
        controller: CommandXboxController = None,
    ):
        self.power = power
        self.subsys = subsys
        self.addRequirements(self.subsys)
        self.isDeafultCommand = isDeafultCommand
        self.controller = controller
        super().__init__()

    def initialize(self):
        return super().initialize()

    def execute(self):
        if self.controller and self.controller.getLeftTriggerAxis() >= 0.5:
            self.subsys.duty_motor(0.5)
        else:
            self.subsys.duty_motor(self.power)
        return super().execute()

    def isFinished(self):
        return False
        return super().isFinished()

    def end(self, interrupted):
        self.subsys.stop()
        return super().end(interrupted)
