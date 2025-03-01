from commands2 import Command
from Subsytem.AlgaeIntakeSubsystem import AlgaeIntakeSubsystem
from Constants import AlgaeIntake
from commands2.button import CommandXboxController


class AlgaeIntakeCommand(Command):
    def __init__(
        self,
        subsys: AlgaeIntakeSubsystem,
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
        pass

    def execute(self):
        if self.controller and self.controller.getLeftTriggerAxis() >= 0.2:
            self.subsys.setPower(AlgaeIntake.keepPower)
        else:
            self.subsys.setPower(self.power)

    def isFinished(self):
        return not self.isDeafultCommand and self.subsys.get_motor_current() >= AlgaeIntake.algaeCollectedAmper

    def end(self, interrupted):
        self.subsys.stop()
