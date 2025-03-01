from commands2 import Command
from Subsytem.CorralIntake import corralIntake
from Constants import CorralIntake
from commands2.button import CommandXboxController
from wpilib import Timer


class corralIntakeCommand(Command):
    def __init__(
        self,
        subsys: corralIntake,
        power: float,
        isDeafultCommand: bool = False,
        controller: CommandXboxController = None,
    ):
        self.power = power
        self.subsys = subsys
        self.addRequirements(subsys)
        self.isDeafultCommand = isDeafultCommand
        self.controller = controller
        super().__init__()

    def initialize(self):
        self.time = Timer()
        self.time.start()
        return super().initialize()

    def execute(self):
        if self.controller and self.controller.getRightTriggerAxis() >= 0.5:
            self.subsys.duty_motor(0.2)
            #self.controller.setRumble(1.0)
        else:
            self.subsys.duty_motor(self.power)
        return super().execute()

    def isFinished(self):
        return not self.isDeafultCommand and self.subsys.get_motor_current() >= 40.5
        return super().isFinished()

    def end(self, interrupted):
        self.subsys.stop()
        self.time.stop()
        self.time.reset()
        return super().end(interrupted)
