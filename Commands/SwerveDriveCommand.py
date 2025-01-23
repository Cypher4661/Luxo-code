from commands2 import Command
from Subsytem.SwerveSubsystem import SwerveSubsystem
from commands2.button import CommandXboxController


class SwerveDriveCommand(Command):
    def __init__(self, subsys: SwerveSubsystem, controller: CommandXboxController):
        super().__init__()
        self.subsys = subsys
        self.addRequirements(subsys)
        self.controller = controller

    def initialize(self):
        return super().initialize()

    def execute(self):
        self.subsys.drive(
            -self.controller.getLeftY(),
            -self.controller.getLeftX(),
            self.controller.getRightX(),
        )
        return super().execute()

    def end(self, interrupted: bool):
        return super().end(interrupted)

    def isFinished(self) -> bool:
        return super().isFinished()
