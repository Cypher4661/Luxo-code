from commands2 import Command
from Subsytem.SwerveSubsystem import SwerveSubsystem
from commands2.button import CommandXboxController
import Constants

class SlowSwerveDriveCommand(Command):
    def __init__(self, subsys: SwerveSubsystem, controller: CommandXboxController):
        super().__init__()
        self.subsys = subsys
        self.addRequirements(self.subsys)
        self.controller = controller

    def initialize(self):
        return super().initialize()

    def execute(self):
        self.subsys.drive(
            -self.controller.getLeftY()*Constants.DriveConstants.slowDriveMultiplier,
            -self.controller.getLeftX()*Constants.DriveConstants.slowDriveMultiplier,
            self.controller.getRightX()*Constants.DriveConstants.slowDriveMultiplier,
        )
        return super().execute()

    def end(self, interrupted: bool):
        return super().end(interrupted)

    def isFinished(self) -> bool:
        return super().isFinished()
