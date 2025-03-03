import commands2.command
from Constants import LimeLightConstants
from Commands.GoToPose import GoToPose
from Subsytem.limelight import limelight
from Subsytem.SwerveSubsystem import SwerveSubsystem
from wpimath.geometry import Pose2d
from wpilib import SmartDashboard

class GoToL3Tag(GoToPose):
    def __init__(self, left: bool, swerve:SwerveSubsystem, vision: limelight, 
                 controller: commands2.button.CommandXboxController):
        super().__init__(None, swerve, controller)
        self.left = left
        self.swerve = swerve
        self.vision = vision

    def initialize(self):
        tid = self.vision.getTagId()
        SmartDashboard.putNumber('Go To Tag Id', tid)
        if tid > 0:
            if self.left:
                self.pose = LimeLightConstants.getLeftL3Position(tid)
            else:
                self.pose = LimeLightConstants.getRightL3Position(tid)
        else:
            self.pose = None
        super().initialize()

    def execute(self):
        super().execute()

    def isFinished(self) -> bool:
        return super().isFinished()

    def end(self, interrupted: bool):
        super().end(interrupted)