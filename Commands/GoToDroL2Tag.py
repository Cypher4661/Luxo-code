import commands2.command
from Constants import LimeLightConstants
from Commands.GoToPose import GoToPose
from Subsytem.limelight import limelight
from Subsytem.SwerveSubsystem import SwerveSubsystem
from wpimath.geometry import Pose2d, Rotation2d
from wpilib import SmartDashboard

class GoToDropL2Tag(GoToPose):
    def __init__(self, swerve:SwerveSubsystem, vision: limelight,
                 controller: commands2.button.CommandXboxController):
        super().__init__(None, swerve, controller)
        self.swerve = swerve
        self.vision = vision

    def initialize(self):
        tid = self.vision.getTagId()
        SmartDashboard.putNumber('Go/Tag Id', tid)
        if tid > 0:
            pose = LimeLightConstants.getLeftL3Position(tid)
            heading = pose.rotation().rotateBy(Rotation2d.fromDegrees(90))
            self.pose = Pose2d(pose.translation(), heading)
        else:
            self.pose = None
        super().initialize()

    def execute(self):
        super().execute()

    def isFinished(self) -> bool:
        return super().isFinished()

    def end(self, interrupted: bool):
        super().end(interrupted)