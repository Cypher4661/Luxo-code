import commands2.command
from Constants import LimeLightConstants
from Commands.GoToPose import GoToPose
from Subsytem.limelight import limelight
from Subsytem.SwerveSubsystem import SwerveSubsystem
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from wpilib import SmartDashboard

class GoToRobotRelative(GoToPose):
    def __init__(self, swerve:SwerveSubsystem, x: float, y: float, heading: float,
                 controller: commands2.button.CommandXboxController):
        super().__init__(None, swerve, controller)
        self.swerve = swerve
        self.translation = Translation2d(x,y)
        self.heading = Rotation2d.fromDegrees(heading)


    def initialize(self):
        pose = self.swerve.getPose()
        t = pose.translation() + self.translation.rotateBy(-pose.rotation())
        r = pose.rotation().rotateBy(self.heading)
        self.pose = Pose2d(t,r)
        super().initialize()

    def execute(self):
        super().execute()

    def isFinished(self) -> bool:
        return super().isFinished()

    def end(self, interrupted: bool):
        super().end(interrupted)