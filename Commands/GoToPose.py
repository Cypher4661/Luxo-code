import wpilib
from commands2 import Command
from Subsytem.SwerveSubsystem import SwerveSubsystem
from Constants import LimeLightConstants
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds


class GoToPose(Command):

    driveKp = 2
    omegaKp = 0.1

    def __init__(self, pose: Pose2d, subsys: SwerveSubsystem):
        self.subsys = subsys
        self.pose = pose
        self.addRequirements(subsys)
        self.targetReached = False
        super().__init__()

    def initialize(self):
        self.targetReached = False

    def execute(self):
        pose = self.subsys.getPose()
        error = self.pose.relativeTo(pose)
        t = error.translation()
        x = t.x * self.driveKp
        y = t.y * self.driveKp
        r = error.rotation().degrees()
        omega = r * self.omegaKp
        self.targetReached = abs(t.x) < 0.02 and abs(t.y) < 0.02 and abs(r) < 3
        self.subsys.autoDrive(ChassisSpeeds(x,y,omega))

    def isFinished(self) -> bool:
        return self.targetReached

    def end(self, interrupted: bool):
        self.subsys.drive(0,0,0)
