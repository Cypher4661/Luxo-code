import wpilib
from commands2 import Command
from Subsytem.SwerveSubsystem import SwerveSubsystem
from Constants import LimeLightConstants
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds
import commands2
from wpilib import SmartDashboard


class GoToPose(Command):

    driveKp = 0.4
    omegaKp = 0.1

    def __init__(self, pose: Pose2d | None, subsys: SwerveSubsystem,
                 controller: commands2.button.CommandXboxController):
        self.subsys = subsys
        self.pose = pose
        self.addRequirements(subsys)
        self.targetReached = False
        self.controller = controller
        super().__init__()

    def initialize(self):
        self.targetReached = False if self.pose != None else True
        if not self.targetReached:
            SmartDashboard.putNumber('Go/X', self.pose.translation().x)
            SmartDashboard.putNumber('Go/Y', self.pose.translation().y) 
            SmartDashboard.putNumber('Go/Deg', self.pose.rotation().degrees())
            pose = self.subsys.getPose()
            SmartDashboard.putNumber('Go/from X', pose.translation().x)
            SmartDashboard.putNumber('Go/from Y', pose.translation().y) 
            SmartDashboard.putNumber('Go/from Deg', pose.rotation().degrees())
            error = self.pose.relativeTo(pose)
            SmartDashboard.putNumber('Go/error X', error.translation().x)
            SmartDashboard.putNumber('Go/error Y', error.translation().y) 
            SmartDashboard.putNumber('Go/error Deg', error.rotation().degrees())

    def execute(self):
        # stop if driver is driving
        if (abs(self.controller.getLeftY()) > 0.1 or
           abs(self.controller.getLeftX()) > 0.1 or
           abs(self.controller.getRightX()) > 0.1):
            self.targetReached = True
        elif self.pose != None:
            pose = self.subsys.getPose()
            error = self.pose.relativeTo(pose)
            t = error.translation()
            x = t.x
            y = t.y
            deg = error.rotation().degrees() 
            self.targetReached = abs(x) < 0.02 and abs(y) < 0.02 and abs(deg) < 3
            self.subsys.setSpeeds(ChassisSpeeds(x*GoToPose.driveKp, y*GoToPose.driveKp, -deg*GoToPose.omegaKp), correctColor=False)

    def isFinished(self) -> bool:
        return self.targetReached

    def end(self, interrupted: bool):
        self.subsys.drive(0,0,0)