
import commands2
from Commands.GoToL3Tag import GoToL3Tag
from Commands.GoToPose import GoToPose
from Subsytem.SwerveSubsystem import SwerveSubsystem
from Subsytem.limelight import limelight
import commands2.command
from Constants import LimeLightConstants
from Commands.GoToPose import GoToPose
from Subsytem.limelight import limelight
from Subsytem.SwerveSubsystem import SwerveSubsystem
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from wpilib import SmartDashboard
import commands2.command
from Constants import LimeLightConstants
from Commands.GoToPose import GoToPose
from Subsytem.limelight import limelight
from Subsytem.SwerveSubsystem import SwerveSubsystem
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from wpilib import SmartDashboard
import wpilib
from commands2 import Command
from Subsytem.SwerveSubsystem import SwerveSubsystem
from Constants import LimeLightConstants
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds
import commands2
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpilib import SmartDashboard


class GoToL3TagAuto(commands2.Command):

    driveKp = 0.52
    omegaKp = 0.02

    def __init__(self, left: bool, swerve: SwerveSubsystem, vision: limelight):
        self.subsys = swerve
        self.wantedPose = None
        self.target = None  # Initialize as None, or set to Translation2d() if needed
        self.targetangle = Rotation2d(0)
        self.addRequirements(swerve)
        self.targetReached = False

        self.left = left
        
        self.swerve = swerve
        self.vision = vision
        super().__init__()
    
    def initialize(self):
        # Get the tag ID from the vision system
        tid = self.vision.getTagId()
        SmartDashboard.putNumber('Go To Tag Id', tid)
        
        if tid > 0:  # If the tag ID is valid
            # Set the target pose based on the tag ID and whether we are going to the left or right
            if self.left:
                self.wantedPose = LimeLightConstants.getLeftL3Position(tid)
            else:
                self.wantedPose = LimeLightConstants.getRightL3Position(tid)
            
            # Get the angle of the tag from the vision system and set it as the target angle
            self.targetangle = Rotation2d.fromDegrees(LimeLightConstants.getTagAngle(tid)).rotateBy(Rotation2d.fromDegrees(180))  # Set the tag's angle
        else:
            self.wantedPose = self.swerve.getPose()
            self.targetangle = self.swerve.getPose().rotation()
            
        
        # Initialize the parent GoToPose with the determined wantedPose and target angle
        self.targetReached = False if self.subsys.getPose() is not None else True
        if not self.targetReached:
            pose = self.subsys.getPose()  # Get current pose from the subsystem
    
    def execute(self):
        pose = self.subsys.getPose()
        
        # Corrected line: subtract the translation parts, not the whole pose
        self.target = self.wantedPose.translation() - pose.translation()
        self.targetangle = self.wantedPose.rotation() - self.targetangle
        self.targetReached = abs(self.target.x) < 0.02 and abs(self.target.y) < 0.02
        self.subsys.setSpeeds(ChassisSpeeds(self.target.x * GoToPose.driveKp, self.target.y * GoToPose.driveKp, ), correctColor=False)
    
    def isFinished(self) -> bool:
        return self.targetReached

    def end(self, interrupted: bool):
        self.subsys.drive(0, 0, 0)