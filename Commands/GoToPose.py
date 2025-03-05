import wpilib
from commands2 import Command
from Subsytem.SwerveSubsystem import SwerveSubsystem
from Constants import LimeLightConstants
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds
import commands2
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpilib import SmartDashboard

class GoToPose(Command):

    driveKp = 2
    omegaKp = 0.02

    def __init__(self, wantedPose: Translation2d, targetangle: Rotation2d, subsys: SwerveSubsystem,
                 controller: commands2.button.CommandXboxController):
        self.subsys = subsys
        self.wantedPose = wantedPose
        self.target = None  # Initialize as None, or set to Translation2d() if needed
        self.targetangle = targetangle
        self.addRequirements(subsys)
        self.targetReached = False
        self.controller = controller
        super().__init__()

    def initialize(self):
        self.targetReached = False if self.subsys.getPose() is not None else True
        if not self.targetReached:
            pose = self.subsys.getPose()  # Get current pose from the subsystem


    def execute(self):
        # Stop if driver is driving
        if self.vision.getTagId() > 0:
            
            if (abs(self.controller.getLeftY()) > 0.1 or
            abs(self.controller.getLeftX()) > 0.1):
                self.targetReached = True
            elif self.subsys.getPose() is not None:
                pose = self.subsys.getPose()
                # Corrected line: subtract the translation parts, not the whole pose
                self.target = self.wantedPose.translation() - pose.translation()
                self.targetangle = self.wantedPose.rotation() - self.targetangle
                self.targetReached = abs(self.target.x) < 0.02 and abs(self.target.y) < 0.02 and abs(self.targetangle.degrees()) < 3
                self.subsys.setSpeeds(ChassisSpeeds(self.target.x * GoToPose.driveKp,
                                                    self.target.y * GoToPose.driveKp,
                                                    -self.controller.getRightX()), 
                                    correctColor=False)

    def isFinished(self) -> bool:
        return self.targetReached or self.vision.getTagId() == 0

    def end(self, interrupted: bool):
        self.subsys.drive(0, 0, 0)
