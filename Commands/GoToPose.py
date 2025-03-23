import wpilib
from commands2 import Command
from Subsytem.SwerveSubsystem import SwerveSubsystem
from Constants import LimeLightConstants
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds
import commands2
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpilib import SmartDashboard
import math
import numpy
class GoToPose(Command):

    driveKp = 0.8
    omegaKp = 1.8

    def __init__(self, wantedPose: Pose2d, targetangle: Rotation2d, subsys: SwerveSubsystem,
                 controller: commands2.button.CommandXboxController, isAuto: bool):
        self.subsys = subsys
        self.wantedPose = wantedPose
        self.target = None  # Initialize as None, or set to Translation2d() if needed
        self.targetangle = targetangle
        self.isAuto = isAuto
        self.addRequirements(subsys)
        self.targetReached = False
        self.targetReachedStronger = False
        self.controller = controller
        super().__init__()

    def initialize(self):
        self.targetReached = False if self.subsys.getPose() is not None else True
        self.targetReachedStronger = False
        if not self.targetReached:
            pose = self.subsys.getPose()  # Get current pose from the subsystem

    def stopCommand(self):
        self.targetReached = True

    def execute(self):
        if (not self.isAuto):
            if (abs(self.controller.getLeftY()) >= 0.1 or abs(self.controller.getLeftX()) >= 0.1):
                print("ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff")
                self.targetReachedStronger = True
        
        if self.subsys.getPose() is not None:
            pose = self.subsys.getPose()
            # Corrected line: subtract the translation parts, not the whole pose
            self.target = self.wantedPose.translation() - pose.translation()
            self.targetangle = self.wantedPose.rotation() - pose.rotation()
            self.targetReached = abs(self.target.x) <= 0.02 and abs(self.target.y) <= 0.02
            #print("OMEGA VEL: ", math.degrees(self.targetangle.radians() * GoToPose.omegaKp ))
            vX = min(abs(self.target.x * GoToPose.driveKp), 2) *  numpy.sign(self.target.x)
            vY = min(abs(self.target.y * GoToPose.driveKp), 2) *  numpy.sign(self.target.y)
            self.subsys.setSpeeds(ChassisSpeeds(vX, vY, self.targetangle.radians() * GoToPose.omegaKp), 
                                correctColor=False)
                

    def isFinished(self) -> bool:
        return self.targetReached or self.targetReachedStronger

    def end(self, interrupted: bool):
        self.subsys.drive(0, 0, 0)
