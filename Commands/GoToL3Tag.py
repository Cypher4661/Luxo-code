import commands2.command
from Constants import LimeLightConstants
from Commands.GoToPose import GoToPose
from Subsytem.limelight import limelight
from Subsytem.SwerveSubsystem import SwerveSubsystem
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from wpilib import SmartDashboard

class GoToL3Tag(GoToPose):
    def __init__(self, left: bool, swerve: SwerveSubsystem, vision: limelight,
                 controller: commands2.button.CommandXboxController, isAuto: bool):
        # Initialize the parent class with the default values
        super().__init__(None, Rotation2d(0), swerve, controller, isAuto)
        self.left = left
        self.isAuto = isAuto
        
        self.swerve = swerve
        self.vision = vision


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
        super().initialize()

    def execute(self):
        # Call the parent class's execute method
        super().execute()

    def isFinished(self) -> bool:

        # Call the parent class's isFinished method
        return super().isFinished()

    def end(self, interrupted: bool):
        # Call the parent class's end method
        super().end(interrupted)
