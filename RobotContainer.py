from commands2 import Command
from Constants import (
    OIConstants,
)
from Subsytem.SwerveSubsystem import SwerveSubsystem
from Subsytem.LEDSubsys import ledSubsys
from Commands.LEDCommand import ledCommand
from Commands.CorallArmCommand import corralArmCommand
from Commands.SlowSwerveDriveCommand import SlowSwerveDriveCommand
from Commands.SwerveDriveCommand import SwerveDriveCommand
from Commands.AlgiArmCommand import algiArmCommand
from Commands.LEDAnimationComand import LEDAnimationCommand
from Commands.AlgiIntakeCommand import algiIntakeCommand
from Commands.CorralIntakeCommand import corralIntakeCommand
import commands2
import commands2.cmd
import commands2.button
from pathplannerlib.path import (
    PathPlannerPath,
    PathConstraints,
    GoalEndState,
    RotationTarget,
    Waypoint,
)
from pathplannerlib.auto import AutoBuilder
from Subsytem.limelight import limelight
from Subsytem.CorallArmSubsy import corralArmSubsys
from Subsytem.AlgiArmSubsys import algiArmSubsys
from Subsytem.AlgiIntake import algiIntake
from Subsytem.CorralIntake import corralIntake
from wpimath.geometry import Pose2d, Rotation2d
import math
from commands2 import SequentialCommandGroup
from commands2 import ParallelCommandGroup

class RobotContainer:
    def __init__(self):
        # Random Data
        self.led_bool_enable = True
        self.led_bool_disable = True

        # Controllers
        self.driverController = commands2.button.CommandXboxController(
            OIConstants.kDriverControllerPort
        )
        self.operatorController = commands2.button.CommandXboxController(
            OIConstants.kOperatorControllerPort
        )

        # Subsystems
        self.corralArmSubsystem = corralArmSubsys()
        self.swerveSubsystem = SwerveSubsystem()
        self.led_subsys = ledSubsys()
        self.limelight = limelight()
        self.left = self.limelight.get_left_limelight()
        self.right = self.limelight.get_right_limelight()
        self.algiIntakeSubsystem = algiIntake()
        self.corralIntakeSubsystem = corralIntake()
        self.algiArmSubsystem = algiArmSubsys()
        
        # Commands
        self.intakeAlgiIntakeCommand = algiIntakeCommand(self.algiIntakeSubsystem, 0.5)
        self.outTakeAlgiIntakeCommand = algiIntakeCommand(self.algiIntakeSubsystem, -1)
        self.intakeCorralIntakeCommand = corralIntakeCommand(self.corralIntakeSubsystem, 0.5)
        self.outputAlgiIntakeCommand = algiIntakeCommand(self.algiIntakeSubsystem, -0.5)
        self.outputCorralIntakeCommand = corralIntakeCommand(self.corralIntakeSubsystem, -0.15)
        self.l3ArmCommand = corralArmCommand(self.corralArmSubsystem, 135,True)
        self.l2ArmCommand = corralArmCommand(self.corralArmSubsystem, 165, True)
        self.intakeCorralArmCommand = corralArmCommand(self.corralArmSubsystem, 42.5)
        self.pickAlgiArmCommand = algiArmCommand(self.algiArmSubsystem, 45)
        self.led_command_green = ledCommand(self.led_subsys, [0, 255, 0])
        self.led_command_blue = ledCommand(self.led_subsys, [0, 0, 255])
        self.led_command_red = ledCommand(self.led_subsys, [255, 0, 0])
        self.led_animmation_command = LEDAnimationCommand(
            self.led_subsys, [255, 0, 0], [255, 0, 100]
        )
        self.led_command_yellow = ledCommand(self.led_subsys, [255, 0, 100])

        #Deafult Commands
        self.deafultAlgiIntakeCommand = algiIntakeCommand(self.algiIntakeSubsystem, 0, True)
        self.deafultCorralIntakeCommand = corralIntakeCommand(self.corralIntakeSubsystem, 0, True)
        self.defaultCorralArmCommand = corralArmCommand(self.corralArmSubsystem, 0, True)
        self.defaultAlgiArmCommand = algiArmCommand(self.algiArmSubsystem, 0, True)
        self.swerveCommand = SwerveDriveCommand(
            self.swerveSubsystem, self.driverController
        )

        #Set Default Commands
        self.swerveSubsystem.setDefaultCommand(self.swerveCommand)
        self.algiArmSubsystem.setDefaultCommand(self.defaultAlgiArmCommand)
        self.corralArmSubsystem.setDefaultCommand(self.defaultCorralArmCommand)
        self.algiIntakeSubsystem.setDefaultCommand(self.deafultAlgiIntakeCommand)
        self.corralIntakeSubsystem.setDefaultCommand(self.deafultCorralIntakeCommand)

        # Command Groups
        self.intakeAlgiCommand = ParallelCommandGroup(self.intakeAlgiIntakeCommand, self.pickAlgiArmCommand)
        self.outtakeAlgiCommand = self.outTakeAlgiIntakeCommand.withTimeout(2)
        self.intakeCorralCommand = ParallelCommandGroup(self.intakeCorralIntakeCommand, self.intakeCorralArmCommand)

        self.configure_button_bindings()


    def configure_button_bindings(self):
        self.driverController.b().onTrue(
            commands2.cmd.runOnce(lambda: self.swerveSubsystem.zeroHeading())
        )
        self.driverController.y().onTrue(
            commands2.cmd.runOnce(lambda: self.swerveSubsystem.check_module_angle())
        )

        self.driverController.a().toggleOnTrue(
            SlowSwerveDriveCommand(self.swerveSubsystem, self.driverController)
        )

        self.operatorController.b().toggleOnTrue(self.intakeAlgiCommand)
        self.operatorController.a().toggleOnTrue(self.intakeCorralCommand)
        self.operatorController.rightBumper().toggleOnTrue(self.l3ArmCommand)
        self.operatorController.leftBumper().toggleOnTrue(self.l2ArmCommand)
        self.operatorController.x().toggleOnTrue(self.outputCorralIntakeCommand)
        self.operatorController.y().toggleOnTrue(self.outtakeAlgiCommand)

    def getYellowLEDCommand(self):
        return self.led_command_yellow

    def getRedLEDCommand(self):
        return self.led_command_red

    def create_path_command(
        self,
        bezierPoints: list[Waypoint],
        goalEndState: GoalEndState,
        rotaionTargets: list[RotationTarget] = [],
    ) -> Command:
        path = PathPlannerPath(
            bezierPoints,
            PathConstraints(3.0, 3.0, 2 * math.pi, 4 * math.pi),
            None,
            goalEndState,
        )
        path_follower_command: Command = AutoBuilder.followPath(path)
        return path_follower_command

    def get_autonomous_command(self) -> Command:
        self.swerveSubsystem.resetOdometry(Pose2d(0, 0, Rotation2d.fromDegrees(0)))
        self.swerveSubsystem.zeroHeading()
        bezierPoints = PathPlannerPath.waypointsFromPoses(
            [
                Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                Pose2d(0.5, 0, Rotation2d.fromDegrees(0)),
            ]
        )
        goalEndState = GoalEndState(0.0, Rotation2d.fromDegrees(0))
        path_follower_command = self.create_path_command(bezierPoints, goalEndState)
        return path_follower_command

    def april_tag_path(self) -> Command:
        april_light = self.limelight.get_left_limelight()
        april_pose = self.limelight.get_target_position(april_light)
        self.swerveSubsystem.resetOdometry(Pose2d(0, 0, Rotation2d.fromDegrees(0)))
        bezirePoints = PathPlannerPath.waypointsFromPoses(
            [
                Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                Pose2d(april_pose[0], april_pose[1], april_pose[2]),
            ]
        )
        path_follower_command = self.create_path_command(
            bezirePoints, GoalEndState(0, Rotation2d.fromDegrees(april_pose[2]))
        )
        return path_follower_command
