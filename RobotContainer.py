from commands2 import Command
from Constants import (
    OIConstants,
)
from Subsytem.SwerveSubsystem import SwerveSubsystem
from Subsytem.LEDSubsys import ledSubsys
from Commands.LEDCommand import ledCommand
from Commands.SlowSwerveDriveCommand import SlowSwerveDriveCommand
from Commands.SwerveDriveCommand import SwerveDriveCommand
from Commands.LEDAnimationComand import LEDAnimationCommand
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
from wpimath.geometry import Pose2d, Rotation2d
import math

class RobotContainer:
    def __init__(self):
        self.led_bool_enable = True
        self.led_bool_disable = True
        self.led_subsys = ledSubsys()
        self.driverController = commands2.button.CommandXboxController(
            OIConstants.kDriverControllerPort
        )
        self.swerveSubsystem = SwerveSubsystem()
        self.swerveCommand = SwerveDriveCommand(self.swerveSubsystem, self.driverController)
        self.limelight = limelight()
        self.left = self.limelight.get_left_limelight()
        self.right = self.limelight.get_right_limelight()
        self.operatorController = commands2.button.CommandXboxController(
            OIConstants.kOperatorControllerPort
        )
        self.swerveSubsystem.setDefaultCommand(
            self.swerveCommand
        )

        self.led_command_green = ledCommand(self.led_subsys, [0, 255, 0])
        self.led_command_blue = ledCommand(self.led_subsys, [0, 0, 255])
        self.led_command_red = ledCommand(self.led_subsys, [255, 0, 0])
        self.led_animmation_command = LEDAnimationCommand(
            self.led_subsys, [255, 0, 0], [255, 0, 100]
        )
        self.led_command_yellow = ledCommand(self.led_subsys, [255, 0, 100])

        self.configure_button_bindings()

    def configure_button_bindings(self):
        # reset robots heading
        self.driverController.b().onTrue(
            commands2.cmd.runOnce(lambda: self.swerveSubsystem.zeroHeading())
        )

        self.driverController.x().onTrue(
            commands2.cmd.runOnce(lambda: self.swerveSubsystem.change_drive(True))
        )
        self.driverController.x().onFalse(
            commands2.cmd.runOnce(lambda: self.swerveSubsystem.change_drive(False))
        )
        self.driverController.y().onTrue(
            commands2.cmd.runOnce(lambda: self.swerveSubsystem.check_module_angle())
        )

        self.driverController.a().toggleOnTrue(
            SlowSwerveDriveCommand(self.swerveSubsystem, self.driverController)
        )

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
        self.swerveSubsystem.resetOdometry(Pose2d(0,0,Rotation2d.fromDegrees(0)))
        self.swerveSubsystem.zeroHeading()
        bezierPoints = PathPlannerPath.waypointsFromPoses(
            [
                Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                Pose2d(0.5, 0, Rotation2d.fromDegrees(0)),
            ]
        )
        goalEndState = GoalEndState(0.0, Rotation2d.fromDegrees(0))
        path = PathPlannerPath(
            waypoints=bezierPoints,
            constraints=PathConstraints(3.0, 3.0, 2 * math.pi, 4 * math.pi),
            ideal_starting_state=None,
            goal_end_state=goalEndState

        )

        path_follower_command: Command = AutoBuilder.followPath(path)
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
