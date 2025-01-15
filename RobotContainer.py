from commands2 import Command, SequentialCommandGroup
from Constants import (
    OIConstants,
)
from Subsytem.SwerveSubsystem import SwerveSubsystem
from Subsytem.LEDSubsys import ledSubsys
from Commands.LEDCommand import ledCommand
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
        self.swerveSubsystem = SwerveSubsystem()
        self.limelight = limelight()
        self.left = self.limelight.get_left_limelight()
        self.right = self.limelight.get_right_limelight()
        self.driverController = commands2.button.CommandXboxController(
            OIConstants.kDriverControllerPort
        )
        self.operatorController = commands2.button.CommandXboxController(
            OIConstants.kOperatorControllerPort
        )
        self.swerveSubsystem.setDefaultCommand(
            SwerveDriveCommand(self.swerveSubsystem, self.driverController)
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

        self.driverController.a().toggleOnTrue(self.led_animmation_command)
        self.driverController.a().toggleOnFalse(self.led_command_yellow)

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
            rotaionTargets,
        )
        path_follower_command: Command = AutoBuilder.followPath(path)
        return path_follower_command

    def get_autonomous_command(self) -> Command:
        bezierPoints = PathPlannerPath.waypointsFromPoses(
            [
                Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                Pose2d(0, 1, Rotation2d.fromDegrees(0)),
            ]
        )
        path_follower_command = self.create_path_command(
            bezierPoints, GoalEndState(0, Rotation2d.fromDegrees(0))
        )
        command_group = SequentialCommandGroup()
        command_group.addCommands(path_follower_command)
        return command_group

    def april_tag_path(self) -> Command:
        april_light = self.limelight.get_left_limelight()
        april_pose = self.limelight.get_target_position(april_light)
        self.swerveSubsystem.resetOdometry(Pose2d(0,0, Rotation2d(0)))
        bezirePoints = PathPlannerPath.waypointsFromPoses(
            [
                Pose2d(0,0,Rotation2d.fromDegrees(0)),
                Pose2d(april_pose[0], april_pose[1], april_pose[2])
            ]
        )
        path_follower_command = self.create_path_command(
            bezirePoints,
            GoalEndState(0, Rotation2d(april_pose[2]))
        )
        return path_follower_command
