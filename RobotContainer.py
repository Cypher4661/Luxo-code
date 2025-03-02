from __future__ import annotations
import wpilib
from commands2 import Command
from Constants import OIConstants, SystemValues
from Subsytem.SwerveSubsystem import SwerveSubsystem
from Subsytem.LEDSubsys import ledSubsys
from Subsytem.limelight import limelight
from Commands.LEDCommand import ledCommand
from Commands.CorallArmCommand import corralArmCommand
from Commands.SlowSwerveDriveCommand import SlowSwerveDriveCommand
from Commands.AutoAlign import AutoAlign
from Commands.SwerveDriveCommand import SwerveDriveCommand
from Commands.AlgaeArmCommand import AlgaeArmCommand
from Commands.LEDAnimationComand import LEDAnimationCommand
from Commands.AlgaeIntakeCommand import AlgaeIntakeCommand
from Commands.CorralIntakeCommand import corralIntakeCommand
from Commands.udiAuto import UdiAuto
from Commands.GoToL3Tag import GoToL3Tag
import commands2
import commands2.cmd
import commands2.button
from Subsytem.CorallArmSubsy import corralArmSubsys
from Subsytem.AlgaeArmSubsys import AlgaeArmSubsys
from Subsytem.AlgaeIntakeSubsystem import AlgaeIntakeSubsystem
from Subsytem.CorralIntake import corralIntake
from wpiutil import Sendable
from commands2 import SequentialCommandGroup, ParallelCommandGroup, ParallelDeadlineGroup


class RobotContainer(Sendable):

    container: RobotContainer
    isRed = False

    def __init__(self):
        super().__init__()
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
        self.AlgaeIntakeSubsystem = AlgaeIntakeSubsystem()
        self.corralIntakeSubsystem = corralIntake()
        self.AlgaeArmSubsystem = AlgaeArmSubsys()
        self.limelight = limelight(self.swerveSubsystem.odometer, self.swerveSubsystem.getVelocity)

        self.configure_commands()
        self.configure_button_bindings()
        RobotContainer.container = self
        wpilib.SmartDashboard.putData(self)

    def configure_commands(self):
        # Commands
        self.specialIntakeCorralCommand = corralIntakeCommand(self.corralIntakeSubsystem,
                                                              SystemValues.specialCorralIntakePower)
        self.intakeAlgaeIntakeCommand = AlgaeIntakeCommand(self.AlgaeIntakeSubsystem, SystemValues.intakeAlgaePower)
        self.outTakeAlgaeIntakeCommand = AlgaeIntakeCommand(self.AlgaeIntakeSubsystem, SystemValues.outputAlgaePower)
        self.intakeCorralIntakeCommand = corralIntakeCommand(
            self.corralIntakeSubsystem, SystemValues.intakeCorralPower
        )

        self.outputAlgaeIntakeCommand = AlgaeIntakeCommand(self.AlgaeIntakeSubsystem, SystemValues.outputAlgaePower)
        self.outputCorralIntakeCommand = corralIntakeCommand(
            self.corralIntakeSubsystem, SystemValues.outputCorralPower
        )

        self.l3ArmCommand = corralArmCommand(self.corralArmSubsystem, SystemValues.l3ArmAngle, True)
        self.l2ArmCommand = corralArmCommand(self.corralArmSubsystem, SystemValues.l2ArmAngle, True)
        self.l0ArmCommand = corralArmCommand(self.corralArmSubsystem, 0, True)

        self.stopIntakeCorralIntakeCommand = corralIntakeCommand(
            self.corralIntakeSubsystem, 0
        )

        self.intakeCorralArmCommand = corralArmCommand(self.corralArmSubsystem, SystemValues.intakeCorralArmAngle, True)
        self.specialIntakeCorralArmCommand = corralArmCommand(self.corralArmSubsystem,
                                                              SystemValues.specialCorralIntakeArmAngle, True)
        self.pickAlgaeArmCommand = AlgaeArmCommand(self.AlgaeArmSubsystem, SystemValues.pickAlgaeArmAngle, True)
        self.outputAlgaeArmCommand = AlgaeArmCommand(self.AlgaeArmSubsystem, SystemValues.ouputAlgaeArmAngle, True)

        self.led_command_green = ledCommand(self.led_subsys, [0, 200, 0])
        self.led_command_blue = ledCommand(self.led_subsys, [0, 0, 200])
        self.led_command_red = ledCommand(self.led_subsys, [200, 0, 0])
        self.led_command_purple = ledCommand(self.led_subsys, [200, 0, 200])
        self.led_command_cyan = ledCommand(self.led_subsys, [0, 200, 200])
        self.led_command_yellow = ledCommand(self.led_subsys, [200, 0, 200])

        self.led_command_flash_blue = LEDAnimationCommand(self.led_subsys, [0, 0, 200], [0, 0, 0], 0.1)
        self.led_command_flash_white = LEDAnimationCommand(self.led_subsys, [200, 200, 200], [0, 0, 0], 0.1)
        self.led_command_flash_purple = LEDAnimationCommand(self.led_subsys, [200, 0, 200], [0, 0, 0], 0.1)
        self.led_command_flash_cyan = LEDAnimationCommand(self.led_subsys, [0, 200, 200], [0, 0, 0], 0.1)

        # Deafult Commands
        self.deafultAlgaeIntakeCommand = AlgaeIntakeCommand(
            self.AlgaeIntakeSubsystem, 0, True, self.operatorController
        )
        self.deafultCorralIntakeCommand = corralIntakeCommand(
            self.corralIntakeSubsystem, 0, True
        )
        self.defaultCorralArmCommand = corralArmCommand(
            self.corralArmSubsystem, 0, True
        )
        self.defaultAlgaeArmCommand = AlgaeArmCommand(self.AlgaeArmSubsystem, 0, True)
        self.swerveCommand = SwerveDriveCommand(
            self.swerveSubsystem, self.driverController
        )
        self.led_subsys.setDefaultCommand(self.led_command_yellow)

        # Set Default Commands
        self.swerveSubsystem.setDefaultCommand(self.swerveCommand)
        self.AlgaeArmSubsystem.setDefaultCommand(self.defaultAlgaeArmCommand)
        self.corralArmSubsystem.setDefaultCommand(self.defaultCorralArmCommand)
        self.AlgaeIntakeSubsystem.setDefaultCommand(self.deafultAlgaeIntakeCommand)
        self.corralIntakeSubsystem.setDefaultCommand(self.deafultCorralIntakeCommand)

        # Command Groups
        self.intakeAlgaeCommand = ParallelDeadlineGroup(
            self.intakeAlgaeIntakeCommand, self.pickAlgaeArmCommand, self.led_command_flash_blue
        )
        self.intakeCorralCommand = ParallelDeadlineGroup(
            self.intakeCorralIntakeCommand, self.intakeCorralArmCommand, self.led_command_flash_purple
        )
        self.specialCorralIntakeCommand = ParallelCommandGroup(
            self.specialIntakeCorralCommand, self.specialIntakeCorralArmCommand, self.led_command_flash_white
        )

    def configure_button_bindings(self):
        # Driver Controller

        self.driverController.b().onTrue(
            commands2.cmd.runOnce(lambda: self.swerveSubsystem.zeroHeading())
        )
        self.driverController.a().toggleOnTrue(
            SlowSwerveDriveCommand(self.swerveSubsystem, self.driverController)
        )
        self.driverController.x().onTrue(
            AutoAlign(self.swerveSubsystem, self.driverController)
        )
        self.driverController.rightBumper().onTrue(GoToL3Tag(False, self.swerveSubsystem, self.limelight,
                                                             self.driverController))
        self.driverController.leftBumper().onTrue(GoToL3Tag(True, self.swerveSubsystem, self.limelight,
                                                            self.driverController))

        # Operator Controller

        self.operatorController.b().toggleOnTrue(self.intakeAlgaeCommand)
        self.operatorController.a().toggleOnTrue(self.intakeCorralCommand)
        self.operatorController.povLeft().toggleOnTrue(self.stopIntakeCorralIntakeCommand)
        self.operatorController.rightBumper().toggleOnTrue(self.l3ArmCommand)
        self.operatorController.leftBumper().toggleOnTrue(self.l2ArmCommand)

        self.driverController.rightBumper().toggleOnTrue(self.l3ArmCommand)
        self.driverController.leftBumper().toggleOnTrue(self.l2ArmCommand)

        self.operatorController.x().toggleOnTrue(self.outputCorralIntakeCommand)
        self.operatorController.y().toggleOnTrue(self.outputAlgaeIntakeCommand)
        self.operatorController.povDown().toggleOnTrue(self.specialCorralIntakeCommand)
        self.operatorController.povUp().toggleOnTrue(corralIntakeCommand(self.corralIntakeSubsystem, 0.2, True))
        self.operatorController.povRight().toggleOnTrue(self.l0ArmCommand)
        self.operatorController.rightTrigger().toggleOnTrue(self.outputAlgaeArmCommand)

    def getYellowLEDCommand(self):
        return self.led_command_yellow

    def getRedLEDCommand(self):
        return self.led_command_red

    def getAlgaeArmSubsys(self):
        return self.AlgaeArmSubsystem

    def get_autonomous_command(self) -> Command:
        return UdiAuto(self.swerveSubsystem)

    def setRed(isRed:bool):
        RobotContainer.isRed = isRed

    def initSendable(self, builder):
        builder.addBooleanProperty('Is Red', lambda : RobotContainer.isRed, lambda x: RobotContainer.setRed(x))
