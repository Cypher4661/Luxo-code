import wpilib
from commands2 import Command
from wpimath.kinematics import ChassisSpeeds
from Commands.GoToL3Tag import GoToL3Tag
from Commands.GoToDroL2Tag import GoToDropL2Tag
from Commands.GoToRobotRelative import GoToRobotRelative
from Constants import (OIConstants, SystemValues)
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
from Commands.L1U import l1U
import commands2
import commands2.cmd
import commands2.button
from Subsytem.CorallArmSubsy import corralArmSubsys
from Subsytem.AlgiArmSubsys import algiArmSubsys
from Subsytem.AlgiIntake import algiIntake
from Subsytem.CorralIntake import corralIntake
from commands2 import ParallelCommandGroup, ParallelDeadlineGroup
from Subsytem.limelight import limelight
from wpilib import SmartDashboard
from wpiutil import Sendable, SendableBuilder
from Commands.calibrateAlgeaArm import algiArmCalibrate
from Commands.AutoGOL3 import GoToL3TagAuto
from Commands.joyCorallArmCommand import joycorralArmCommand
from pathplannerlib.auto import NamedCommands
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.auto import PathPlannerAuto
from cscore import CameraServer as CS

class RobotContainer(Sendable):

    _isRed = False
    container = None

    def __init__(self, isDisabled):
        super().__init__()

        
        # Get the UsbCamera from CameraServer
        CS.startAutomaticCapture(0).setResolution(640, 480)

        # Random Data
        self.led_bool_enable = True
        self.led_bool_disable = True
        self.isDisabled = isDisabled

        # Controllers
        self.driverController = commands2.button.CommandXboxController(
            OIConstants.kDriverControllerPort
        )
        self.operatorController = commands2.button.CommandXboxController(
            OIConstants.kOperatorControllerPort
        )

        # Subsystems
        self.swerveSubsystem = SwerveSubsystem()
        self.corralArmSubsystem = corralArmSubsys()
        self.led_subsys = ledSubsys()
        self.algiIntakeSubsystem = algiIntake()
        self.corralIntakeSubsystem = corralIntake()
        self.algiArmSubsystem = algiArmSubsys()
        self.limelight = limelight(self.swerveSubsystem,self.led_subsys, self.swerveSubsystem.getVelocity, self.isDisabled)

        NamedCommands.registerCommand('Shoot L1', corralIntakeCommand(self.corralIntakeSubsystem, SystemValues.outputCorralPower))
        NamedCommands.registerCommand('AlgeaOutTake', algiArmCommand(self.algiArmSubsystem, SystemValues.ouputAlgiArmAngle, True).withTimeout(0.5))
        NamedCommands.registerCommand('AlgeaShoot',algiIntakeCommand(self.algiIntakeSubsystem, SystemValues.outputAlgiPower).withTimeout(2))
        NamedCommands.registerCommand('L3-goto', corralArmCommand(self.corralArmSubsystem, SystemValues.l3ArmAngle, True).schedule())
        NamedCommands.registerCommand('ShootCorral', corralIntakeCommand(self.corralIntakeSubsystem, SystemValues.outputCorralPower).withTimeout(1))
        SmartDashboard.putData('Swerve', self.swerveSubsystem)
        SmartDashboard.putData('vision', self.limelight)

        self.configure_commands()
        self.configure_button_bindings()        

    def configure_commands(self):

        # Commands
        self.specialIntakeCorralCommand = corralIntakeCommand(self.corralIntakeSubsystem, SystemValues.specialCorralIntakePower)
        self.intakeAlgiIntakeCommand = algiIntakeCommand(self.algiIntakeSubsystem, SystemValues.intakeAlgiPower)
        self.outTakeAlgiIntakeCommand = algiIntakeCommand(self.algiIntakeSubsystem, SystemValues.outputAlgiPower)
        self.intakeCorralIntakeCommand = corralIntakeCommand(self.corralIntakeSubsystem, SystemValues.intakeCorralPower)

        self.outputAlgiIntakeCommand = algiIntakeCommand(self.algiIntakeSubsystem, SystemValues.outputAlgiPower)
        self.outputCorralIntakeCommand = corralIntakeCommand(
            self.corralIntakeSubsystem, SystemValues.outputCorralPower
        )
        self.outputCorralIntakeCommand2 = corralIntakeCommand(self.corralIntakeSubsystem, -0.3)

        self.l3ArmCommand = corralArmCommand(self.corralArmSubsystem, SystemValues.l3ArmAngle, True)
        self.l2ArmCommand = corralArmCommand(self.corralArmSubsystem, SystemValues.l2ArmAngle, True)
        self.l0ArmCommand = corralArmCommand(self.corralArmSubsystem, 0, True)

        self.stopIntakeCorralIntakeCommand = corralIntakeCommand(
            self.corralIntakeSubsystem, 0
        )

        self.intakeCorralArmCommand = corralArmCommand(self.corralArmSubsystem, SystemValues.intakeCorralArmAngle, True)
        self.specialIntakeCorralArmCommand = corralArmCommand(self.corralArmSubsystem, SystemValues.specialCorralIntakeArmAngle, True)
        self.pickAlgiArmCommand = algiArmCommand(self.algiArmSubsystem, SystemValues.pickAlgiArmAngle, True)
        self.outputAlgiArmCommand = algiArmCommand(self.algiArmSubsystem, SystemValues.ouputAlgiArmAngle, True)

        self.led_command_green = ledCommand(self.led_subsys, [0, 200, 0])
        self.led_command_blue = ledCommand(self.led_subsys, [0, 0, 200])
        self.led_command_red = ledCommand(self.led_subsys, [200, 0, 0])
        self.led_command_purple = ledCommand(self.led_subsys, [255,0,255])
        self.led_command_cyan = ledCommand(self.led_subsys, [0, 200, 200])
        self.led_command_yellow = ledCommand(self.led_subsys, [240, 240, 0])

        self.led_command_flash_blue = LEDAnimationCommand(self.led_subsys, [0, 0, 200], [0, 0, 0], 0.1)
        self.led_command_flash_white = LEDAnimationCommand(self.led_subsys, [200, 200, 200], [0,0,0], 0.1)
        self.led_command_flash_purple = LEDAnimationCommand(self.led_subsys, [200, 0, 200], [0,0,0], 0.1)
        self.led_command_flash_cyan = LEDAnimationCommand(self.led_subsys, [0, 200, 200], [0,0,0], 0.1)

        # Deafult Commands
        self.deafultAlgiIntakeCommand = algiIntakeCommand(
            self.algiIntakeSubsystem, 0, True, self.operatorController
        )
        self.deafultCorralIntakeCommand = corralIntakeCommand(
            self.corralIntakeSubsystem, 0, True
        )
        self.defaultCorralArmCommand = corralArmCommand(
            self.corralArmSubsystem, 0, True
        )
        self.defaultAlgiArmCommand = algiArmCommand(self.algiArmSubsystem, 0, True)
        self.swerveCommand = SwerveDriveCommand(
            self.swerveSubsystem, self.driverController
        )
        self.led_subsys.change_color([240, 240, 0])
        self.joycoral = joycorralArmCommand(self.corralArmSubsystem, self.operatorController)

        # Set Default Commands
        self.swerveSubsystem.setDefaultCommand(self.swerveCommand)
        #self.algiArmSubsystem.setDefaultCommand(self.defaultAlgiArmCommand)
        self.corralArmSubsystem.setDefaultCommand(self.defaultCorralArmCommand)
        self.algiIntakeSubsystem.setDefaultCommand(self.deafultAlgiIntakeCommand)
        self.corralIntakeSubsystem.setDefaultCommand(self.deafultCorralIntakeCommand)

        # Command Groups
        self.intakeAlgiCommand = ParallelDeadlineGroup(
            self.intakeAlgiIntakeCommand, self.pickAlgiArmCommand, self.led_command_blue
        )
        self.intakeCorralCommand = ParallelDeadlineGroup(
            self.intakeCorralIntakeCommand, self.intakeCorralArmCommand, self.led_command_purple
        )
        self.specialCorralIntakeCommand = ParallelCommandGroup(
            self.specialIntakeCorralCommand, self.specialIntakeCorralArmCommand, self.led_command_yellow
        )
    def configure_button_bindings(self):

        # Driver Controller

        self.driverController.b().onTrue(
            commands2.cmd.runOnce(lambda: self.swerveSubsystem.zeroHeading()).ignoringDisable(True)
        )
        self.driverController.y().onTrue(
            commands2.cmd.runOnce(lambda: self.swerveSubsystem.check_module_angle())
        )
        self.driverController.a().toggleOnTrue(
            SlowSwerveDriveCommand(self.swerveSubsystem, self.driverController)
        )

        self.driverController.rightBumper().onTrue(GoToL3Tag(False, self.swerveSubsystem, self.limelight,
                                                             self.driverController))
        self.driverController.leftBumper().onTrue(GoToL3Tag(True, self.swerveSubsystem, self.limelight,
                                                            self.driverController))

        #Operator Controller

        self.operatorController.b().toggleOnTrue(self.intakeAlgiCommand)
        self.operatorController.a().toggleOnTrue(self.intakeCorralCommand)
        self.operatorController.povLeft().toggleOnTrue(self.stopIntakeCorralIntakeCommand)
        self.operatorController.rightBumper().toggleOnTrue(self.l3ArmCommand)
        self.operatorController.leftBumper().toggleOnTrue(self.l2ArmCommand)
        
        self.operatorController.x().toggleOnTrue(self.outputCorralIntakeCommand)
        self.operatorController.y().whileTrue(self.outputAlgiIntakeCommand)
        #self driver
        self.driverController.povDown().toggleOnTrue(self.specialCorralIntakeCommand) #single driver
        self.driverController.povUp().whileTrue(corralIntakeCommand(self.corralIntakeSubsystem, 0.2, True))
        self.driverController.povRight().toggleOnTrue(self.l0ArmCommand)
        self.operatorController.povDown().toggleOnTrue(self.specialCorralIntakeCommand)
        self.operatorController.povUp().toggleOnTrue(corralIntakeCommand(self.corralIntakeSubsystem, 0.2, True))
        self.operatorController.povRight().toggleOnTrue(self.l0ArmCommand)
        self.operatorController.rightTrigger().toggleOnTrue(self.outputAlgiArmCommand)
        self.operatorController.leftStick().toggleOnTrue(algiArmCalibrate(self.algiArmSubsystem))
        self.operatorController.leftTrigger().toggleOnTrue(self.defaultAlgiArmCommand)
        self.driverController.x(commands2.cmd.runOnce(lambda: self.corralArmSubsystem.reset_encoder()).ignoringDisable(True))
    def getYellowLEDCommand(self):
        return self.led_command_yellow
    def getRedLEDCommand(self):
        return self.led_command_red
    def getAlgiArmSubsys(self):
        return self.algiArmSubsystem
    def autoL3Command(self) -> Command:



        cmd = commands2.cmd.run(lambda: self.swerveSubsystem.setSpeedsRR(ChassisSpeeds(1.2,0 , 0)), self.swerveSubsystem).withTimeout(1)
        print("1")
        cmd = cmd.andThen(commands2.cmd.runOnce(lambda: self.swerveSubsystem.setSpeeds(ChassisSpeeds(0, 0, 0)), self.swerveSubsystem))
        '''
        print("2")
        cmd = cmd.andThen(self.outputAlgiArmCommand).withTimeout(0.5)
        print("3")
        #cmd = cmd.andThen(commands2.cmd.runOnce(lambda: algiArmCalibrate(self.algiArmSubsystem).withTimeout(1)))
        #cmd = cmd.andThen(commands2.cmd.run(lambda: self.swerveSubsystem.setSpeeds(ChassisSpeeds(0, 0, 0)), self.swerveSubsystem).withTimeout(1))
        cmd = cmd.andThen(algiArmCalibrate(self.algiArmSubsystem))
        print("4")
        cmd = cmd.andThen(commands2.cmd.run(lambda: self.swerveSubsystem.setSpeedsRR(ChassisSpeeds(0 ,- 0.5 , 0)), self.swerveSubsystem).withTimeout(0.7))
        cmd = cmd.andThen(commands2.cmd.run(lambda: self.swerveSubsystem.setSpeeds(ChassisSpeeds(0, 0, 0)), self.swerveSubsystem).withTimeout(0.5))
        cmd = cmd.andThen(commands2.cmd.run(lambda: self.swerveSubsystem.setSpeeds(ChassisSpeeds(0 ,0 , 1), False), self.swerveSubsystem).withTimeout(1.1))
        cmd = cmd.andThen(commands2.cmd.run(lambda: self.swerveSubsystem.setSpeeds(ChassisSpeeds(0, 0, 0)), self.swerveSubsystem).withTimeout(0.5))
        cmd = cmd.andThen(GoToL3TagAuto(True,self.swerveSubsystem, self.limelight).withTimeout(1.5))
        cmd = cmd.andThen(commands2.cmd.runOnce(lambda: corralArmCommand(self.corralArmSubsystem, SystemValues.l3ArmAngle, True).schedule()).withTimeout(1.8))
        cmd = cmd.andThen(commands2.cmd.run(lambda: self.swerveSubsystem.setSpeeds(ChassisSpeeds(0, 0, 0)), self.swerveSubsystem).withTimeout(2))
        cmd = cmd.andThen(commands2.cmd.runOnce(lambda: corralIntakeCommand(self.corralIntakeSubsystem, SystemValues.outputCorralPower).schedule()).withTimeout(3))'''
        return cmd  
    def autoL1Command(self) -> Command:
        cmd = commands2.cmd.run(lambda: self.swerveSubsystem.setSpeedsRR(ChassisSpeeds(-1.2,0 , 0)), self.swerveSubsystem).withTimeout(2)
        cmd = cmd.andThen(commands2.cmd.run(lambda: self.swerveSubsystem.setSpeedsRR(ChassisSpeeds(0 ,0 , 0)), self.swerveSubsystem).withTimeout(0.5))
        cmd = cmd.andThen(corralIntakeCommand(
            self.corralIntakeSubsystem, SystemValues.outputCorralPower
        ).withTimeout(1))
        return cmd
    
    def autoPathPlanner(self) -> Command: 
        AutoBuilder.configureHolonomic(
        self.swerveSubsystem.getEstimatedPose,
        self.swerveSubsystem.resetPose,
        self.swerveSubsystem.getChassisSpeeds,
        self.swerveSubsystem.drive,
        self.swerveSubsystem.kinematics,
        constraints,
        self.swerveSubsystem
    )
        return PathPlannerAuto('AlgeaL3')


    def get_autonomous_command(self) -> Command:
        return self.autoPathPlanner()
    

 

