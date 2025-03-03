import wpilib
import commands2
from RobotContainer import RobotContainer
from wpilib import SmartDashboard
from Constants import SystemValues


class MyRobot(commands2.TimedCommandRobot):
    # robot
    def robotInit(self) -> None:
        self.scheduler = commands2.CommandScheduler.getInstance().enable()
        self.container = RobotContainer()
        RobotContainer.container = self.container
        #wpilib.SmartDashboard.putData('Robot', self.container)

        #SmartDashboard.putNumber("L3 Angle", SystemValues.l3ArmAngle)
        #SmartDashboard.putNumber("L2 Angle", SystemValues.l2ArmAngle)
        #SmartDashboard.putNumber("Intake Algi Power", SystemValues.intakeAlgiPower)
        #SmartDashboard.putNumber(
        #    "Special Corral Intake Power", SystemValues.specialCorralIntakePower
        #)
        #SmartDashboard.putNumber("Output Algi Power", SystemValues.outputAlgiPower)
        #SmartDashboard.putNumber("Intake Corral Power", SystemValues.intakeCorralPower)
        #SmartDashboard.putNumber("Output Corral Power", SystemValues.outputCorralPower)
        #SmartDashboard.putNumber(
        #    "Intake Corral Angle", SystemValues.intakeCorralArmAngle
        #)
        #SmartDashboard.putNumber(
        #    "Special Corral Intake Angle", SystemValues.specialCorralIntakeArmAngle
        #)
        #SmartDashboard.putNumber("Pick Algi Angle", SystemValues.pickAlgiArmAngle)
        #SmartDashboard.putNumber("Output Algi Angle", SystemValues.ouputAlgiArmAngle)

    def robotPeriodic(self) -> None:
        commands2.CommandScheduler.getInstance().run()


    # autonomus
    def autonomousInit(self) -> None:
        self.auto_command = self.container.get_autonomous_command()
        if self.auto_command != None:
            self.auto_command.schedule()

    def autonomousPeriodic(self) -> None:
        pass

    # teleoperated
    def teleopInit(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        print(self.container.swerveSubsystem.getDefaultCommand())
        pass

    def disabledInit(self) -> None:
        return super().disabledInit()

    def disabledExit(self) -> None:
        return super().disabledExit()


if __name__ == "__main__":
    wpilib.run(MyRobot)
