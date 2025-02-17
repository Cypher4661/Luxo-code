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
        self.container.swerveSubsystem.zeroHeading()
        self.container.swerveSubsystem.check_module_angle()
        self.container.getRedLEDCommand().schedule()

        SmartDashboard.putNumber("L3 Angle", SystemValues.l3ArmAngle)
        SmartDashboard.putNumber("L2 Angle", SystemValues.l2ArmAngle)
        SmartDashboard.putNumber("Intake Algi Power", SystemValues.intakeAlgiPower)
        SmartDashboard.putNumber(
            "Special Corral Intake Power", SystemValues.specialCorralIntakePower
        )
        SmartDashboard.putNumber("Output Algi Power", SystemValues.outputAlgiPower)
        SmartDashboard.putNumber("Intake Corral Power", SystemValues.intakeCorralPower)
        SmartDashboard.putNumber("Output Corral Power", SystemValues.outputCorralPower)
        SmartDashboard.putNumber(
            "Intake Corral Angle", SystemValues.intakeCorralArmAngle
        )
        SmartDashboard.putNumber(
            "Special Corral Intake Angle", SystemValues.specialCorralIntakeArmAngle
        )
        SmartDashboard.putNumber("Pick Algi Angle", SystemValues.pickAlgiArmAngle)
        SmartDashboard.putNumber("Output Algi Angle", SystemValues.ouputAlgiArmAngle)

    def robotPeriodic(self) -> None:
        commands2.CommandScheduler.getInstance().run()

        SystemValues.l3ArmAngle = SmartDashboard.getNumber(
            "L3 Angle", SystemValues.l3ArmAngle
        )
        SystemValues.l2ArmAngle = SmartDashboard.getNumber(
            "L2 Angle", SystemValues.l2ArmAngle
        )
        SystemValues.intakeAlgiPower = SmartDashboard.getNumber(
            "Intake Algi Power", SystemValues.intakeAlgiPower
        )
        SystemValues.specialCorralIntakePower = SmartDashboard.getNumber(
            "Special Corral Intake Power", SystemValues.specialCorralIntakePower
        )
        SystemValues.outputAlgiPower = SmartDashboard.getNumber(
            "Output Algi Power", SystemValues.outputAlgiPower
        )
        SystemValues.intakeCorralPower = SmartDashboard.getNumber(
            "Intake Corral Power", SystemValues.intakeCorralPower
        )
        SystemValues.outputCorralPower = SmartDashboard.getNumber(
            "Output Corral Power", SystemValues.outputCorralPower
        )
        SystemValues.intakeCorralArmAngle = SmartDashboard.getNumber(
            "Intake Corral Angle", SystemValues.intakeCorralArmAngle
        )
        SystemValues.specialCorralIntakeArmAngle = SmartDashboard.getNumber(
            "Special Corral Intake Angle", SystemValues.specialCorralIntakeArmAngle
        )
        SystemValues.pickAlgiArmAngle = SmartDashboard.getNumber(
            "Pick Algi Angle", SystemValues.pickAlgiArmAngle
        )
        SystemValues.ouputAlgiArmAngle = SmartDashboard.getNumber(
            "Output Algi Angle", SystemValues.ouputAlgiArmAngle
        )

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
        return super().teleopPeriodic()

    def disabledInit(self) -> None:
        self.container.getRedLEDCommand().schedule()
        return super().disabledInit()

    def disabledExit(self) -> None:
        return super().disabledExit()


if __name__ == "__main__":
    wpilib.run(MyRobot)
