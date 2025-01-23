import wpilib
import commands2
from RobotContainer import RobotContainer


class MyRobot(commands2.TimedCommandRobot):
    # robot
    def robotInit(self) -> None:
        self.scheduler = commands2.CommandScheduler.getInstance().enable()
        self.container = RobotContainer()
        self.container.swerveSubsystem.zeroHeading()
        self.container.swerveSubsystem.check_module_angle()
        self.container.getRedLEDCommand().schedule()

    def robotPeriodic(self) -> None:
        commands2.CommandScheduler.getInstance().run()

    # autonomus
    def autonomousInit(self) -> None:
        self.container.get_autonomous_command()
        pass

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
        self.container.getYellowLEDCommand().schedule()
        return super().disabledExit()


if __name__ == "__main__":
    wpilib.run(MyRobot)
