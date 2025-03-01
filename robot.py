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

    def robotPeriodic(self) -> None:
        commands2.CommandScheduler.getInstance().run()

    # autonomus
    def autonomousInit(self) -> None:
        self.auto_command = self.container.get_autonomous_command()
        if self.auto_command:
            self.auto_command.schedule()

    def autonomousPeriodic(self) -> None:
        pass

    # teleoperated
    def teleopInit(self) -> None:
        if self.auto_command:
            self.auto_command.cancel()

    def teleopPeriodic(self) -> None:
        pass

    def disabledInit(self) -> None:
        pass

    def disabledExit(self) -> None:
        pass


if __name__ == "__main__":
    wpilib.run(MyRobot)
