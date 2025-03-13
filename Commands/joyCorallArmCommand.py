from commands2 import Command
from Subsytem.CorallArmSubsy import corralArmSubsys
from Constants import CoralSubsys
from commands2.button import CommandXboxController


class joycorralArmCommand(Command):
    def __init__(
        self, subsys: corralArmSubsys, controller: CommandXboxController
    ):
        self.subsys = subsys
        self.addRequirements(subsys)
        self.controller = controller
        super().__init__()

    def initialize(self):
        return super().initialize()

    def execute(self):
        self.subsys.duti(self.controller.getLeftY()*0.1)
        return super().execute()

    def isFinished(self):
        self.subsys.duti(0)
        return super().isFinished()

    def end(self, interrupted):
        return super().end(interrupted)
