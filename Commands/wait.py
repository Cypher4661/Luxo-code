from commands2 import Command
from wpilib import Timer


class waitCommand(Command):
    def __init__(self, delay):
        self.delay = delay
        super().__init__()

    def initialize(self):
        self.time = Timer()
        self.time.start()
        return super().initialize()

    def execute(self):
        return super().execute()

    def isFinished(self) -> bool:
        return self.time.get() >= self.delay
