from wpimath.geometry import Pose2d, Rotation2d
from limelight import limelight

class PathPlanner:
    def __init__(self):
        self.current_position = Pose2d()
        self.limelight_subsystem = limelight()

    def updatePositionWithLimelight(self):
        self.current_position = self.limelight_subsystem.getRobotPosition()

    def planPath(self, target_position: Pose2d):
        self.updatePositionWithLimelight()
        # Use self.current_position to plan the path to target_position
        # ...existing path planning logic...
        