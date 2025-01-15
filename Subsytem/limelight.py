from ntcore import NetworkTableInstance
from commands2 import Subsystem
from pathplannerlib.pathfinders import Tuple
from Constants import LimeLightConstants
import math
from ntcore import NetworkTable


class limelight(Subsystem):
    def __init__(self) -> None:
        self.light_left = NetworkTableInstance.getDefault().getTable(
            LimeLightConstants.limelight_left_name
        )
        self.light_right = NetworkTableInstance.getDefault().getTable(
            LimeLightConstants.limelight_right_name
        )

    def getX(self, light: NetworkTable) -> float:
        return light.getNumber("tx", 999)

    def getY(self, light: NetworkTable) -> float:
        return light.getNumber("ty", 999)

    def distance(self, light, l_height, t_height, angle) -> float:
        angle_to_goal_degrees = angle + self.getY(light)
        angle_to_goal_radians = math.radians(angle_to_goal_degrees)
        distance = (t_height - l_height) / math.tan(angle_to_goal_radians)
        return distance

    def id(self, light: NetworkTable) -> int:
        return light.getNumber("tid", 999)

    def inView(self, light: NetworkTable) -> bool:
        return bool(light.getNumber("tv", 0))

    def get_right_limelight(self) -> NetworkTable:
        return self.light_right

    def get_left_limelight(self) -> NetworkTable:
        return self.light_left

    def get_target_position(self, light: NetworkTable) -> Tuple:
        april_pose = light.getNumberArray("camerapose_targetspace", [999] * 6)
        x = april_pose[0]
        y = april_pose[2]
        yaw = april_pose[4]
        return (x, y, yaw)
