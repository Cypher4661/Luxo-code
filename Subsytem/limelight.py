from ntcore import NetworkTableInstance
from commands2 import Subsystem
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
        angle_to_goal_radians = angle_to_goal_degrees * (3.14159 / 180)
        distance = (t_height - l_height) / math.tan(angle_to_goal_radians)
        return distance

    def id(self, light: NetworkTable) -> int:
        return light.getNumber("tid", 999)

    def inView(self, light: NetworkTable) -> bool:
        return bool(light.getNumber("tv", 0))

    def inViewLeft(self) -> bool:
        return self.inView(self.light_left)

    def inViewRight(self) -> bool:
        return self.inView(self.light_right)

    def getXLeft(self) -> float:
        return self.getX(self.light_left)

    def getXRight(self) -> float:
        return self.getX(self.light_right)

    def getYRight(self) -> float:
        return self.getY(self.light_right)

    def getYLeft(self) -> float:
        return self.getY(self.light_left)

    def getDistanceRight(self) -> float:
        id = self.id(self.light_right)
        return self.distance(
            self.light_right,
            LimeLightConstants.limelight_right_height,
            LimeLightConstants.april_tag_height[id],
            LimeLightConstants.limelight_right_angle,
        )

    def getDistanceLeft(self) -> float:
        id = self.id(self.light_left)
        return self.distance(
            self.light_left,
            LimeLightConstants.limelight_left_height,
            LimeLightConstants.april_tag_height[id],
            LimeLightConstants.limelight_left_angle,
        )
