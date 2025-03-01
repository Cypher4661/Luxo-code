from wpimath.geometry import Rotation2d


class SwerveModuleConstants:
    def __init__(
        self,
        drive_motor_id: int,
        angle_motor_id: int,
        can_coder_id: int,
        angle_offset: Rotation2d,
    ):
        self.driveMotorID = drive_motor_id
        self.angleMotorID = angle_motor_id
        self.canCoderID = can_coder_id
        self.angleOffset = angle_offset
