from wpilib import *

from drivetrain.chassis import Chassis
from drivetrain.swerve_module import *
from tools import PipelineManager


class Swerve2(Chassis):

    front_left = SwerveModule(7, 8, 0, 1, 2, 3, Translation2d(0.381, 0.381))
    front_right = SwerveModule(5, 6, 4, 5, 6, 7, Translation2d(0.381, -0.381))
    back_left = SwerveModule(3, 4, 8, 9, 10, 11, Translation2d(-0.381, 0.381))
    back_right = SwerveModule(1, 2, 12, 13, 14, 15, Translation2d(-0.381, -0.381))

    gyro = AnalogGyro(0)

    kinematics: SwerveDrive4Kinematics = SwerveDrive4Kinematics(front_left.location, front_right.location,
                                                                back_left.location, back_right.location)

    odometry: SwerveDrive4Odometry = SwerveDrive4Odometry(
        kinematics,
        gyro.getRotation2d(),
        (
            front_left.getPosition(),
            front_right.getPosition(),
            back_left.getPosition(),
            back_right.getPosition()
        )
    )

    def __init__(self, pipeline: PipelineManager):
        super().__init__(pipeline)
        self.gyro.reset()

    def drive(self):
        drive_movement: ChassisSpeeds
        field_relative = True
        if field_relative:
            drive_movement = ChassisSpeeds.fromFieldRelativeSpeeds(self.pipeline.rotation(), self.pipeline.drive(),
                                                                   self.pipeline.direction_x(),
                                                                   self.gyro.getRotation2d())
        else:
            drive_movement = ChassisSpeeds(self.pipeline.rotation(), self.pipeline.drive(), self.pipeline.direction_x())

        swerve_module_states = self.kinematics.toSwerveModuleStates(drive_movement, Translation2d(0, 0))
        swerve_module_states = SwerveDrive4Kinematics.desaturateWheelSpeeds(swerve_module_states, max_speed)
        self.front_left.setDesiredState(swerve_module_states[0])
        self.front_right.setDesiredState(swerve_module_states[1])
        self.back_left.setDesiredState(swerve_module_states[2])
        self.back_right.setDesiredState(swerve_module_states[3])

    def updatePosition(self):
        self.odometry.update(
            self.gyro.getRotation2d(),
            self.front_left.getPosition(),
            self.front_right.getPosition(),
            self.back_left.getPosition(),
            self.back_right.getPosition()
        )
