from wpilib import *
from wpimath.geometry import *
from wpimath.kinematics import *

from drivetrain.chassis import Chassis
from drivetrain.swerve_module import *
from tools import PipelineManager


# Represents a swerve drive style drivetrain.
class Swerve2(Chassis):
    m_front_left_location = Translation2d(0.381, 0.381)
    m_front_right_location = Translation2d(0.381, -0.381)
    m_back_left_location = Translation2d(-0.381, 0.381)
    m_back_right_location = Translation2d(-0.381, -0.381)

    m_front_left = SwerveModule(7, 8, 0, 1, 2, 3)
    m_front_right = SwerveModule(5, 6, 4, 5, 6, 7)
    m_back_left = SwerveModule(3, 4, 8, 9, 10, 11)
    m_back_right = SwerveModule(1, 2, 12, 13, 14, 15)

    m_gyro = AnalogGyro(0)

    m_kinematics = SwerveDrive4Kinematics(m_front_left_location, m_front_right_location, m_back_left_location,
                                          m_back_right_location)

    m_odometry = SwerveDrive4Odometry(
        m_kinematics,
        m_gyro.getRotation2d(),
        (
            m_front_left.getPosition(),
            m_front_right.getPosition(),
            m_back_left.getPosition(),
            m_back_right.getPosition()
        )
    )

    def __init__(self, pipeline: PipelineManager):
        super().__init__(pipeline)
        self.m_gyro.reset()

    """
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   """

    def drive(self):
        drive_movement = None
        field_relative = True
        if field_relative:
            drive_movement = ChassisSpeeds.fromFieldRelativeSpeeds(self.pipeline.rotation, self.pipeline.throttle,
                                                                   self.pipeline.direction_x,
                                                                   self.m_gyro.getRotation2d())
        else:
            drive_movement = ChassisSpeeds(self.pipeline.rotation, self.pipeline.throttle, self.pipeline.direction_x)

        swerve_module_states = self.m_kinematics.toSwerveModuleStates(drive_movement, Translation2d(0, 0))
        SwerveDrive4Kinematics.desaturateWheelSpeeds(swerve_module_states, kMaxSpeed)
        self.m_front_left.setDesiredState(swerve_module_states[0])
        self.m_front_right.setDesiredState(swerve_module_states[1])
        self.m_back_left.setDesiredState(swerve_module_states[2])
        self.m_back_right.setDesiredState(swerve_module_states[3])

    """ Updates the field relative position of the robot. """

    def updateOdometry(self):
        self.m_odometry.update(
            self.m_gyro.getRotation2d(),
            self.m_front_left.getPosition(),
            self.m_front_right.getPosition(),
            self.m_back_left.getPosition(),
            self.m_back_right.getPosition()
        )
