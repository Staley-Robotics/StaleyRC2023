import wpimath
from wpilib import *
from wpimath.geometry import *
from wpimath.kinematics import *
from ctre.sensors import WPI_Pigeon2
from drivetrain.chassis import Chassis
from drivetrain.swerve_module import *
from tools import PipelineManager


# Represents a swerve drive style drivetrain.
class Swerve2(Chassis):
    m_front_left_location = Translation2d(0.254, 0.305)
    m_front_right_location = Translation2d(0.254, -0.305)
    m_back_left_location = Translation2d(-0.254, 0.305)
    m_back_right_location = Translation2d(-0.254, -0.305)

    m_front_left = SwerveModule(7, 8, 18, 47.373) #-136.494)#, 1, 2, 3, 0)
    m_front_right = SwerveModule(5, 6, 16, -134.492+180) #34.277)#, 5, 6, 7, 341)
    m_back_left = SwerveModule(3, 4, 14, 25.996) # -161.191)#, 9, 10, 11, 1706)
    m_back_right = SwerveModule(1, 2, 12, -113.115+180) #66.885)#, 13, 14, 15, 1365)

    m_gyro = WPI_Pigeon2(9, "rio")

    m_kinematics = SwerveDrive4Kinematics(m_front_left_location, m_front_right_location, m_back_left_location,
                                          m_back_right_location)

    def __init__(self, pipeline: PipelineManager):
        super().__init__(pipeline)
        # self.m_odometry = SwerveDrive4Odometry(
        #     self.m_kinematics,
        #     self.m_gyro.getRotation2d(),
        #     (
        #         self.m_front_left.getPosition(),
        #         self.m_front_right.getPosition(),
        #         self.m_back_left.getPosition(),
        #         self.m_back_right.getPosition()
        #     ),
        #     Pose2d(Translation2d(0.0, 0.0), Rotation2d(0.0))
        # )
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
        controller = XboxController(0)
        velocity = controller.getLeftY()
        rotation = controller.getLeftX()
        direction = controller.getRightX()
        if field_relative:
            drive_movement = ChassisSpeeds.fromFieldRelativeSpeeds(wpimath.applyDeadband(rotation, 0.05, 1.0), wpimath.applyDeadband(velocity, 0.05, 1.0),
                                                                   wpimath.applyDeadband(direction, 0.05, 1.0) * 3.14,
                                                                   self.m_gyro.getRotation2d())
        else:
            drive_movement = ChassisSpeeds(wpimath.applyDeadband(self.pipeline.rotation(), 0.05, 1.0), wpimath.applyDeadband(self.pipeline.throttle(), 0.05, 1.0), wpimath.applyDeadband(self.pipeline.direction_x(), 0.05, 1.0) * 3.14)

        swerve_module_states = self.m_kinematics.toSwerveModuleStates(drive_movement, Translation2d(0, 0))
        swerve_module_states = SwerveDrive4Kinematics.desaturateWheelSpeeds(swerve_module_states, kMaxSpeed)
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
