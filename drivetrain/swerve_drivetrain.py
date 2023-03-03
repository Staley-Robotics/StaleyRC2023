import math

from wpilib import *
from wpimath.geometry import *
from wpimath.kinematics import *

from drivetrain.swerve_module import *


kMaxAngularSpeed = math.pi  # 1/2 rotation per second


# Represents a swerve drive style drivetrain.
class Drivetrain:
    m_frontLeftLocation = Translation2d(0.381, 0.381)
    m_frontRightLocation = Translation2d(0.381, -0.381)
    m_backLeftLocation = Translation2d(-0.381, 0.381)
    m_backRightLocation = Translation2d(-0.381, -0.381)

    m_frontLeft = SwerveModule(7, 8, 0, 1, 2, 3)
    m_frontRight = SwerveModule(5, 6, 4, 5, 6, 7)
    m_backLeft = SwerveModule(3, 4, 8, 9, 10, 11)
    m_backRight = SwerveModule(1, 2, 12, 13, 14, 15)

    m_gyro = AnalogGyro(0)

    m_kinematics = SwerveDrive4Kinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,m_backRightLocation)

    #m_odometry = SwerveDrive4Odometry(
    #    m_kinematics,
    #    m_gyro.getRotation2d(),
    #    (
    #        m_frontLeft.getPosition(),
    #        m_frontRight.getPosition(),
    #        m_backLeft.getPosition(),
    #        m_backRight.getPosition()
    #    )
    #)

    def __init__(self):
        self.m_gyro.reset()

    """
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   """

    def drive(self, xSpeed, ySpeed, rot, fieldRelative):
        driveMovement = None
        if fieldRelative:
            driveMovement = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, self.m_gyro.getRotation2d())
        else:
            driveMovement = ChassisSpeeds(xSpeed, ySpeed, rot)

        swerveModuleStates = self.m_kinematics.toSwerveModuleStates(driveMovement, Translation2d(0, 0))
        SwerveDrive4Kinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed)
        self.m_frontLeft.setDesiredState(swerveModuleStates[0])
        self.m_frontRight.setDesiredState(swerveModuleStates[1])
        self.m_backLeft.setDesiredState(swerveModuleStates[2])
        self.m_backRight.setDesiredState(swerveModuleStates[3])

    """ Updates the field relative position of the robot. """

    def updateOdometry(self):
        #self.m_odometry.update(
        #   self.m_gyro.getRotation2d(),
        #       self.m_frontLeft.getPosition(),
        #       self.m_frontRight.getPosition(),
        #       self.m_backLeft.getPosition(),
        #       self.m_backRight.getPosition()
        #   )
        pass