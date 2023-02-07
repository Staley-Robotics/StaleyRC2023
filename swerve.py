import math
from ctre import WPI_TalonFX
import wpimath.geometry
import wpimath.kinematics
from robot import Robot
from wpilib import drive
import wpilib
import ctre



#Setting up the TalonFx objects for each of the four drive motors
frontLeftDrive = WPI_TalonFX(1)
frontRightDrive = WPI_TalonFX(2)
rearLeftDrive = WPI_TalonFX(3)
rearRightDrive = WPI_TalonFX(4)

#List to store the drive motors
driveMotors = [frontLeftDrive, frontRightDrive, rearLeftDrive, rearRightDrive]

#Setting up the TalonFX objects for each of the four
frontLeftSteer = WPI_TalonFX(5)
frontRightSteer = WPI_TalonFX(6)
rearLeftSteer = WPI_TalonFX(7)
rearRightSteer = WPI_TalonFX(8)


m_frontLeftLocation = wpimath.geometry.Translation2d(0.295, 0.295)
m_frontRightLocation = wpimath.geometry.Translation2d(0.295, -0.295)
m_backLeftLocation = wpimath.geometry.Translation2d(-0.295, 0.295)
m_backRightLocation = wpimath.geometry.Translation2d(-0.295, -0.295)

m_kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
    m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation)

# Example chassis speeds: 1 meter per second forward, 3 meters
# per second to the left, and rotation at 1.5 radians per second
# counterclockwise.
speeds = wpimath.kinematics.ChassisSpeeds(1.0, 3.0, 1.5)

# Convert to module states
moduleStates = m_kinematics.toSwerveModuleStates(speeds)

# Front left module state
frontLeft = moduleStates[0]

# Front right module state
frontRight = moduleStates[1]

# Back left module state
backLeft = moduleStates[2]

# Back right module state
backRight = moduleStates[3]

frontLeftOptimized = moduleStates[0].optimize(frontLeft, wpimath.geometry.Rotation2d(0))
frontRightOptimized = moduleStates[1].optimize(frontLeft, wpimath.geometry.Rotation2d(0))
BackLeftOptimized = moduleStates[2].optimize(frontLeft, wpimath.geometry.Rotation2d(0))
backRightOptimized = moduleStates[3].optimize(frontLeft, wpimath.geometry.Rotation2d(0))


# The desired field relative speed here is 2 meters per second
# toward the opponent's alliance station wall, and 2 meters per
# second toward the left field boundary. The desired rotation
# is a quarter of a rotation per second counterclockwise. The current
# robot angle is 45 degrees.
speeds = wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
  2.0, 2.0, math.pi / 2.0, wpimath.geometry.Rotation2d.fromDegrees(45.0))

# Now use this in our kinematics
moduleStates = m_kinematics.toSwerveModuleStates(speeds)

"""
#Creating a list to store the steer motors
steerMotors = [frontLeftSteer, frontRightSteer, rearLeftSteer, rearRightSteer]

#Method to calculate the speed and angle of the each wheel
def calculationForSwerveDrive():
    #Get the joystick inputs for the x and y axes
    x = controller1.getRightX()
    y = controller1.getRightY()

    #Calculate the speed and angle of each wheel

    #Cameron, Austin, and Hudson need to build off of the speed and positioning code to get this
    #section of code.


    for i in range(4):
        driveMotors[i].set(speed[i])
        driveMotors[i].set(angle[i])
        
"""