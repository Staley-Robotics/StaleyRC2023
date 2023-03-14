class Trajectory:
    pass


"""
NOT SURE IF RAMSETE IS POSSIBLE
MAY NEED TO USE HOLONOMIC CONTROLLER

Hardware requirements:
Encoders  (CANcoders
Gyroscope (Pigeon)

Software requirements:
Characterization
Kinematics + Odometry
Trajectory Generation
Ramsete Control

Odometry requirements:
Wheel velocity (all swerve modules)
heading (gathered from pigeon)


(Odometry tells the controller where robot is, trajectory says what we want it to be)

Odometry sends robotPose
(x, y, heading) to Ramsete controller

Trajectory sends to Ramsete controller:
referencePose and velocities
along with heading

Ramsete controller calculates Linear and angular velocity (rate of change of the angle)
(m/s and radians/s)
and sends data to the kinematics

PID controllers take left and right velocity from kinematics
(Will have some accumulated error, can fix through Encoder + Gyro readings)

Robot characterization takes left and right velocities from kinematics

PID controllers will send feedback voltages
Robot Characterization will send feedforward voltages



MORE IN DEPTH HARDWARE/SOFTWARE:
encoders:
    measure distance traveled on each side of a differential drive robot
    measure angle of swerve module and the distance traveled by each module

gyro:
    Measures angular rate of robot
    Measures absolute angle of the robot
    
odometry:
    Uses encoder/gyro data
    calculates robotPose (position) on the field
    robotPose for tracking trajectories
    
    Instead of continous, use a series of vectors that can be used to calc the delta x and y, which can be added
    together to find robots position on the field
    
Kinematics:
    Converts between ChassisSpeeds and DifferentialDriveWheelSpeeds
    ChassisSpeeds contains linear and angular velocities
    DifferentialDriveWheelSpeeds contains left and right velocities (might have to use this for all 4 wheels
    due to 0 point turn
    


"""
