import swerve
from wpimath.geometry import Rotation2d, Pose2d
from swerve import *
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveDrive4Odometry

class Robot(wpilib.TimedRobot):

    def robotInit(self):
        print("Robot Initialized")
        self.controller1 = wpilib.XboxController(0)
        self.controller2 = wpilib.XboxController(1)
        self.arm = ctre.WPI_VictorSPX(11)
        self.kin = SwerveDrive4Kinematics()
        self.swerve = SwerveDrive4Odometry(self.kin, Rotation2d(0, 0), Pose2d())

    def autonomousInit(self):
        print("Started Autonomous")

    def autonomousPeriodic(self):
        pass

    def autonomousExit(self):
        print("Exited Autonomous")

    def teleopInit(self):
        print("Started Teleop")
        self.arm.set(self.controller1.getRightY() * 0.3)


    def teleopPeriodic(self):
        swerve.drive(xSpeed, ySpeed, rot, fieldRelative)

    def teleopExit(self):
        print("Exited Teleop")


if __name__ == "__main__":
    wpilib.run(Robot)
