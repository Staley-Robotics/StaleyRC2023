import math
import wpimath
import drivetrain
from swerve import *
import wpilib


class Robot(wpilib.TimedRobot):

    def robotInit(self):
        print("Robot Initialized")
        self.controller1 = wpilib.XboxController(0)
        self.controller2 = wpilib.XboxController(1)
        #self.arm = ctre.WPI_VictorSPX(11)
        #self.kin = SwerveDrive4Kinematics()
        #self.swerve = SwerveDrive4Odometry(self.kin, Rotation2d(0, 0), Pose2d())
        self.swerve = drivetrain.Drivetrain()

    def autonomousInit(self):
        print("Started Autonomous")

    def autonomousPeriodic(self):
        pass

    def autonomousExit(self):
        print("Exited Autonomous")

    def teleopInit(self):
        print("Started Teleop")
       # self.arm.set(self.controller1.getRightY() * 0.3)


    def teleopPeriodic(self):
        def clamp(num, min_value):
            if abs(num) < min_value:
                return 0
            return num

        leftx1 = wpimath.applyDeadband(self.controller1.getLeftX(), 0.5, 1)
        lefty1 = wpimath.applyDeadband(self.controller1.getLeftY(), 0.5, 1)
        rightx1 = wpimath.applyDeadband(self.controller1.getRightX(), 0.5, 1)

        self.swerve.drive(leftx1, lefty1, rightx1, True)

    def teleopExit(self):
        print("Exited Teleop")


if __name__ == "__main__":
    wpilib.run(Robot)
