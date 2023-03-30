from wpimath.geometry import *
from wpimath.kinematics import *
from pyfrc.physics.core import PhysicsInterface

from ctre import *
from ctre.sensors import *

from subsystems.swervedrive import SwerveDrive4

class PhysicsEngine:
    def __init__(self, physics_controller, robot):
        self.physics_controller:PhysicsInterface = physics_controller

        for i in range(len(robot.subsystems)):
            if type(robot.subsystems[i]) == SwerveDrive4:
                self.drivetrain:SwerveDrive4 = robot.subsystems[0]
                self.gyroSim:BasePigeonSimCollection = self.drivetrain.gyro.getSimCollection()

                self.moduleFR_AM:TalonFXSimCollection = self.drivetrain.moduleFR.angleMotor.getSimCollection()
                self.moduleFR_AM.setIntegratedSensorRawPosition(1991)
                self.moduleFR_AS:CANCoderSimCollection = self.drivetrain.moduleFR.angleSensor.getSimCollection()
                self.moduleFR_AS.setRawPosition(1991)

    def update_sim(self, now, tm_diff):
        if self.drivetrain is None:
            return
        elif self.drivetrain.cSpeeds is None:
            return
        
        # Physics Controller Drivetrain Simulation
        cSpeeds:ChassisSpeeds = self.drivetrain.cSpeeds
        newPose:Pose2d = self.physics_controller.drive( cSpeeds, tm_diff )

        # Gyro Simulation
        newRot:Rotation2d = newPose.rotation()
        self.gyroSim.setRawHeading( newRot.degrees() )

        #self.moduleFR_AM.addIntegratedSensorPosition( self.drivetrain.moduleFR.adjTicks )
        #self.moduleFR_AS.addPosition( self.drivetrain.moduleFR.adjTicks )