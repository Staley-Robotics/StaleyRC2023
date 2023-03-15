import sys

from . import Subsystems
from .__func__ import *
from ctre import WPI_TalonFX, ControlMode, FeedbackDevice, RemoteFeedbackDevice, NeutralMode
from ctre.sensors import *  #WPI_Pigeon2, WPI_CANCoder, SensorInitializationStrategy, AbsoluteSensorRange
from wpilib import SmartDashboard, XboxController
from wpimath import applyDeadband
from wpimath.controller import PIDController, ProfiledPIDController, ProfiledPIDControllerRadians, HolonomicDriveController, SimpleMotorFeedforwardMeters
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveDrive4Odometry, SwerveModulePosition, SwerveModuleState, ChassisSpeeds
from wpimath.trajectory import TrapezoidProfile, TrapezoidProfileRadians

class SwerveDrive4(Subsystems):
    cSpeeds:ChassisSpeeds = ChassisSpeeds()
    op1 = XboxController(0)
    halfSpeed = False

    def initSubsystem(self):
        # Gyro
        gyroId:int = int(self.ntCfgs.getNumber("gyroId", 61))
        gyroCanbus:str = self.ntCfgs.getString("gyroCanbus", "rio")
        self.gyro = WPI_Pigeon2(gyroId, gyroCanbus)
        self.gyro.setYaw(0)
        
        # SwerveModules
        self.moduleFL = SwerveModule(subComponent="FrontLeft")
        self.moduleFR = SwerveModule(subComponent="FrontRight")
        self.moduleBL = SwerveModule(subComponent="BackLeft")
        self.moduleBR = SwerveModule(subComponent="BackRight")

        # SwerveKinematics
        self.kinematics = SwerveDrive4Kinematics(
            self.moduleFL.getModulePosition(),
            self.moduleFR.getModulePosition(),
            self.moduleBL.getModulePosition(),
            self.moduleBR.getModulePosition()
        )

        # SwerveOdometry
        self.odometry = SwerveDrive4Odometry(
            self.kinematics,
            self.gyro.getRotation2d(),
            (
                self.moduleFL.getPosition(),
                self.moduleFR.getPosition(),
                self.moduleBL.getPosition(),
                self.moduleBR.getPosition()
            ),
            Pose2d(Translation2d(0, 0), Rotation2d(0))
        )

        self.slx = SlewRateLimiter(2)
        self.sly = SlewRateLimiter(2)

    def initVariables(self):
        # Field Relative
        self.fieldRelative = self.ntVars.getBoolean("fieldRelative", True)

        # ChassisSpeed Velocities
        self.speed_linear_maxvelocity = self.ntVars.getNumber("speed/linear/maxVelocity", 1.0)
        self.speed_linear_maxvelocityx = self.ntVars.getNumber("speed/linear/maxVelocityX", self.speed_linear_maxvelocity)
        self.speed_linear_maxvelocityy = self.ntVars.getNumber("speed/linear/maxVelocityY", self.speed_linear_maxvelocity)
        self.speed_angular_maxvelocity = self.ntVars.getNumber("speed/angular/maxVelocity", (8 * math.pi))
        self.speed_angular_maxacceleration = self.ntVars.getNumber("speed/angular/maxAcceleration", (4 * math.pi))

        # Chassis X Direction PID
        #self.pidX_kp = self.ntVars.getNumber("ChassisControl/X/kP",1.0)
        #self.pidX_ki = self.ntVars.getNumber("ChassisControl/X/kI",0.0)
        #self.pidX_kd = self.ntVars.getNumber("ChassisControl/X/kD",0.0)
        #self.pidX_ks = self.ntVars.getNumber("ChassisControl/X/kS",0.0)
        #self.pixX_kv = self.ntVars.getNumber("ChassisControl/X/kV",0.0)
        #self.pidX_ka = self.ntVars.getNumber("ChassisControl/X/kA",0.0)
        #self.pidX_tolerance = self.ntVars.getNumber("ChassisControl/X/tolerance",1.0)         

        #self.pidY_kp = self.ntVars.getNumber("ChassisControl/Y/kP",1.0)
        #self.pidY_ki = self.ntVars.getNumber("ChassisControl/Y/kI",0.0)
        #self.pidY_kd = self.ntVars.getNumber("ChassisControl/Y/kD",0.0)
        #self.pidY_ks = self.ntVars.getNumber("ChassisControl/Y/kS",0.0)
        #self.pidY_kv = self.ntVars.getNumber("ChassisControl/Y/kV",0.0)
        #self.pidY_ka = self.ntVars.getNumber("ChassisControl/Y/kA",0.0)
        #self.pidY_tolerance = self.ntVars.getNumber("ChassisControl/Y/tolerance",1.0)         

        # Chassis Omega PID
        self.chassiscontrol_omega_kp = self.ntVars.getNumber("chassisControl/omega/kP",1.0)
        self.chassiscontrol_omega_ki = self.ntVars.getNumber("chassisControl/omega/kI",0.0)
        self.chassiscontrol_omega_kd = self.ntVars.getNumber("chassisControl/omega/kD",0.0)
        self.chassiscontrol_omega_ks = self.ntVars.getNumber("chassisControl/omega/kS",0.0)
        self.chassiscontrol_omega_kv = self.ntVars.getNumber("chassisControl/omega/kV",0.0)
        self.chassiscontrol_omega_ka = self.ntVars.getNumber("chassisControl/omega/kA",0.0)
        self.chassiscontrol_omega_tolerance = self.ntVars.getNumber("chassisControl/omega/tolerance",0.05)

        self.initPidControllers()
    
    def updateVariableSubsystems(self):
        self.updatePidControllers()

    def initPidControllers(self):
        # Create X Direction PID Controller
        #self.pidX = PIDController()

        # Create Y Direction PID Controller
        #self.pidY = PIDController()
        
        # Create Omega PID
        self.pidO = ProfiledPIDControllerRadians(
            self.chassiscontrol_omega_kp,
            self.chassiscontrol_omega_ki, 
            self.chassiscontrol_omega_kd,
            TrapezoidProfileRadians.Constraints(
                self.chassiscontrol_omega_kv,
                self.chassiscontrol_omega_ka
            )
        )
        self.pidO.enableContinuousInput(-math.pi, math.pi)
        self.pidO.setTolerance(self.chassiscontrol_omega_tolerance)
        
        # Create HolonomicDriveController PID Controller
        #self.pidH = HolonomicDriveController(self.pidX, self.pidY, self.pidO)

    def updatePidControllers(self):
        # Update X Direction PID Controller
        #pidX = self.pidX #self.pidH.getXController()

        # Update Y Direction PID Controller 
        #pidY = self.pidY #self.pidH.getYController()

        # Update Omega PID Controller
        pidO = self.pidO #self.pidH.getThetaController()
        pidO.setP( self.chassiscontrol_omega_kp )
        pidO.setI( self.chassiscontrol_omega_ki )
        pidO.setD( self.chassiscontrol_omega_kd )
        pidO.setConstraints(
            TrapezoidProfileRadians.Constraints(
                self.chassiscontrol_omega_kv,
                self.chassiscontrol_omega_ka
            )
        )
        pidO.setTolerance(self.chassiscontrol_omega_tolerance)

        # Update Holonomic PID Controller
        #self.pidH.setTolerance()

    def run(self):
        # Process Controller Inputs
        lx, ly, lt, rx, ry, rt = self.getInputs()
        
        # Drive
        if rx != 0.0 or ry != 0.0:
            self.driveWithRotate( lx, ly, rx, ry )
        else:
            omega = (rt - lt)
            self.drive( lx, ly, omega )

    def getInputs(self) -> tuple[float,float,float,float,float,float]:
        lx = applyDeadband(self.op1.getLeftY(), 0.1, 1.0)
        slx = self.slx.calculate(lx)
        ly = applyDeadband(self.op1.getLeftX(), 0.1, 1.0)
        sly = self.sly.calculate(ly)
        lt = applyDeadband(self.op1.getLeftTriggerAxis(), 0.1, 1.0)
        rx = applyDeadband(self.op1.getRightY(), 0.1, 1.0)
        ry = applyDeadband(self.op1.getRightX(), 0.1, 1.0)
        rt = applyDeadband(self.op1.getRightTriggerAxis(), 0.1, 1.0)

        if self.op1.getAButtonPressed():
            self.halfSpeed = not self.halfSpeed

        if self.halfSpeed:
            slx *= 0.5
            sly *= 0.5

        return slx, sly, lt, rx, ry, rt

    def driveToPose(self, pose:Pose2d, velocity:float=None):
    #    if velocity is None:
    #        velocity = self.linear_maxvelocity
    #
    #    currentPose = self.odometry.getPose()
    #    x = self.pidX.calculate( currentPose.X(), pose.X() )
    #    y = self.pidY.calculate( currentPose.Y(), pose.Y() )
    #    rot = self.driveToRotation( x, y, 1, rotation=pose.rotation() )
    #    print( currentPose.X(), pose.X() )
    #    print( currentPose.Y(), pose.Y() )
    #    print( x, y, pose.rotation() )
    #    return self.pidX.atSetpoint() and self.pidY.atSetpoint() and rot
        pass

    def driveWithRotate( self, lx=0.0, ly=0.0, rx=0.0, ry=0.0 ):
        magnitude = math.sqrt( rx * rx + ry * ry )
        rotation = Rotation2d( rx, ry )
        dtr = self.driveToRotation( lx, ly, magnitude, rotation )
        return dtr

    def driveToRotation(self, x, y, magnitude=0.0, rotation:Rotation2d=Rotation2d(0,0)):
        robotAngle = self.gyro.getRotation2d()
        
        currentRad = robotAngle.radians()
        newRad = rotation.radians()
        omegaTarget = self.pidO.calculate( currentRad, newRad )

        omega = max(abs(magnitude),1) * omegaTarget
        if omega > 1.0: omega = 1.0
        elif omega < -1.0: omega = -1.0
        omega = applyDeadband( omega, self.chassiscontrol_omega_tolerance )
        self.drive( x, y, omega )
        if omega == 0:
            return True
        else:
            return False

    def drive(self, x=0.0, y=0.0, omega=0.0):
        if self.fieldRelative:
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                vx = x * self.speed_linear_maxvelocityx,
                vy = y * self.speed_linear_maxvelocityy,
                omega = omega * self.speed_angular_maxvelocity,
                robotAngle = self.gyro.getRotation2d()
            )
        else:
            speeds = ChassisSpeeds(
                vx = x * self.speed_linear_maxvelocityx,
                vy = y * self.speed_linear_maxvelocityy,
                omega = omega * self.speed_angular_maxvelocity,
            )
        self.cSpeeds = speeds  ## FOR SIMULATION
        self.updateSwerveModules(speeds)

    def updateSwerveModules(self, speeds:ChassisSpeeds):
        rotationCenter = Translation2d(0,0)
        modStates = self.kinematics.toSwerveModuleStates(speeds, rotationCenter)
        modStates = SwerveDrive4Kinematics.desaturateWheelSpeeds( modStates, self.speed_linear_maxvelocity )
    
        self.moduleFL.setDesiredState(modStates[0])
        self.moduleFR.setDesiredState(modStates[1])
        self.moduleBL.setDesiredState(modStates[2])
        self.moduleBR.setDesiredState(modStates[3])
        
        self.updateOdometry()

    def updateOdometry(self):
        old = self.odometry.getPose()
        pose = self.odometry.update(
            self.gyro.getRotation2d(),
            self.moduleFL.getPosition(),
            self.moduleFR.getPosition(),
            self.moduleBL.getPosition(),
            self.moduleBR.getPosition()
        )
        SmartDashboard.putNumberArray("Field/Swerve",[pose.X(), pose.Y(), pose.rotation().degrees()])
        #if old.x != pose.x or old.y != pose.y or old.rotation().degrees() != pose.rotation().degrees():
        #    pass

        

class SwerveModule(Subsystems):
    def initSubsystem(self):
        driveMotorId:int = int(self.ntCfgs.getNumber("driveMotorId",1))
        driveMotorCanbus:str = self.ntCfgs.getString("driveMotorCanbus","rio")
        angleMotorId:int = int(self.ntCfgs.getNumber("angleMotorId",2))
        angleMotorCanbus:str = self.ntCfgs.getString("angleMotorCanbus","rio")
        angleSensorId:int = int(self.ntCfgs.getNumber("angleSensorId",3))
        angleSensorCanbus:str = self.ntCfgs.getString("angleSensorCanbus","rio")

        modulePositionX:float = self.ntCfgs.getNumber("modulePositionX",0.25)
        modulePositionY:float = self.ntCfgs.getNumber("modulePositionY",0.25)
        self.modulePosition = Translation2d(modulePositionX, modulePositionY)
        
        driveMotorReversed:bool = self.ntCfgs.getBoolean("driveMotorReversed",False)
        driveMotorPhase:bool = self.ntCfgs.getBoolean("driveMotorPhase",False)

        angleMotorReversed:bool = self.ntCfgs.getBoolean("angleMotorReversed",True)
        angleMotorPhase:bool = self.ntCfgs.getBoolean("angleMotorPhase",True)

        angleSensorReversed:bool = self.ntCfgs.getBoolean("angleSensorReversed",False)
        angleSensorOffset:float = self.ntCfgs.getNumber("angleSensorOffset",0.0)

        # Angle Sensor
        self.angleSensor:WPI_CANCoder = WPI_CANCoder(angleSensorId, angleSensorCanbus)
        self.angleSensor.configFactoryDefault()
        self.angleSensor.configSensorInitializationStrategy(SensorInitializationStrategy.BootToZero)
        self.angleSensor.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360 )
        self.angleSensor.configSensorDirection(angleSensorReversed)
        self.angleSensor.setPosition( self.angleSensor.getAbsolutePosition() - angleSensorOffset )

        # Angle Motor
        self.angleMotor:WPI_TalonFX = WPI_TalonFX(angleMotorId, angleMotorCanbus)
        self.angleMotor.configFactoryDefault()
        self.angleMotor.configRemoteFeedbackFilter(self.angleSensor, 0)
        self.angleMotor.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0, 0)
        self.angleMotor.configSelectedFeedbackSensor(FeedbackDevice.None_,1)
        self.angleMotor.setSensorPhase(angleMotorPhase)
        self.angleMotor.setInverted(angleMotorReversed)
        self.angleMotor.setNeutralMode(NeutralMode.Coast)
        self.angleMotor.configFeedbackNotContinuous(True)

        # Drive Motor
        self.driveMotor:WPI_TalonFX = WPI_TalonFX(driveMotorId, driveMotorCanbus)
        self.driveMotor.configFactoryDefault()
        self.driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor)
        self.driveMotor.setInverted(driveMotorReversed)
        self.driveMotor.setSensorPhase(driveMotorPhase)
        self.driveMotor.setNeutralMode(NeutralMode.Coast)

    def initVariables(self):
        # Integrated PID
        self.drivemotors_integratedpid:bool = self.ntVars.getBoolean("driveMotors/integratedPid",True)
        self.anglemotors_integratedpid:bool = self.ntVars.getBoolean("angleMotors/integratedPid",True)

        # Integrated PID
        self.drivemotors_slewratesteps:int = self.ntVars.getNumber("driveMotors/slewRateSteps", 200000)

        # Physical Mechanics
        self.wheelRadius:float = self.ntVars.getNumber("wheelRadius",0.05)
        self.drivemotors_ticks:float = self.ntVars.getNumber("driveMotors/ticks",2048.0)
        self.anglemotors_ticks:float = self.ntVars.getNumber("angleMotors/ticks",2048.0)
        self.anglesensors_ticks:float = self.ntVars.getNumber("angleSensors/ticks",4096.0)

        self.drivemotors_gearratio: float = self.ntVars.getNumber("driveMotors/gearRatio", 1.0)

        # Drive PID Mechanics
        self.drivemotors_pidff_kp:float = self.ntVars.getNumber("driveMotors/pidFf/kP",0.15)
        self.drivemotors_pidff_ki:float = self.ntVars.getNumber("driveMotors/pidFf/kI",0.0)
        self.drivemotors_pidff_kd:float = self.ntVars.getNumber("driveMotors/pidFf/kD",0.0)
        self.drivemotors_pidff_kf:float = self.ntVars.getNumber("driveMotors/pidFf/kF",0.065)  #recommend start here
        self.drivemotors_pidff_ks:float = self.ntVars.getNumber("driveMotors/pidFf/kS",0.0)
        self.drivemotors_pidff_kv:float = self.ntVars.getNumber("driveMotors/pidFf/kV",0.0)
        self.drivemotors_pidff_ka:float = self.ntVars.getNumber("driveMotors/pidFf/kA",0.0)
        self.drivemotors_pidff_tolerance:float = self.ntVars.getNumber("driveMotors/pidFf/tolerance",0.0)

        # Rotate PID Mechanics
        self.anglemotors_pidff_kp:float = self.ntVars.getNumber("angleMotors/pidFf/kP",0.5)   #recommended: (throttle*1023/4096) ~ 0.125
        self.anglemotors_pidff_ki:float = self.ntVars.getNumber("angleMotors/pidFf/kI",0.0)   #recommended: 1% of kP
        self.anglemotors_pidff_kd:float = self.ntVars.getNumber("angleMotors/pidFf/kD",0.0)   #recommended: 10-100x of kP
        self.anglemotors_pidff_kf:float = self.ntVars.getNumber("angleMotors/pidFf/kF",0.0)
        self.anglemotors_pidff_ks:float = self.ntVars.getNumber("angleMotors/pidFf/kS",0.0)
        self.anglemotors_pidff_kv:float = self.ntVars.getNumber("angleMotors/pidFf/kV",0.0)
        self.anglemotors_pidff_ka:float = self.ntVars.getNumber("angleMotors/pidFf/kA",0.0)
        self.anglemotors_pidff_tolerance:float = self.ntVars.getNumber("angleMotors/pidFf/tolerance",0.0)

        self.initPidControllers()
        self.initSlewRateLimiter()

    def updateVariableSubsystems(self):
        self.updatePidControllers()
        self.initSlewRateLimiter()  ### Should we consider only updating this during reboot?

    def initSlewRateLimiter(self):
        self.driveSRL = SlewRateLimiter( self.drivemotors_slewratesteps )

    def initPidControllers(self):
        self.pidDrive = PIDController(
            self.drivemotors_pidff_kp,
            self.drivemotors_pidff_ki,
            self.drivemotors_pidff_kd
        )
        self.ffDrive = SimpleMotorFeedforwardMeters(
            self.drivemotors_pidff_kf,
            self.drivemotors_pidff_kv,
            self.drivemotors_pidff_ks
        )
        self.pidAngle = ProfiledPIDControllerRadians(
            self.anglemotors_pidff_kp,
            self.anglemotors_pidff_ki,
            self.anglemotors_pidff_kd,
            TrapezoidProfileRadians.Constraints(
                self.anglemotors_pidff_kv,
                self.anglemotors_pidff_ka
            )
        )
        self.pidAngle.enableContinuousInput(-math.pi,math.pi)
        self.updatePidControllers()

    def updatePidControllers(self):
        self.driveMotor.config_kF(1, self.drivemotors_pidff_kf) # 0.065
        self.driveMotor.config_kP(1, self.drivemotors_pidff_kp) # 0.15
        self.driveMotor.config_kI(1, self.drivemotors_pidff_ki)
        self.driveMotor.config_kD(1, self.drivemotors_pidff_kd)

        if self.drivemotors_integratedpid:
            self.driveMotor.selectProfileSlot(1,0)
        else:
            self.driveMotor.selectProfileSlot(0,0)

        # Rotate PID
        self.angleMotor.config_kP(1, self.anglemotors_pidff_kp) # 1.2
        self.angleMotor.config_kI(1, self.anglemotors_pidff_ki)
        self.angleMotor.config_kD(1, self.anglemotors_pidff_kd)
        self.angleMotor.config_kF(1, self.anglemotors_pidff_kf) # 0.14

        if self.anglemotors_integratedpid:
            self.angleMotor.selectProfileSlot(1,0)
        else:
            self.pidAngle.setP(self.anglemotors_pidff_kp)
            self.pidAngle.setI(self.anglemotors_pidff_ki)
            self.pidAngle.setD(self.anglemotors_pidff_kd)
            self.pidAngle.setConstraints(
                TrapezoidProfileRadians(
                    self.anglemotors_pidff_kv,
                    self.anglemotors_pidff_ka
                )
            )
            self.angleMotor.selectProfileSlot(0,0)

    def setDesiredState(self, desiredState:SwerveModuleState):
        # Optimize the reference state to avoid spinning further than 90 degrees and reversing drive motor power
        angleCurrentPosition = self.angleMotor.getSelectedSensorPosition(0)
        angleCurrentRotation = getRotationFromTicks( angleCurrentPosition, self.anglesensors_ticks )
        optimalState:SwerveModuleState = SwerveModuleState.optimize(
            desiredState,
            angleCurrentRotation  
        )

        # Drive Motor (open loop??? or closed loop PID)
        if self.drivemotors_integratedpid:
            # Get Velocity in Ticks per 100 ms
            osTp100ms = getVelocityMpsToTp100ms( optimalState.speed, self.drivemotors_ticks, self.wheelRadius )
            ####
            #### Do we want to add a Slew Rate Limiter Here?
            osTp100ms = self.driveSRL.calculate( osTp100ms )
            ####

            # Set Closed Loop Velocity
            self.driveMotor.set(
                ControlMode.Velocity,
                osTp100ms
            )
        else:
            ### Software Drive PID
            driveOutput = self.pidDrive.calculate(
                angleCurrentRotation.radians(),
                optimalState.angle.radians()
            )
            driveFFwd = self.ffDrive.calculate(
                self.pidDrive.getSetpoint().velocity
            )
            self.driveMotor.setVoltage( driveOutput + driveFFwd )

        # Angle Motor (closed loop PID)
        if self.anglemotors_integratedpid:
            # Get Target Position
            osTicks = getTicksFromRotation(optimalState.angle, self.anglesensors_ticks) # Get Optomized Target as Ticks
            osTicks = getContinuousInputMeasurement(angleCurrentPosition, osTicks, self.anglesensors_ticks) # Correction for [-180,180)
            
            # Set Closed Loop
            self.angleMotor.set(
                ControlMode.Position,
                osTicks
            )
        else:
            ### Software Angle PID
            angleOutput = self.pidAngle.calculate(
                angleCurrentRotation.radians(),
                optimalState.angle.radians()
            )
            self.angleMotor.setVoltage( angleOutput )

    def getModulePosition(self) -> Translation2d:
        return self.modulePosition

    def getPosition(self) -> SwerveModulePosition:
        dPosition = self.driveMotor.getSelectedSensorPosition(0)
        driveMeters = getDistanceTicksToMeters( dPosition, self.drivemotors_ticks, self.wheelRadius, self.drivemotors_gearratio )
        rPosition = self.angleMotor.getSelectedSensorPosition(0) ## Should we use self.angleSensor ??
        rotatePosition = getRotationFromTicks( rPosition, self.anglesensors_ticks )
        
        return SwerveModulePosition(
            distance = driveMeters, #Meters
            angle = rotatePosition # Angle
        )
