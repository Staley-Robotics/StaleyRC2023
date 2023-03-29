import sys

from . import Subsystems
from .__func__ import *
from ctre import WPI_TalonFX, ControlMode, FeedbackDevice, RemoteFeedbackDevice, NeutralMode
from ctre.sensors import *  # WPI_Pigeon2, WPI_CANCoder, SensorInitializationStrategy, AbsoluteSensorRange
from wpilib import SmartDashboard, XboxController, DriverStation, Timer
from wpimath import applyDeadband
from wpimath.controller import PIDController, ProfiledPIDController, ProfiledPIDControllerRadians, \
    HolonomicDriveController, SimpleMotorFeedforwardMeters
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveDrive4Odometry, SwerveModulePosition, SwerveModuleState, \
    ChassisSpeeds
from wpimath.trajectory import TrapezoidProfile, TrapezoidProfileRadians, TrajectoryGenerator, TrajectoryConfig, Trajectory


class SwerveDrive4(Subsystems):
    cSpeeds: ChassisSpeeds = ChassisSpeeds()
    op1 = XboxController(0)
    halfSpeed = False
    isRed = True

    def initSubsystem(self):
        SmartDashboard.putBoolean("halfSpeed", self.halfSpeed)

        # Gyro
        gyroId: int = int(self.ntCfgs.getNumber("gyroId", 61))
        gyroCanbus: str = self.ntCfgs.getString("gyroCanbus", "rio")
        self.gyro = WPI_Pigeon2(gyroId, gyroCanbus)
        self.gyro.setYaw(180)

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
        self.odometry = self.setOdometry()

        self.slx = SlewRateLimiter(2)
        self.sly = SlewRateLimiter(2)

        self.ntLimelight = self.ntInst.getTable("limelight-one")

        try:
            alliance = DriverStation.getAlliance()
            self.isRed = (alliance == DriverStation.Alliance.kRed)
        except Exception as e:
            print( "Swerve:", e )


    def initVariables(self):
        # Field Relative
        self.fieldRelative = self.ntVars.getBoolean("fieldRelative", True)

        # ChassisSpeed Velocities
        self.speed_linear_maxvelocity = self.ntVars.getNumber("speed/linear/maxVelocity", 1.0)
        self.speed_linear_maxvelocityx = self.ntVars.getNumber("speed/linear/maxVelocityX",
                                                               self.speed_linear_maxvelocity)
        self.speed_linear_maxvelocityy = self.ntVars.getNumber("speed/linear/maxVelocityY",
                                                               self.speed_linear_maxvelocity)
        self.speed_angular_maxvelocity = self.ntVars.getNumber("speed/angular/maxVelocity", (8 * math.pi))
        self.speed_angular_maxacceleration = self.ntVars.getNumber("speed/angular/maxAcceleration", (4 * math.pi))

        self.pathConfig = TrajectoryConfig( 1.5, 0.75 )

        # Chassis X Direction PID
        self.chassiscontrol_x_kp = self.ntVars.getNumber("ChassisControl/X/kP",1.0)
        self.chassiscontrol_x_ki = self.ntVars.getNumber("ChassisControl/X/kI",0.0)
        self.chassiscontrol_x_kd = self.ntVars.getNumber("ChassisControl/X/kD",0.0)
        self.chassiscontrol_x_ks = self.ntVars.getNumber("ChassisControl/X/kS",0.0)
        self.chassiscontrol_x_kv = self.ntVars.getNumber("ChassisControl/X/kV",0.0)
        self.chassiscontrol_x_ka = self.ntVars.getNumber("ChassisControl/X/kA",0.0)
        self.chassiscontrol_x_tolerance = self.ntVars.getNumber("ChassisControl/X/tolerance",1.0)

        self.chassiscontrol_y_kp = self.ntVars.getNumber("ChassisControl/Y/kP",1.0)
        self.chassiscontrol_y_ki = self.ntVars.getNumber("ChassisControl/Y/kI",0.0)
        self.chassiscontrol_y_kd = self.ntVars.getNumber("ChassisControl/Y/kD",0.0)
        self.chassiscontrol_y_ks = self.ntVars.getNumber("ChassisControl/Y/kS",0.0)
        self.chassiscontrol_y_kv = self.ntVars.getNumber("ChassisControl/Y/kV",0.0)
        self.chassiscontrol_y_ka = self.ntVars.getNumber("ChassisControl/Y/kA",0.0)
        self.chassiscontrol_y_tolerance = self.ntVars.getNumber("ChassisControl/Y/tolerance",1.0)

        # Chassis Omega PID
        self.chassiscontrol_omega_kp = self.ntVars.getNumber("chassisControl/omega/kP", 1.0)
        self.chassiscontrol_omega_ki = self.ntVars.getNumber("chassisControl/omega/kI", 0.0)
        self.chassiscontrol_omega_kd = self.ntVars.getNumber("chassisControl/omega/kD", 0.0)
        self.chassiscontrol_omega_ks = self.ntVars.getNumber("chassisControl/omega/kS", 0.0)
        self.chassiscontrol_omega_kv = self.ntVars.getNumber("chassisControl/omega/kV", 0.0)
        self.chassiscontrol_omega_ka = self.ntVars.getNumber("chassisControl/omega/kA", 0.0)
        self.chassiscontrol_omega_tolerance = self.ntVars.getNumber("chassisControl/omega/tolerance", 0.05)

        self.initPidControllers()

    def updateVariableSubsystems(self):
        self.updatePidControllers()

    def initPidControllers(self):
        # Create X Direction PID Controller
        self.pidX = PIDController(
            self.chassiscontrol_x_kp,
            self.chassiscontrol_x_ki,
            self.chassiscontrol_x_kd
        )
        self.pidX.setTolerance( self.chassiscontrol_x_tolerance )

        # Create Y Direction PID Controller
        self.pidY = PIDController(
            self.chassiscontrol_y_kp,
            self.chassiscontrol_y_ki,
            self.chassiscontrol_y_kd
        )
        self.pidY.setTolerance( self.chassiscontrol_y_tolerance )

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
        self.pidH = HolonomicDriveController(self.pidX, self.pidY, self.pidO)
        self.pidH.setTolerance(
            Pose2d( 0.01, 0.01, 0.01 )
        )

    def updatePidControllers(self):
        # Update X Direction PID Controller
        pidX = self.pidH.getXController()
        pidX.setP(self.chassiscontrol_x_kp)
        pidX.setI(self.chassiscontrol_x_ki)
        pidX.setD(self.chassiscontrol_x_kd)
        pidX.setTolerance( self.chassiscontrol_x_tolerance )

        # Update Y Direction PID Controller
        pidY = self.pidH.getYController()
        pidY.setP(self.chassiscontrol_y_kp)
        pidY.setI(self.chassiscontrol_y_ki)
        pidY.setD(self.chassiscontrol_y_kd)
        pidY.setTolerance( self.chassiscontrol_y_tolerance )

        # Update Omega PID Controller
        pidO = self.pidH.getThetaController()
        pidO.setP(self.chassiscontrol_omega_kp)
        pidO.setI(self.chassiscontrol_omega_ki)
        pidO.setD(self.chassiscontrol_omega_kd)
        pidO.setConstraints(
            TrapezoidProfileRadians.Constraints(
                self.chassiscontrol_omega_kv,
                self.chassiscontrol_omega_ka
            )
        )
        pidO.setTolerance(self.chassiscontrol_omega_tolerance)

        # Update Holonomic PID Controller
        # self.pidH.setTolerance()

    def run(self):

        if self.op1.getYButtonPressed():
            try:
                self.path = TrajectoryGenerator.generateTrajectory(
                    self.odometry.getEstimatedPosition(),
                    [],
                    Pose2d( 4, 3.15, Rotation2d(0).fromDegrees(180) ),
                    self.pathConfig
                )
                self.pathClock = Timer()
                self.pathClock.start()
            except Exception as e:
                print( "YPressed:", e )
        if self.op1.getYButtonReleased():
            self.path = None
            self.pathClock.stop()
        if self.op1.getYButton():
            try:
                nextState = self.path.sample( self.pathClock.get() )
                #print( nextState.pose )
                pose = Pose2d( 4, 3.15, Rotation2d(0).fromDegrees(180) )
                self.driveToState( nextState )
                #self.driveToPose( nextState.pose )
            except Exception as e:
                print( "Y Held:", e )
            return


        # Process Controller Inputs
        lx, ly, lt, rx, ry, rt = self.getInputs()

        # Drive
        if self.op1.getBButton():
            self.balance()
        elif rx != 0.0 or ry != 0.0:
            self.driveWithRotate(lx, ly, rx, ry)
        else:
            omega = (lt - rt)
            self.drive(lx, ly, omega)

    def getInputs(self) -> tuple[float, float, float, float, float, float]:
        lx = applyDeadband(-self.op1.getLeftY(), 0.1, 1.0)
        slx = self.slx.calculate(lx)
        ly = applyDeadband(-self.op1.getLeftX(), 0.1, 1.0)
        sly = self.sly.calculate(ly)
        lt = applyDeadband(self.op1.getLeftTriggerAxis(), 0.1, 1.0)
        rx = 0.0  # applyDeadband(-self.op1.getRightY(), 0.1, 1.0)
        ry = 0.0  # applyDeadband(-self.op1.getRightX(), 0.1, 1.0)
        rt = applyDeadband(self.op1.getRightTriggerAxis(), 0.1, 1.0)

        if self.op1.getRightBumperPressed():
            self.halfSpeed = not self.halfSpeed
            SmartDashboard.putBoolean("halfSpeed", self.halfSpeed)

        if self.halfSpeed:
            slx *= 0.5
            sly *= 0.5
            lt *= 0.5
            rt *= 0.5

        return slx, sly, lt, rx, ry, rt

    def balance(self):
        print(self.gyro.getPitch())
        offset = applyDeadband(self.gyro.getRoll(), 2.5, 360) * math.ceil((self.gyro.getYaw() / 360) % 1) * 0.02
        self.drive(offset, 0, 0)
        return abs(offset) < 0.05
        # flip = 90 > (self.gyro.getYaw() % 360) > -90
        # if self.gyro.getRoll() > 5 or self.gyro.getPitch() < -5:
        #     self.drive(-0.5 if flip else 0.5, 0, 0)
        # elif self.gyro.getRoll() < -5 or self.gyro.getPitch() > 5:
        #     self.drive(0.5 if flip else -0.5, 0, 0)
        # else:
        #     self.drive(0.0, 0, 0)
        # return abs(self.gyro.getRoll()) < 5

    def driveToState(self, state:Trajectory.State ):
        cSpeed = self.pidH.calculate(
            self.odometry.getEstimatedPosition(),
            state,
            state.pose.rotation()
        )
        self.updateSwerveModules( cSpeed )
        print( self.pidH.atReference() )

    def driveToPose(self, pose: Pose2d, velocity: float = None):
        if velocity is None:
            velocity = self.speed_linear_maxvelocity

        currentPose = self.odometry.getEstimatedPosition()
        cSpeed = self.pidH.calculate(
            currentPose,
            pose,
            velocity,
            pose.rotation()
        )
        #self.updateSwerveModules(cSpeed)
        # print( currentPose, pose ) #, self.pidH.atReference() )
        x = self.pidX.calculate( currentPose.X(), pose.X() )
        y = self.pidY.calculate( currentPose.Y(), pose.Y() )
        print( x, y, pose.rotation() )
        rot = self.driveToRotation( x, y, 1, rotation=pose.rotation() )

        #return self.pidX.atSetpoint() and self.pidY.atSetpoint() and rot

    def driveWithRotate(self, lx=0.0, ly=0.0, rx=0.0, ry=0.0):
        magnitude = math.sqrt(rx * rx + ry * ry)
        rotation = Rotation2d(rx, ry)
        dtr = self.driveToRotation(lx, ly, magnitude, rotation)
        return dtr

    def driveToRotation(self, x, y, magnitude=0.0, rotation: Rotation2d = Rotation2d(0, 0)):
        robotAngle = self.gyro.getRotation2d()

        currentRad = robotAngle.radians()
        newRad = rotation.radians()
        omegaTarget = self.pidO.calculate(currentRad, newRad)

        omega = max(abs(magnitude), 1) * omegaTarget
        if omega > 1.0:
            omega = 1.0
        elif omega < -1.0:
            omega = -1.0
        omega = applyDeadband(omega, self.chassiscontrol_omega_tolerance)
        self.drive(x, y, omega)
        if omega == 0:
            return True
        else:
            return False

    def drive(self, x=0.0, y=0.0, omega=0.0):
        if self.fieldRelative:
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                vx=x * self.speed_linear_maxvelocityx,
                vy=y * self.speed_linear_maxvelocityy,
                omega=omega * self.speed_angular_maxvelocity,
                robotAngle=self.gyro.getRotation2d()
            )
        else:
            speeds = ChassisSpeeds(
                vx=x * self.speed_linear_maxvelocityx,
                vy=y * self.speed_linear_maxvelocityy,
                omega=omega * self.speed_angular_maxvelocity,
            )
        self.cSpeeds = speeds  ## FOR SIMULATION
        self.updateSwerveModules(speeds)

    def updateSwerveModules(self, speeds: ChassisSpeeds):
        rotationCenter = Translation2d(0, 0)
        modStates = self.kinematics.toSwerveModuleStates(speeds, rotationCenter)
        modStates = SwerveDrive4Kinematics.desaturateWheelSpeeds(modStates, self.speed_linear_maxvelocity)

        self.moduleFL.setDesiredState(modStates[0])
        self.moduleFR.setDesiredState(modStates[1])
        self.moduleBL.setDesiredState(modStates[2])
        self.moduleBR.setDesiredState(modStates[3])

        #self.updateOdometry()

    def setOdometry(self):
        odom = SwerveDrive4PoseEstimator(
            self.kinematics,
            self.gyro.getRotation2d(),
            (
                self.moduleFL.getPosition(),
                self.moduleFR.getPosition(),
                self.moduleBL.getPosition(),
                self.moduleBR.getPosition()
            ),
            Pose2d(Translation2d(0, 0), Rotation2d(0).fromDegrees(180))
        )
        return odom

    def updateOdometry(self):
        # Limelight Data
        try:
            aprilTag = self.ntLimelight.getNumber("tid",-1)
            if aprilTag != -1:
                if self.isRed:
                    botPose = self.ntLimelight.getNumberArray("botpose_wpired",None)
                else:
                    botPose = self.ntLimelight.getNumberArray("botpose_wpiblue",None)

                if botPose is None: return

                llx = botPose[0]
                lly = botPose[1]
                llz = botPose[5]
                llPose = Pose2d(llx,lly, Rotation2d(0).fromDegrees(llz))
                llTime = botPose[6]
                self.odometry.addVisionMeasurement(
                    llPose,
                    (Timer.getFPGATimestamp() - (llTime/1000)),
                )
        except Exception as e:
            print( "Update Odometry: ", e )

        pose = self.odometry.updateWithTime(
            Timer.getFPGATimestamp(),
            self.gyro.getRotation2d(),
            [
                self.moduleFL.getPosition(),
                self.moduleFR.getPosition(),
                self.moduleBL.getPosition(),
                self.moduleBR.getPosition()
            ]
        )

        SmartDashboard.putNumberArray("Field/SwerveNew", [pose.X(), pose.Y(), pose.rotation().degrees()])


class SwerveModule(Subsystems):
    def initSubsystem(self):
        driveMotorId: int = int(self.ntCfgs.getNumber("driveMotorId", 1))
        driveMotorCanbus: str = self.ntCfgs.getString("driveMotorCanbus", "rio")
        angleMotorId: int = int(self.ntCfgs.getNumber("angleMotorId", 2))
        angleMotorCanbus: str = self.ntCfgs.getString("angleMotorCanbus", "rio")
        angleSensorId: int = int(self.ntCfgs.getNumber("angleSensorId", 3))
        angleSensorCanbus: str = self.ntCfgs.getString("angleSensorCanbus", "rio")

        modulePositionX: float = self.ntCfgs.getNumber("modulePositionX", 0.25)
        modulePositionY: float = self.ntCfgs.getNumber("modulePositionY", 0.25)
        self.modulePosition = Translation2d(modulePositionX, modulePositionY)

        driveMotorReversed: bool = self.ntCfgs.getBoolean("driveMotorReversed", False)
        driveMotorPhase: bool = self.ntCfgs.getBoolean("driveMotorPhase", False)

        angleMotorReversed: bool = self.ntCfgs.getBoolean("angleMotorReversed", True)
        angleMotorPhase: bool = self.ntCfgs.getBoolean("angleMotorPhase", True)

        angleSensorReversed: bool = self.ntCfgs.getBoolean("angleSensorReversed", False)
        angleSensorOffset: float = self.ntCfgs.getNumber("angleSensorOffset", 0.0)

        # Angle Sensor
        self.angleSensor: WPI_CANCoder = WPI_CANCoder(angleSensorId, angleSensorCanbus)
        self.angleSensor.configFactoryDefault()
        self.angleSensor.configSensorInitializationStrategy(SensorInitializationStrategy.BootToZero)
        self.angleSensor.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360)
        self.angleSensor.configSensorDirection(angleSensorReversed)
        self.angleSensor.setPosition(self.angleSensor.getAbsolutePosition() - angleSensorOffset)

        # Angle Motor
        self.angleMotor: WPI_TalonFX = WPI_TalonFX(angleMotorId, angleMotorCanbus)
        self.angleMotor.configFactoryDefault()
        self.angleMotor.configRemoteFeedbackFilter(self.angleSensor, 0)
        self.angleMotor.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0, 0)
        self.angleMotor.configSelectedFeedbackSensor(FeedbackDevice.None_, 1)
        self.angleMotor.setSensorPhase(angleMotorPhase)
        self.angleMotor.setInverted(angleMotorReversed)
        self.angleMotor.setNeutralMode(NeutralMode.Coast)
        self.angleMotor.configFeedbackNotContinuous(True)

        # Drive Motor
        self.driveMotor: WPI_TalonFX = WPI_TalonFX(driveMotorId, driveMotorCanbus)
        self.driveMotor.configFactoryDefault()
        self.driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor)
        self.driveMotor.setInverted(driveMotorReversed)
        self.driveMotor.setSensorPhase(driveMotorPhase)
        self.driveMotor.setNeutralMode(NeutralMode.Coast)

    def initVariables(self):
        # Integrated PID
        self.drivemotors_integratedpid: bool = self.ntVars.getBoolean("driveMotors/integratedPid", True)
        self.anglemotors_integratedpid: bool = self.ntVars.getBoolean("angleMotors/integratedPid", True)

        # Integrated PID
        self.drivemotors_slewratesteps: int = self.ntVars.getNumber("driveMotors/slewRateSteps", 200000)

        # Physical Mechanics
        self.wheel_radius: float = self.ntVars.getNumber("wheel/radius", 0.0508)
        self.drivemotors_ticks: float = self.ntVars.getNumber("driveMotors/ticks", 2048.0)
        self.anglemotors_ticks: float = self.ntVars.getNumber("angleMotors/ticks", 2048.0)
        self.anglesensors_ticks: float = self.ntVars.getNumber("angleSensors/ticks", 4096.0)

        self.drivemotors_gearratio: float = self.ntVars.getNumber("driveMotors/gearRatio", 1.0)

        # Drive PID Mechanics
        self.drivemotors_pidff_kp: float = self.ntVars.getNumber("driveMotors/pidFf/kP", 0.15)
        self.drivemotors_pidff_ki: float = self.ntVars.getNumber("driveMotors/pidFf/kI", 0.0)
        self.drivemotors_pidff_kd: float = self.ntVars.getNumber("driveMotors/pidFf/kD", 0.0)
        self.drivemotors_pidff_kf: float = self.ntVars.getNumber("driveMotors/pidFf/kF", 0.065)  # recommend start here
        self.drivemotors_pidff_ks: float = self.ntVars.getNumber("driveMotors/pidFf/kS", 0.0)
        self.drivemotors_pidff_kv: float = self.ntVars.getNumber("driveMotors/pidFf/kV", 0.0)
        self.drivemotors_pidff_ka: float = self.ntVars.getNumber("driveMotors/pidFf/kA", 0.0)
        self.drivemotors_pidff_tolerance: float = self.ntVars.getNumber("driveMotors/pidFf/tolerance", 0.0)

        # Rotate PID Mechanics
        self.anglemotors_pidff_kp: float = self.ntVars.getNumber("angleMotors/pidFf/kP",
                                                                 0.5)  # recommended: (throttle*1023/4096) ~ 0.125
        self.anglemotors_pidff_ki: float = self.ntVars.getNumber("angleMotors/pidFf/kI", 0.0)  # recommended: 1% of kP
        self.anglemotors_pidff_kd: float = self.ntVars.getNumber("angleMotors/pidFf/kD",
                                                                 0.0)  # recommended: 10-100x of kP
        self.anglemotors_pidff_kf: float = self.ntVars.getNumber("angleMotors/pidFf/kF", 0.0)
        self.anglemotors_pidff_ks: float = self.ntVars.getNumber("angleMotors/pidFf/kS", 0.0)
        self.anglemotors_pidff_kv: float = self.ntVars.getNumber("angleMotors/pidFf/kV", 0.0)
        self.anglemotors_pidff_ka: float = self.ntVars.getNumber("angleMotors/pidFf/kA", 0.0)
        self.anglemotors_pidff_tolerance: float = self.ntVars.getNumber("angleMotors/pidFf/tolerance", 0.0)

        self.initPidControllers()
        self.initSlewRateLimiter()

    def updateVariableSubsystems(self):
        self.updatePidControllers()
        self.initSlewRateLimiter()  ### Should we consider only updating this during reboot?

    def initSlewRateLimiter(self):
        self.driveSRL = SlewRateLimiter(self.drivemotors_slewratesteps)

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
        self.pidAngle.enableContinuousInput(-math.pi, math.pi)
        self.updatePidControllers()

    def updatePidControllers(self):
        self.driveMotor.config_kF(1, self.drivemotors_pidff_kf)  # 0.065
        self.driveMotor.config_kP(1, self.drivemotors_pidff_kp)  # 0.15
        self.driveMotor.config_kI(1, self.drivemotors_pidff_ki)
        self.driveMotor.config_kD(1, self.drivemotors_pidff_kd)

        if self.drivemotors_integratedpid:
            self.driveMotor.selectProfileSlot(1, 0)
        else:
            self.driveMotor.selectProfileSlot(0, 0)

        # Rotate PID
        self.angleMotor.config_kP(1, self.anglemotors_pidff_kp)  # 1.2
        self.angleMotor.config_kI(1, self.anglemotors_pidff_ki)
        self.angleMotor.config_kD(1, self.anglemotors_pidff_kd)
        self.angleMotor.config_kF(1, self.anglemotors_pidff_kf)  # 0.14

        if self.anglemotors_integratedpid:
            self.angleMotor.selectProfileSlot(1, 0)
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
            self.angleMotor.selectProfileSlot(0, 0)

    def setDesiredState(self, desiredState: SwerveModuleState):
        # Optimize the reference state to avoid spinning further than 90 degrees and reversing drive motor power
        angleCurrentPosition = self.angleMotor.getSelectedSensorPosition(0)
        angleCurrentRotation = getRotationFromTicks(angleCurrentPosition, self.anglesensors_ticks)
        optimalState: SwerveModuleState = SwerveModuleState.optimize(
            desiredState,
            angleCurrentRotation
        )

        # Drive Motor (open loop??? or closed loop PID)
        if self.drivemotors_integratedpid:
            # Get Velocity in Ticks per 100 ms
            osTp100ms = getVelocityMpsToTp100ms(optimalState.speed, self.drivemotors_ticks, self.wheel_radius,
                                                self.drivemotors_gearratio)
            ####
            #### Do we want to add a Slew Rate Limiter Here?
            osTp100ms = self.driveSRL.calculate(osTp100ms)
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
            self.driveMotor.setVoltage(driveOutput + driveFFwd)

        # Angle Motor (closed loop PID)
        if self.anglemotors_integratedpid:
            # Get Target Position
            osTicks = getTicksFromRotation(optimalState.angle, self.anglesensors_ticks)  # Get Optomized Target as Ticks
            osTicks = getContinuousInputMeasurement(angleCurrentPosition, osTicks,
                                                    self.anglesensors_ticks)  # Correction for [-180,180)

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
            self.angleMotor.setVoltage(angleOutput)

    def getModulePosition(self) -> Translation2d:
        return self.modulePosition

    def getPosition(self) -> SwerveModulePosition:
        dPosition = self.driveMotor.getSelectedSensorPosition(0)
        driveMeters = getDistanceTicksToMeters(dPosition, self.drivemotors_ticks, self.wheel_radius,
                                               self.drivemotors_gearratio)
        rPosition = self.angleMotor.getSelectedSensorPosition(0)  ## Should we use self.angleSensor ??
        rotatePosition = getRotationFromTicks(rPosition, self.anglesensors_ticks)

        return SwerveModulePosition(
            distance=driveMeters,  # Meters
            angle=rotatePosition  # Angle
        )
