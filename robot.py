import time
from wpilib import * # TimedRobot, run
from ntcore import NetworkTableInstance, Event, EventFlags

from build import Build

from subsystems import Subsystems
from subsystems.auto import Auto
from subsystems.swervedrive import SwerveDrive4
from subsystems.pneumatics import Pneumatics
from subsystems.arm import Arm
from subsystems.claw import Claw
#from subsystems.bumper import Bumper
#from subsystems.limelight import Limelight


class Robot(TimedRobot):
    ntInst: NetworkTableInstance
    subsystems: list[Subsystems]
    swerve: SwerveDrive4
    arm: Arm
    pneumatics_system: Pneumatics
    claw: Claw
    auto: Auto
    resetSpeeds: list[float]

    def robotInit(self):
        # Initialization Wait for Canbus (Known Issue)
        time.sleep(2)

        # Get Build Settings
        Build().buildInitConfig()
        Build().buildVariables()

        # Connect to NetworkTables
        self.ntInst = NetworkTableInstance.getDefault()
        self.ntTbl = self.ntInst.getTable("Startup")

        pdm = PowerDistribution(0, PowerDistribution.ModuleType.kCTRE)
        pdm.clearStickyFaults()
        # Build Subsystems
        self.subsystems = []
        try:
            self.swerve = SwerveDrive4()
        except:
            pass
        self.arm = Arm()
        self.pneumatics_system = Pneumatics()
        self.claw = Claw(self.pneumatics_system)
        self.subsystems.append(self.swerve)
        self.subsystems.append(self.arm)
        self.subsystems.append(self.claw)

        time.sleep(1)
        self.ntTbl.putBoolean("leftS", False)
        self.ntTbl.setPersistent("leftS")
        self.ntTbl.putBoolean("centerS", False)
        self.ntTbl.setPersistent("centerS")
        self.ntTbl.putBoolean("rightS", False)
        self.ntTbl.setPersistent("rightS")

    def robotPeriodic(self):
        #print(
        #    self.subsystems[0].moduleFR.angleMotor.getSelectedSensorPosition(),
        #    self.subsystems[0].moduleFR.angleSensor.getAbsolutePosition(),
        #    self.subsystems[0].moduleFR.angleSensor.getPosition()
        #)
        self.pneumatics_system.run()

    def autonomousInit(self):
        self.auto = Auto(self.swerve, self.arm, self.claw)
        self.auto.using_path = False
        if self.ntTbl.getBoolean("centerS", False):
            self.auto.set_mode("AutonomousLeft")
        elif self.ntTbl.getBoolean("leftS", False):
            self.auto.set_mode("AutonomousLeft")
        elif self.ntTbl.getBoolean("rightS", False):
            self.auto.set_mode("AutonomousLeft")
        else:
            self.auto.set_mode("AutonomousLeft")

        self.auto.index = 0
        self.resetSpeeds = [
            self.swerve.speed_linear_maxvelocity,
            self.swerve.speed_linear_maxvelocityx,
            self.swerve.speed_linear_maxvelocityy
        ]
        self.swerve.speed_linear_maxvelocity = 8
        self.swerve.speed_linear_maxvelocityx = 8
        self.swerve.speed_linear_maxvelocityy = 8
    def autonomousPeriodic(self):
        self.auto.run()
    def autonomousExit(self):
        self.swerve.speed_linear_maxvelocity = self.resetSpeeds[0]
        self.swerve.speed_linear_maxvelocityx = self.resetSpeeds[1]
        self.swerve.speed_linear_maxvelocityy = self.resetSpeeds[2]


    def teleopInit(self):
        for i in range(len(self.subsystems)):
            s: Subsystems = self.subsystems[i]
            s.runInit()

    def teleopPeriodic(self):
        for i in range(len(self.subsystems)):
            s:Subsystems = self.subsystems[i]
            s.run()

    def teleopExit(self): pass

    def testInit(self):
        # Load NT Table for Testing
        ntTest = self.ntInst.getTable("Testing")
        ntTestVars = ntTest.getTopics()
        for i in range(len(ntTestVars)):
            vName = ntTestVars[i].getName().removeprefix("/Testing/") 
            vValue = ntTest.getBoolean( vName, False )
            exec(f"self.test_{vName} = {vValue}")
        self._ntListener = self.ntInst.addListener(
            [ "/Testing" ],
            EventFlags.kValueAll,
            self.updateNtTestValues
        )
    def testPeriodic(self):
        for i in range(len(self.subsystems)):
            try:
                s:Subsystems = self.subsystems[i]
                sName = s.__class__.__name__
                runInTest = eval( f"self.test_{sName}" )
                if runInTest: s.run()
            except:
                pass
    def testExit(self):
        self.ntInst.removeListener(self._ntListener)

    def disabledInit(self): pass
    def disabledPeriodic(self): pass
    def disabledExit(self): pass

    def updateNtTestValues(self, event:Event):
        # Get Variable Name and New Value
        varName = event.data.topic.getName().removeprefix(f"/Testing/")
        newValue = event.data.value.value()
        # Set Variable
        exec( f"self.test_{varName} = {newValue}" )


if __name__ == '__main__':
    run(Robot)