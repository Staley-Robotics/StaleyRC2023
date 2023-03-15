import time
from wpilib import * # TimedRobot, run
from ntcore import NetworkTableInstance, Event, EventFlags

from build import Build

from subsystems import Subsystems
from subsystems.auto import Auto
from subsystems.swervedrive import SwerveDrive4
#from subsystems.pneumatics import RobotCompressor
from subsystems.arm import Arm
from subsystems.claw import Claw
#from subsystems.bumper import Bumper
#from subsystems.limelight import Limelight


class Robot(TimedRobot):
    def robotInit(self): 
        # Initialization Wait for Canbus (Known Issue)
        time.sleep(2)

        # Get Build Settings
        Build().buildInitConfig()
        Build().buildVariables()

        # Connect to NetworkTables
        self.ntInst = NetworkTableInstance.getDefault()

        pdm = PowerDistribution(0, PowerDistribution.ModuleType.kCTRE)
        pdm.clearStickyFaults()
        # Build Subsystems
        self.subsystems = []

        # Add SwerveDrive to Subsystems
        try:
            self.subsystems.append( SwerveDrive4() )
        except Exception as e:
            print( e )
            pass

        # Add RobotCompressor to Subsystems
        #try:
        #    self.subsystems.append( RobotCompressor() )
        #except:
        #    pass

        # Add Arm to Subsystems
        try:
            self.subsystems.append( Arm() )
        except:
            pass

        # Add Claw to Subsystems
        try:
            self.subsystems.append( Claw() )
        except:
            pass

        # Add Bumper to Subsystems
        #try:
        #    self.subsystems.append( Bumper() )
        #except:
        #    pass

        # Add Limelight to Subsystems
        #try:
        #    self.subsystems.append( Limelight() )
        #except:
        #    pass

    def robotPeriodic(self):
        #print(
        #    self.subsystems[0].moduleFR.angleMotor.getSelectedSensorPosition(),
        #    self.subsystems[0].moduleFR.angleSensor.getAbsolutePosition(),
        #    self.subsystems[0].moduleFR.angleSensor.getPosition()
        #)
        pass

    def autonomousInit(self):
        self.auto = Auto(self.subsystems[0], self.subsystems[1], self.subsystems[2])
    def autonomousPeriodic(self):
        self.auto.run()
    def autonomousExit(self): pass
 
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
        ntTest = self.ntInst.getTable("Testing" )
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