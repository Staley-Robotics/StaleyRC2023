from ntcore import NetworkTableInstance, NetworkTable, NetworkTableEntry, NetworkTableType, Event, EventFlags
#from .arm import *
#from .bumper import *
#from .claw import *
#from .limelight import *
#from .pneumatics import *
#from .swervedrive import *

class Subsystems:
    ntInst:NetworkTableInstance = None
    ntCfgs:NetworkTable = None
    ntVars:NetworkTable = None

    def __init__(self, subComponent:str=None):
        cName = self.__class__.__name__
        # Network Table Instance
        self.ntInst = NetworkTableInstance.getDefault()
        
        # Config Path
        if subComponent is None:
            initStr = f"InitConfig/{cName}"
        else:
            initStr = f"InitConfig/{cName}/{subComponent}"

        # Network Tables
        self.ntCfgs = self.ntInst.getTable(f"{initStr}")
        self.ntVars = self.ntInst.getTable(f"Variables/{cName}")


        # Set Testing Variable
        ntTest = self.ntInst.getTable(f"Testing")
        eType = ntTest.getEntry(f"{cName}").getType()
        if eType == NetworkTableType.kUnassigned:
            ntTest.putBoolean(f"{cName}", False)
            ntTest.setPersistent(f"{cName}")

        # Initialize Subsystem
        self.initSubsystem()
        self.initVariables()
        self.initNtListener()

    # Network Tables Listener
    def initNtListener(self):
        cName = self.__class__.__name__
        self.ntInst.addListener(
            [ f"/Variables/{cName}" ],
            EventFlags.kValueAll,
            self.updateVariables
        )

    # Update 
    def updateVariables(self, event:Event):
        cName = self.__class__.__name__
        # Get Variable Name and New Value
        varName = event.data.topic.getName().removeprefix(f"/Variables/{cName}/").replace("/", "_").lower()
        newValue = event.data.value.value()
        # Set Variable
        exec( f"self.{varName} = {newValue}" )
        # Run Component Updates
        self.updateVariableSubsystems()

    # @Override Functions
    def initSubsystem(self): pass  ### Initialization
    def initVariables(self): pass  ### Initialize Variables from Network Tables
    def updateVariableSubsystems(self): pass  ### Update Subsystems that leverage variables
    def run(self): pass  ### Run Subsystem
    def runInit(self): pass
