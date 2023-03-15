import json
import os
from ntcore import *

class Build:
    def buildInitConfig(self):
        ntInst = NetworkTableInstance.getDefault()
        tbl = ntInst.getTable("Build")
        override:bool = tbl.getBoolean("initOverride", False)
        initTbl = ntInst.getTable("InitConfig")
        self.build(initTbl, "config/", persist=True, override=override)
        tbl.putBoolean("initOverride", False)
        tbl.setPersistent("initOverride")

    def buildVariables(self):
        ntInst = NetworkTableInstance.getDefault()
        tbl = ntInst.getTable("Build")
        override:bool = tbl.getBoolean("variableOverride", False)
        varTbl = ntInst.getTable("Variables")
        self.build(varTbl, "variables/", persist=True, override=override)
        tbl.putBoolean("variableOverride", False)
        tbl.setPersistent("variableOverride")

    def build(self, ntTable, folderPath, persist=False, override=False):
        myDir = os.path.dirname(__file__)
        searchDir = os.path.join( myDir, folderPath )
        files = os.listdir( searchDir )
        for file in files:
            # Only Load Json Files
            if not file.endswith(".json"):
                continue
            # Loop Through Files
            fullPath = os.path.join( searchDir, file )
            with open(f"{fullPath}", 'r') as f:
                content = json.load( f )
                for top in content:
                    self.loadLoop(ntTable, "", content, persist, override)

    def loadLoop(self, table:NetworkTable, path, content, persist=True, override=False):
        for key in content:
            if path == "":
                thisPath = f"{key}"
            else:
                thisPath = f"{path}/{key}"

            if type( content[key] ) != dict:
                if not override:
                    entry = table.getEntry( thisPath )
                    entryType = entry.getType()
                    if entryType != NetworkTableType.kUnassigned:
                        return
                table.putValue( thisPath, content[key] )
                if persist:
                    table.setPersistent( thisPath )
                else:
                    table.clearPersistent( thisPath )
            else:
                self.loadLoop( table, thisPath, content[key], persist, override)
