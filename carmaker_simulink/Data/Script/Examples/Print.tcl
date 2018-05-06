##
## Print.tcl 
## CarMaker 3.0 ScriptControl Example - IPG Automotive GmbH (www.ipg.de)
##
## $Id: Print.tcl,v 1.5 2012/11/12 15:00:17 vw Exp $

# Files with no directory component will go to SimOutput/ScriptLog/<date>
OpenSLog PrintExample1.slog

Log         "This is going to screen, and (if one was opened) to file"
Log file    "This is going in the file (file MUST be opened)"
Log screen  "This is going to the screen"
Log file+   "This is going to file and screen (file MUST be opened)"
Log screen+ "This is going to screen and file (file MUST be opened)"

LogExec         {LoadTestRun "Examples/CarMakerFunctions/IPGRoad/Hockenheim"}
LogExec screen   StartSim
LogExec screen  {Sleep 10000}
LogExec screen   StopSim
LogExec screen  {Sleep 5000}

CloseSLog


# Files with no directory component will go to SimOutput/ScriptLog/<date>
OpenSLog PrintExample2.slog

WaitForStatus idle
Log ""
LogExec file    {LoadTestRun "Examples/VehicleDynamics/Braking"}
LogExec screen+ {Sleep 1000}
LogExec file     StartSim
LogExec file    {Sleep 10000}
LogExec file     StopSim
LogExec file    {Sleep 5000}

WaitForStatus idle
Log ""
LogExec screen+ {LoadTestRun "Examples/CarMakerFunctions/IPGRoad/Hockenheim"}
LogExec screen+ {Sleep 1000}
LogExec screen+  StartSim
LogExec screen+ {Sleep 10000}
LogExec screen+  StopSim
LogExec file    {Sleep 5000}

WaitForStatus idle
Log ""
LogExec file+   {LoadTestRun "Examples/CarMakerFunctions/IPGRoad/Road_Bumps"}
LogExec screen+ {Sleep 1000}
LogExec file+    StartSim
LogExec file+   {Sleep 10000}
LogExec file+    StopSim

CloseSLog

