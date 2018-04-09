##
## MiniManJump.tcl 
## CarMaker 3.0 ScriptControl Example - IPG Automotive GmbH (www.ipg.de)
##
## This example shows how to jump from one mini manuever to the next.
##
## $Id: MiniManJump.tcl,v 1.5 2012/11/12 15:00:17 vw Exp $

Log "* Load Test Run and start simulation"
LoadTestRun "Examples/VehicleDynamics/Braking"

StartSim
WaitForStatus running
Sleep 10000

# jump to mini maneuver 1
Log "* Jump to mini maneuver 1"
ManJump 1

Sleep 5000

# jump to mini maneuver 0
Log "* Jump back to mini maneuver 0"
ManJump 0

Sleep 5000

# jump to mini maneuver 1
Log "* Jump to mini maneuver 1"
ManJump 1

Sleep 2000 

# jump to mini maneuver 0
Log "* Jump back to mini maneuver 0"
ManJump 0

# wait for the simulation to stop
WaitForStatus idle

