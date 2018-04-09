##
## SimStatus.tcl 
## CarMaker 3.0 ScriptControl Example - IPG Automotive GmbH (www.ipg.de)
## 
## This example illustrates the use of the SimStatus command.                                             
##                                                       
## In Verbose mode, you can see the output in the CarMaker log file.                                   
##                                                       
## The Sleep() function are used to slow down execution and make it 
## more viewable. They are normally not necessary.                                           
##
## $Id: SimStatus.tcl,v 1.4 2012/11/12 15:00:17 vw Exp $

LoadTestRun "Examples/CarMakerFunctions/IPGRoad/Hockenheim"

set status [SimStatus]
Log "Simulation status: $status\n"

Log "Start simulation"
StartSim

Log "Wait until the simulation runs"
WaitForStatus running
set status [SimStatus]
Log "Simulation status: $status\n"

Log "Simulate 10 seconds"
Sleep 10000

Log "Stop the simulation"
StopSim

Log "Wait until the simulation stops"
WaitForStatus idle
set status [SimStatus]
Log "Simulation status: $status\n"

