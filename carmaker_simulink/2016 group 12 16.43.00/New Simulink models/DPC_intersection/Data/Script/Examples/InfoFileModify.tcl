##
## InfoFileModify.tcl 
## CarMaker 3.0 ScriptControl Example - IPG Automotive GmbH (www.ipg.de)
##
## This example shows how to modify parameter files (InfoFile keys).
##
## IFileModify <file> <key> <value>
## IFileModify takes three arguments
##  1) The "Option" argument is used to determine what info file
##     should be modified. Currently "Vehicle", "TestRun" or 
##     Brake can be options.
##  2) Key Name
##  3) New Value of the Key
##  
## The Test Run must be loaded prior to calling the function.
##
## $Id: InfoFileModify.tcl,v 1.5 2012/11/12 15:00:17 vw Exp $

Log "* The test run is loaded"
LoadTestRun "Examples/CarMakerFunctions/IPGRoad/Hockenheim"

Log "* Make a working copy of Testrun Hockenheim"
SaveTestRun "MyOwnTestRun"

Log ""
Log "* Modify parameters (vehicle load 0)"
set mass0 [IFileRead TestRun "VehicleLoad.0.mass"]
set pos0  [IFileRead TestRun "VehicleLoad.0.pos"]

IFileModify TestRun "VehicleLoad.0.mass" 500 
IFileModify TestRun "VehicleLoad.0.pos" "1.5 0 1.5" 

Log "* Flush modifications and run simulation..."
IFileFlush

StartSim
WaitForStatus running
Sleep 10000
StopSim
WaitForStatus idle 10000

Log ""
Log "* Reset parameters to the original values."
IFileModify TestRun "VehicleLoad.0.mass" $mass0 
IFileModify TestRun "VehicleLoad.0.pos"  $pos0 

Log "* Flush modifications and run simulation..."
IFileFlush

StartSim
WaitForStatus running
Sleep 10000
StopSim

