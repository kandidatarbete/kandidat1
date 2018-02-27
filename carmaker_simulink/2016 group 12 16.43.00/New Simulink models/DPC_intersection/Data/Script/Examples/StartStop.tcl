##
## StartStop.tcl 
## CarMaker 3.0 ScriptControl Example - IPG Automotive GmbH (www.ipg.de)
## 
## Load, start and stop test runs
## 
## $Id: StartStop.tcl,v 1.4 2012/11/12 15:00:17 vw Exp $

Log "* Run 15 seconds of Hockenheim"
LoadTestRun "Examples/CarMakerFunctions/IPGRoad/Hockenheim"
StartSim
WaitForStatus running
WaitForTime 15

StopSim
WaitForStatus idle 10000

Log "* Run 15 seconds of LaneChangeISO"
LoadTestRun "Examples/VehicleDynamics/LaneChange_ISO"
StartSim
WaitForStatus running
WaitForTime 15

StopSim

