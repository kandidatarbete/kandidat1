##
## DVA.tcl  --  Direct Variable Access
## CarMaker 3.0 ScriptControl Example - IPG Automotive GmbH (www.ipg.de)
##
## This example shows how to set the value of a variable using DVA.
## The functions introduced are:
##
## DVAWrite Name Value Count Mode
##    <Name>   - Quantity name   
##    <Value>  - New Quantity Value
##    <Count>  - Number of Cycles ... default 1 
##    <Mode>   - (0 = value, 1 = offset, 2 = factor) ... default 0
##    Sets the value of the named quantity using DVA.
##
## DVAReleaseQuants 
##    Releases the quantities from DVA control. The quantity value
##    not reset, and unless it is modified by some internal function,
##    it will keep the value it was given. 
##
## $Id: DVA.tcl,v 1.4 2012/11/12 15:00:17 vw Exp $


# Subscribe all needed quantities
QuantSubscribe DM.Steer.Ang


Log "* Load TestRun" 
LoadTestRun "Examples/VehicleDynamics/Braking"

Log "* Start Simulation..."
StartSim
WaitForStatus running

Sleep 1000
Log "  Wheel turned to Angle 2 rad."
DVAWrite DM.Steer.Ang 2 5000 0

Log ""
Log "  +++++++++++++++++++++++++++++++++++++++++++++"
Log "  Car went off the road at: "
Log "  Time                 =  $Qu(Time) seconds"
Log "  Steering Wheel Angle =  2 rad"
Log "  +++++++++++++++++++++++++++++++++++++++++++++\n" 
	    
               
#=======================================================================
# release the quantity from DVA control. The value will not change, 
# since load is not normally modified internally by CarMaker. However, 
# it is possible to change the load back to its original value by calling
# DVAWrite again, using the original value as the new value.
#=======================================================================
DVAReleaseQuants

