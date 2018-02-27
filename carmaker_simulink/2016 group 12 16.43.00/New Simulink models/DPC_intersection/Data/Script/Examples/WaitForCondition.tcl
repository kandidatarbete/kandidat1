##
## WaitForCondition.tcl 
## CarMaker 3.0 ScriptControl Example - IPG Automotive GmbH (www.ipg.de)
##
## This Example shows how to use the WaitForCondition function.               
## It is important to note that any variables that are passed in the          
## conditional statement must be global variables. In this example            
## the Qu array, declared with global scope, is used. WaitForCondition
## will wait for the velocity of the car to exceed 50 kph. Once the velocity  
## is greater than 50 kph, the speed is printed and the simulation is stopped.
## 
## $Id: WaitForCondition.tcl,v 1.5 2012/11/12 15:00:17 vw Exp $


### Subscribe all needed quantities
QuantSubscribe Car.v

## Load the test run
LoadTestRun "Examples/CarMakerFunctions/IPGRoad/Hockenheim"

## Make sure the previous simulation is idle then start sim 
WaitForStatus idle
StartSim
WaitForStatus running

WaitForCondition {$Qu(Car.v) > 10}
Log "The Velocity is $Qu(Car.v) m/s"

WaitForCondition {$Qu(Car.v) > 20}
Log "The Velocity is $Qu(Car.v) m/s"

WaitForCondition {$Qu(Car.v) > 30}
Log "The Velocity is $Qu(Car.v) m/s"

## Print the Velocity and stop the simulation. 
Log "The Velocity is $Qu(Car.v) m/s"
StopSim

WaitForStatus idle

