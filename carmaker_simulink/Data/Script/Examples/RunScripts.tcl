##
## RunScripts.tcl 
## CarMaker 3.0 ScriptControl Example - IPG Automotive GmbH (www.ipg.de)
##
## $Id: RunScripts.tcl,v 1.3 2012/11/12 15:00:17 vw Exp $


# A script can specified with a path relative to current script
RunScript StartStop.tcl
RunScript WaitForCondition.tcl
RunScript Math.tcl

# Absolute paths can also be used, but should be avoided.
# Example:
# RunScript /home/hil/Scripts/StartStop.tcl


