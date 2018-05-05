##
## Boolean.tcl 
## CarMaker 3.0 ScriptControl Example - IPG Automotive GmbH (www.ipg.de)
##
## This example shows how to make logical boolean 
## expression, and also how to use the bitwise operators
##
## $Id: Boolean.tcl,v 1.3 2012/11/12 15:00:17 vw Exp $

set Int1 32
set Int2 10

Log "++++++++++++++++++++++++++++++++++++++++++++++"
Log "The bitwise Not of $Int1 is            : [expr ~$Int1] "
Log "$Int1 left shifted by 2 is             : [expr 32 << 2]"
Log "$Int1 right shifted by 2 is            : [expr 32 >> 2]"
Log "Bitwise AND of $Int1 and $Int2 is         : [expr $Int1 & $Int2]"
Log "Bitwise OR  of $Int1 and $Int2 is         : [expr $Int1 | $Int2]"
Log "Bitwise XOR of $Int1 and $Int2 is         : [expr $Int1 ^ $Int2]"
Log "Is $Int1 > $Int2?  (True = 1, False = 0)  : [expr $Int1 > $Int2]"
Log "Is $Int1 < $Int2?  (True = 1, False = 0)  : [expr $Int1 < $Int2]"
Log "Is $Int1 = $Int2?  (True = 1, False = 0)  : [expr $Int1 == $Int2]"
Log "What is the Negation of ($Int1 > $Int2)?  : [expr !($Int1 > $Int2)]"
Log "++++++++++++++++++++++++++++++++++++++++++++++"

