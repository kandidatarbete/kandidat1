##
## SessionLog.tcl 
## CarMaker 3.0 ScriptControl Example - IPG Automotive GmbH (www.ipg.de)
##
## The example shows the functions used to write to the CarMaker 
## Session Log file. 
## Please note: without a running application, the use of these functions
## does not make sense.
##  
## SessionLogMsg  <msg>
##      where  <msg> is the message to be written to the Log 
##  
## SessionLogWarn <warn_msg>
##      where <warn_msg> is the warning message to send to the Log
##  
## SessionLogError <err_msg> 
##      where   <err_msg> is the error message to send to the Log 
##
## $Id: SessionLog.tcl,v 1.3 2012/11/12 15:00:17 vw Exp $


# Write a log message
SessionLogMsg "Hello World"

# Write a warning message
SessionLogWarn "Objects are closer than they appear!"

# Write an error message
SessionLogError "Your socks don't match!"

Log "Take a look at the Session Log!"
Log "The messages should be shown within a few seconds."

