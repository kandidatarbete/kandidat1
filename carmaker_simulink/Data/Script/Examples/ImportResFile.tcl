##
## ImportResFile.tcl  --  Import Results File
## CarMaker 3.0 ScriptControl Example - IPG Automotive GmbH (www.ipg.de)
##
## MODIFY THE SCRIPT:
## set the name of the result file to read.
##
## $Id: ImportResFile.tcl,v 1.3 2012/11/12 15:00:17 vw Exp $
#******************************************************************************

set file "SimOutput/Offline/<date>/<result_file_.erg>" 

ImportResFile $file {DM.Gas Vhcl.vx} Var

Log [lindex $Var(Vhcl.vx) 5]
