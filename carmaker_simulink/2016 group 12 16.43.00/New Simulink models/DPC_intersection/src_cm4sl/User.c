/*
******************************************************************************
**  CarMaker - Version 5.0.3
**  Vehicle Dynamic Simulation Toolkit
**
**  Copyright (C)   IPG Automotive GmbH
**                  Bannwaldallee 60             Phone  +49.721.98520.0
**                  76185 Karlsruhe              Fax    +49.721.98520.99
**                  Germany                      WWW    http://www.ipg.de
******************************************************************************
**
** Functions
** ---------
**
** Initialization
**
**	User_Init_First ()
**	User_PrintUsage ()
**	User_ScanCmdLine ()
**
**	User_AppLogFilter ()
**
**	User_Init ()
**	User_Register ()
**	User_DeclQuants ()
**
**	User_Param_Add ()
**	User_Param_Get ()
**
**	My_EngineControl ()
**
**
** Main TestRun Start/End:
**
**	User_TestRun_Start_atBegin ()
**	User_TestRun_Start_atEnd ()
**	User_TestRun_Start_StaticCond_Calc ()
**	User_TestRun_Start_Finalize ()
**	User_TestRun_RampUp ()
**
**	User_TestRun_End_First ()
**	User_TestRun_End ()
**
**
** Main Cycle:
**
**	User_In ()
**
**	User_DrivMan_Calc ()
**	User_Traffic_Calc ()
**	User_Calc ()
**	User_Check_IsIdle ()
**
**	User_Out ()
**
**
** APO Communication:
**
**	User_ApoMsg_Eval ()
**	User_ApoMsg_Send ()
**
**	User_ShutDown ()
**	User_End ()
**	User_Cleanup ()
**
**
******************************************************************************
*/

#include <Global.h>

#if defined(WIN32) && !defined(INTIME)
#  include <windows.h>
#endif

#include <stdlib.h>
#include <string.h>
#include <math.h>

#if defined(XENO)
#  include <mio.h>
#endif

#include <CarMaker.h>

#  include <Car/Vehicle_Car.h>

#include <ADASRP.h>


#include "IOVec.h"
#if defined(WITH_IO_CAN)
#  include "IO_CAN.h"
#  include "IO_CAN_User.h"
#endif
#if defined(WITH_FLEXRAY)
#  include <flex_rbs.h>
#endif
#if defined(WITH_SOMEIP)
#  include <sip_rbs.h>
#endif

#include "User.h"


/* @@PLUGIN-BEGIN-INCLUDE@@ - Automatically generated code - don't edit! */
/* @@PLUGIN-END@@ */


/*#define SIMULINK_EXAMPLE*/

#if defined (SIMULINK_EXAMPLE)
   /* Example of a Simulink model */
#  include "SimuModel_CarMaker_rtw/SimuModel_wrap.h"
#endif



/* For debugging purposes */
float	UserOut[N_USEROUTPUT];
tUser	User;


/* APO communication channel for user/application defined messages;
   see file SimCore.h for a list of predefined channels! */
static const int UserApoCh = ApoCh_UserApp;



/*
** User_Init_First ()
**
** First, low level initialization of the User module
**
** Call:
** - one times at start of program
** - no realtime conditions
**
*/

int
User_Init_First (void)
{
    memset (&User,    0, sizeof(User));
    memset (&UserOut, 0, sizeof(UserOut));
    /* LogOutputHook = &AppLogFilter; */

    return 0;
}



/*
** User_PrintUsage ()
**
** Print the user/application specific programm arguments
*/

void
User_PrintUsage (const char *Pgm)
{
    /* REMARK: 1 log statement for each usage line, no line breaks */
    LogUsage("\n");
    LogUsage("Usage: %s [options] [testrun]\n", Pgm);
    LogUsage("Options:\n");

#if defined(CM_HIL)
    {
	if (IO_GetDefault() != NULL)
	    printf(" -io %-12s Default I/O configuration\n", IO_GetDefault());
	const tIOConfig *cf;
	for (cf=IO_GetConfigurations(); cf->Name!=NULL; cf++)
	    LogUsage(" -io %-12s %s\n", cf->Name, cf->Description);
    }
#endif

#if 0
    /*** Example: Scan command line arguments ***/
    LogUsage(" -a <x>           dummy with 1 arg\n");
    LogUsage(" -b               dummy\n");
#endif
}



/*
** User_ScanCmdLine ()
**
** Scan application specific command line arguments
**
** Return:
** - argv: last unscanned argument
** - NULL: error or unknown argument
*/

char **
User_ScanCmdLine (int argc, char **argv)
{
    const char *Pgm = argv[0];

    /* I/O configuration to be used in case no configuration was
       specified on the command line. */
    IO_SelectDefault("none" /* or "demoapp", "demoapp,demofr" etc. */);
#if defined(WITH_FLEXRAY)
    IO_SelectDefault("demofr");
#endif
#if defined(WITH_SOMEIP)
    IO_SelectDefault("demosip");
#endif

    while (*++argv) {
	if (strcmp(*argv, "-io") == 0 && argv[1] != NULL) {
	    if (IO_Select(*++argv) != 0)
		return NULL;
#if 0
	/*** Example: Scan command line arguments ***/
	} else if (strcmp(*argv, "-a") == 0 && argv[1] != NULL)	{
	    double x = atof(argv[1]);
	    ++argv;
	} else if (strcmp(*argv, "-x") == 0 && argv[1] != NULL) {
	    char *s = argv[1];
	    ++argv;
	} else if (strcmp(*argv, "-b") == 0) {
	    b = 1;
#endif
	} else if (strcmp(*argv, "-h") == 0 || strcmp(*argv, "-help") == 0) {
	    User_PrintUsage(Pgm);
	    SimCore_PrintUsage(Pgm); /* Possible exit(), depending on CM-platform! */
	    return  NULL;
	} else if ((*argv)[0] == '-') {
	    LogErrF(EC_General, "Unknown option '%s'", *argv);
	    return NULL;
	} else {
	    break;
	}
    }

    /* Depending on your application, selection of more than one I/O configuration
       may be supported or not. This is the place to check the current selection
       (e.g. find forbidden combinations) or make automatic corrections. */
#if 1
    if (IO_CountSelected() > 1) {
	LogErrStr(EC_Init, "Selection of more than one I/O configuration not allowed");
	return NULL;
    }
#else
    /* Example: Forbidden I/O combination.
    if (IO_MagicA && IO_StupidB) {
	LogErrStr(EC_Init, "MagicA and StupidB cannot be selected simultaneously");
	return NULL;
    }
    */

    /* Example: Selection of one configuration should automatically include another one.
    if (IO_MagicA)
	IO_Select("funnyc");
    */
#endif

    return argv;
}



#if 0
/*** Example: Function to filter all Log messages	***************** Begin Example	***/


/*
** User_AppLogFilter ()   ((*LogOutputHook) ())
**
** Application specific hook function to filter all input messages
** of the log module. Before the standard error handling is done,
** this function -- if defined -- is called.
** The User can decide how to react on an error:
** - suppress a message
** - do something special
**
** Info:
** - Log messages have an negative ECId
**
** Call:
** - pay attention to realtime condition
**
** return values:
**   0 : go on with default error handling
**  >0 : don't continue with default error handling
**       everything is done here
*/

static int
User_AppLogFilter (unsigned ECId, const char *Txt, tLogMsgKind MsgKind)
{
    /* filter CarMaker error classes */
    if (ECId == EC_Init) {

	if (MsgKind == LogMsgKind_Warn) {
	    /* handle warning */
	} else if (MsgKind == LogMsgKind_Err) {
	    /* handle error */
	}
	return 0;
    }
    return 0;
}


/*** Example: Function to filter all Log messages	***************** End Example	***/
#endif


/*
** User_Init ()
**
** Basic initialization of the module User.o
**
** Call:
** - once at program start
** - no realtime conditions
*/

int
User_Init (void)
{
    return 0;
}



int
User_Register (void)
{

    /* @@PLUGIN-BEGIN-REGISTER@@ - Automatically generated code - don't edit! */
    /* @@PLUGIN-END@@ */

    return 0;
}



/*
** User_DeclQuants ()
**
** Add user specific quantities to the dictionary
**
** Call:
** - once at program start
** - no realtime conditions
*/

void
User_DeclQuants (void)
{
    int i;

    /* DDefInt     (NULL, "FDigits1",   "", &FDigits1, DVA_None); */
    /* DDefDouble4 (NULL, "BrakeTime",  "s",   &User.BrakeTime, DVA_None); */

    for (i=0; i < N_USEROUTPUT; i++) {
	char sbuf[32];
	sprintf (sbuf, "UserOut_%02d", i);
	DDefFloat (NULL, sbuf, "", &UserOut[i], DVA_IO_In);
    }

#if defined (SIMULINK_EXAMPLE)
    /*** Integration of a simulink model (example) */
    SimuModel_DeclQuants(NULL);

#endif
#if defined(WITH_FLEXRAY)
    if (IO_DemoFR)
        RBS_DeclQuants();
#endif
#if defined(WITH_SOMEIP)
    if (IO_DemoSIP)
        SIP_DeclQuants();
#endif
}




/*
** User_Param_Add ()
**
** Update all modified application specific parameters in the test stand
** parameter file (ECUParameters).
**
** If the variable SimCore.TestRig.ECUParam.Modified set to 1 somewhere else
** CarMaker calls this function to let the user add or change all necessary
** entries before the file is written.
** So, if writing the ECUParam file is necessary, set ECUParam.Modified to 1.
** The next TestRun start or end, CarMaker calls this function and writes
** the file to the harddisk.
**
** Call:
** - in a separate thread (no realtime contitions)
** - when starting a new test run
*/

int
User_Param_Add (void)
{

#if defined(CM_HIL)
    /* ECU parameters */
    if (SimCore.TestRig.ECUParam.Inf == NULL)
	return -1;

    /* InfoSetLong(SimCore.TestRig.ECUParam.Inf, "IO.LWS_Id", IO.LWS_Id); */
#endif

    return 0;
}



/*
** User_Param_Get ()
**
** Update all modified application specific parameters from the test stand
** parameter file (ECUParameters).
**
** Call:
** - in a separate thread (no realtime conditions)
** - if User_Param_Get() wasn't called
** - when starting a new test run, if
**   - the files SimParameters and/or
**   - ECUParameters
**   are modified since last reading
**
** return values:
**  0	ok
** -1	no testrig parameter file
** -2	testrig parameter error
** -3	i/o configuration specific error
** -4	no simulation parameters
** -5	simulation parameters error
** -6	FailSafeTester parameter/init error
*/

int
User_Param_Get (void)
{
    int rv = 0;

#if defined(CM_HIL)
    /* Uncomment the following line, when reading/setting parameters,
       that could confuse connected hardware via the I/O operations
       in the concurrently running main thread. */
    /*
    if (!DrivMan.OperatorActive)
	DrivMan.OperatorActive = 1;
    DrivMan.OperationState_trg = OperState_Absent;
    SysUSleep(10000);
    */
#endif


#if defined(CM_HIL)
    /*** testrig / ECU parameters */
    if (SimCore.TestRig.ECUParam.Inf == NULL)
	return -1;

    if (IO_Param_Get(SimCore.TestRig.ECUParam.Inf) != 0)
	rv = -2;

    /*** parameters for different i/o hardware */
    if (IO_DemoApp) {
    }
#endif

    /*** simulation parameters */
    if (SimCore.TestRig.SimParam.Inf == NULL)
	return -4;


    return rv;
}



/*
** My_EngineControl ()
**
** Simulation model for an engine control ECU
** Modify the engine torque (PowerTrain.Engine.Trq), in most cases
** based on an static engine torque characteristic.
**
** Call:
** - called after EngineTrq model (PowerTrain.Engine.Trq is assigned
**   with the output of EngineTrq model)
** - pay attention to realtime condition
*/

#if 0		/* example */
static void
My_EngineControl (void)
{
    /* PowerTrain.Engine.Trq = fcn(PowerTrain.Engine.Trq,...); */
}
#endif		/* example */



/*
** User_TestRun_Start_atBegin ()
**
** Special things before a new simulation starts like
** - reset user variables to their default values
** - reset counters
** - ...
**
** Call:
** - in separate thread (no realtime conditions)
** - when starting a new test run
** - after (standard) infofiles are read in
** - before reading parameters for Environment, DrivMan, Car, ...
**   the models are NOT in the simulation-can-start-now state
**   (after Start(), before StaticCond())
*/

int
User_TestRun_Start_atBegin (void)
{
    int rv = 0;
    int i;

#if 0
    /* activate an engine control software module */
    Vehicle.EngineControl = My_EngineControl;
#endif


    for (i=0; i < N_USEROUTPUT; i++)
	UserOut[i] = 0.0;


    if (IO_None)
	return rv;

# if defined(CM_HIL) && !defined(INTIME) && !defined(ARTE_INTIME)
    if (FST_New(SimCore.TestRig.ECUParam.Inf) != 0)
	rv = -6;
# endif

    if (IO_DemoApp) {
	/* ... */
    }

    return rv;
}




/*
** User_TestRun_Start_atEnd ()
**
** Special things before a new simulation starts like
** - reset user variables to there default values
** - reset counters
** - ...
**
** Call:
** - in seperate thread (no realtime conditions)
** - when starting a new test run
** - at the end, behind reading parameters for Environment, DrivMan,
**   Car, ...
**   the models are NOT in the simulation-can-start-now state
**   (after Start(), before StaticCond())
*/

int
User_TestRun_Start_atEnd (void)
{
    int rv = 0;
    /*tInfos *inf = SimCore.Vhcl.Inf;*/

#if defined (SIMULINK_EXAMPLE)
    /*** Integration of a simulink model (example) */
    if (SimuModel_Inst != NULL) {
        /* Cleanup model instance of the previous testrun. */
        SimuModel_Delete(SimuModel_Inst);
	SimuModel_Inst = NULL;
    }
    SimuModel_Inst = SimuModel_New(SimCore.Vhcl.Inf);
#endif

    return rv;
}



/*
** User_TestRun_Start_StaticCond_Calc ()
**
** called in non RT context
*/

int
User_TestRun_Start_StaticCond_Calc (void)
{
    return 0;
}



/*
** User_TestRun_Start_Finalize ()
**
** called in RT context
*/

int
User_TestRun_Start_Finalize (void)
{
    return 0;
}



/*
** User_TestRun_RampUp ()
**
** Perform a smooth transitition of variables (e.g. I/O)
** from their current state  to the new testrun.
** This function is called repeatedly, once during each cycle, until
** it returns true (or issues an error message), so the function should
** return true if transitioning is done, false otherwise.
**
** In case of an error the function should issue an apropriate
** error message and return false;
**
** Called in RT context, in state SCState_StartSim,
** after preprocessing is done, before starting the engine.
** Please note, that in this early initialization state no calculation
** of the vehicle model takes place.
*/

int
User_TestRun_RampUp (double dt)
{
    int IsReady = 1;

    return IsReady;
}



/*
** User_TestRun_End_First ()
**
** Invoked immediately after the end of a simulation is initiated,
** but before data storage ends and before transitioning into SCState_Idle.
** - Send Scratchpad-note
** - ...
**
** Call:
** - in main task, in the main loop (real-time conditions!)
** - when a test run is finished (SimCore.State is SCState_End)
*/

int
User_TestRun_End_First (void)
{
    return 0;
}



/*
** User_TestRun_End ()
**
** Special things after the end of a simulation like
** - switch off an air compressor
** - Write something to a file
** - ...
**
** Call:
** - in seperate thread (no realtime conditions)
** - when a test run is finished (SimCore.State is SCState_End<xyz>)
*/

int
User_TestRun_End (void)
{
    if (IO_DemoApp) {
    }

    return 0;
}



/*
** User_In ()
**
** Assign quantities of the i/o vector to model variables
**
** Call:
** - in the main loop
** - pay attention to realtime condition
** - just after IO_In()
*/

void
User_In (const unsigned CycleNo)
{
    /*** Handle Write Access (first action after IO_In() */

    /* User.xxx = IO.yxz; */

#if 0
    /*** Brake System */
    /* Use together with built-in hydraulic controller model HydHIL or a user
       model; be sure not to overwrite any of the variables you get from I/O. */ 
    Brake.HydBrakeCU_IF.V[0] = IO.HydValve[0];
    Brake.HydBrakeCU_IF.V[1] = IO.HydValve[1];
    Brake.HydBrakeCU_IF.V[2] = IO.HydValve[2];
    Brake.HydBrakeCU_IF.V[3] = IO.HydValve[3];

    Brake.HydBrakeCU_IF.V[4] = IO.HydValve[4];
    Brake.HydBrakeCU_IF.V[5] = IO.HydValve[5];
    Brake.HydBrakeCU_IF.V[6] = IO.HydValve[6];
    Brake.HydBrakeCU_IF.V[7] = IO.HydValve[7];

    Brake.HydBrakeCU_IF.V[8]  = IO.HydValve[8];
    Brake.HydBrakeCU_IF.V[9]  = IO.HydValve[9];
    Brake.HydBrakeCU_IF.V[10] = IO.HydValve[10];
    Brake.HydBrakeCU_IF.V[11] = IO.HydValve[11];

    Brake.HydBrakeCU_IF.PumpCtrl  = IO.PumpIsOn;
    Brake.HydBrakeCU_IF.BooSignal = IO.BooSignal;
#endif

#if defined(WITH_IO_CAN)
    IO_CAN_User_In(CycleNo);
#endif

    if (SimCore.State != SCState_Simulate)
	return;

#if 0
    unsigned int   msgID;
    unsigned char  data[8];
    unsigned short dlc;


    while (ADASRP_RecvCANMsg(&msgID, data, &dlc) == 1) {
	/* Do something with msgID, data, dlc */
    }
#endif

#if defined (SIMULINK_EXAMPLE)
#if 0
    /*** Integration of a simulink model (example)
     * Call the calc function here, if model calculations
     * should take place BEFORE all other models.
     */
    if (SimuModel_Inst != NULL)
	SimuModel_Calc(SimuModel_Inst);
#endif
#endif
}



/*
** User_DrivMan_Calc ()
**
** called
** - in RT context
** - after DrivMan_Calc()
*/

int
User_DrivMan_Calc (double dt)
{
    /* Rely on the Vehicle Operator within DrivMan module to get
       the vehicle in driving state using the IPG's
       PowerTrain Control model 'Generic' or similar */
    if (Vehicle.OperationState != OperState_Driving)
	return 0;

    return 0;
}



/*
** User_Traffic_Calc ()
**
** called
** - in RT context
** - after Traffic_Calc()
*/

int
User_Traffic_Calc (double dt)
{
    if (SimCore.State != SCState_Simulate)
	return 0;

    return 0;
}



/*
** User_Calc ()
**
** called in RT context
*/

int
User_Calc (double dt)
{
    /* Uncomment the following line in order to exlude User_Calc() from
       pre- and postprocessing (default behaviour in CM 4.0 and earlier). */
    /*if (SimCore.State != SCState_Simulate) return 0;*/

#if defined (SIMULINK_EXAMPLE)
    /*** Integration of a simulink model (example) */
    if (SimuModel_Inst != NULL)
        SimuModel_Calc(SimuModel_Inst);

#endif
    return 0;
}



/*
** User_Check_IsIdle ()
**
** Checking, if the simulation model is in idle conditions (stand still,
** steeringwheel angle zero, cluch pedal pressed, ...).
** If reached idle state, the calculation of vehicle model and driving
** manoevers is stopped.
** Ready for start new simulation.
**
** Return:
** 1  idle state reached
** 0  else
**
** Call:
** - in main task, in the main loop
** - pay attention to realtime condition
** - while SimCore.State==SCState_EndIdleGet
*/

int
User_Check_IsIdle (int IsIdle)
{
    double val;

    /*** ECU / carmodel signals */

    /* vehicle and wheels: stand still */
    val = 0.5*kmh2ms;
    if (Vehicle.v > val
     || fabs(Vehicle.Wheel[0]->vBelt) > val || fabs(Vehicle.Wheel[1]->vBelt) > val
     || fabs(Vehicle.Wheel[2]->vBelt) > val || fabs(Vehicle.Wheel[3]->vBelt) > val) {
	IsIdle = 0;
    }

    /* SteerAngle: drive  straight forward position */
    val = 1.0*deg2rad;
    if (Vehicle.Steering.Ang > val || Vehicle.Steering.Ang < -val)
	IsIdle = 0;

    return IsIdle;
}



/*
** User_Out ()
**
** Assigns model quantities to variables of the i/o vector
**
** call:
** - in the main loop
** - pay attention to realtime condition
** - just before IO_Out();
*/

void
User_Out (const unsigned CycleNo)
{

#if 0
    /* IO.Kl15SW = Vehicle.Ignition; */

    /* With a defined body sensor named 'ESP' the body sensor index
       should be determined in function User_TestRun_Start_atEnd() with
            BS_ESP = BodySensor_FindIndexForName("ESP");
       where BS_ESP should be defined as a global variable like
            int BS_ESP = -1;
    */
    if (BS_ESP >= 0) {
	IO.ax         = BodySensor[BS_ESP].Acc_B[0];
	IO.ay         = BodySensor[BS_ESP].Acc_B[1];
	IO.YawRate    = BodySensor[BS_ESP].Omega_B[2];
    } else {
	IO.ax         = 0.0;
	IO.ay         = 0.0;
	IO.YawRate    = 0.0;
    }
    IO.StWhlAngle = VehicleControl.Steering.Ang;
    IO.Gas        = VehicleControl.Gas;
    IO.BLS        = VehicleControl.Brake > 0.0;

    IO.pMC    = Brake.HydBrakeIF.pMC;
    IO.pWB[0] = Brake.HydBrakeIF.pWB[0];
    IO.pWB[1] = Brake.HydBrakeIF.pWB[1];
    IO.pWB[2] = Brake.HydBrakeIF.pWB[2];
    IO.pWB[3] = Brake.HydBrakeIF.pWB[3];

    IO.PuRetVolt = Brake.HydBrakeIF.PuRetVolt;
    IO.Rel_SW    = Brake.HydBrakeIF.Rel_SW;

    IO.WheelSpd[0] = Car.Tire[0].WheelSpd;
    IO.WheelSpd[1] = Car.Tire[1].WheelSpd;
    IO.WheelSpd[2] = Car.Tire[2].WheelSpd;
    IO.WheelSpd[3] = Car.Tire[3].WheelSpd;
#endif

#if defined(WITH_IO_CAN)
    IO_CAN_User_Out(CycleNo);
#endif

#if defined(WITH_FLEXRAY)
    if (IO_DemoFR)
        RBS_OutMap(CycleNo);
#endif
#if defined(WITH_SOMEIP)
    if (IO_DemoSIP)
        SIP_OutMap(CycleNo);
#endif

    if (SimCore.State != SCState_Simulate)
	return;

#if 0
    {
	unsigned int   msgID   = 0x02;
	unsigned char  data[8] = {0x11,0x22,0x33,0x44,0x12,0x22,0x34,0x45};
	unsigned short dlc     = 8;
   	ADASRP_SendCANMsg (msgID,data,dlc);
    }
#endif

#if defined (SIMULINK_EXAMPLE)
#if 0
    /*** Integration of a simulink model (example)
     * Call the calc function here, if model calculations
     * should take place AFTER all other models.
     */
    if (SimuModel_Inst != NULL)
        SimuModel_Calc(SimuModel_Inst);
#endif
#endif
}



/*
** User_ApoMsg_Eval ()
**
** Communication between the application and connected GUIs.
** Evaluate messages from GUIs
**
** Call:
** - in the main loop
** - pay attention to realtime condition
** - near the end of the main loop, if the function SimCore_ApoMsg_Eval()
**    skips the message
**
** Return:
**   0 : message evaluated
**  -1 : message not handled
*/

int
User_ApoMsg_Eval (int Ch, char *Msg, int len, int who)
{
#if defined(CM_HIL) && !defined(INTIME) && !defined(ARTE_INTIME)
    /*** FailSafeTester */
    if (Ch == ApoCh_CarMaker) {
	if (FST_ApoMsgEval(Ch, Msg, len) <= 0)
	    return 0;
    }
#endif
    if (Ch == ApoCh_Diagnostics) {
#if 0 		/* example for Diagnostics */
	if (Diag_ApoMsg_Eval(Ch, Msg, len) >= 0)
	    return 0;
#endif	 	/* example for Diagnostics */

	/* suppress further warnings about unknown messages */
	return 1;
    }

    if (Ch == UserApoCh) {
#if 0 		/* example */
	if (len >= 4 && strncmp(Msg,"foo",3)==0) {
	    /* do something */
	    return 0;
	}
#endif	 	/* example */
    }

    return -1;
}



/*
** User_ApoSend ()
**
** Communication between the application and connected GUIs.
** Sends messages to GUIs
**
** Call:
** - in the main loop
** - pay attention to realtime condition
** - near the end of the main loop, after Hil_ApoSend()
*/

void
User_ApoMsg_Send (double T, const unsigned CycleNo)
{
#if 0 	/* example */
    if (CycleNo%100 == 37) {
	static struct {
	    float T;
	} msg;
	msg.T = 	  IO.T;
	AposSendAppMsg (UserApoCh, &msg, sizeof(msg));
    }
#endif 	/* example */
}



/*
** User_ShutDown ()
**
** Prepare application for shut down
**
** Call:
** - at end of program
** - no realtime conditions
*/

int
User_ShutDown (int ShutDownForced)
{
    int IsDown = 0;

    /* Prepare application for shutdown and return that
       shutdown conditions are reached */
    if (1) {
	IsDown = 1;
    }

    return IsDown;
}



/*
** User_End ()
**
** End all models of the user module
**
** Call:
** - one times at end of program
** - no realtime conditions
*/

int
User_End (void)
{
#if defined (SIMULINK_EXAMPLE)
    /*** Integration of a simulink model (example) */
    if (SimuModel_Inst != NULL) {
        /* Cleanup model instance of the previous testrun. */
        SimuModel_Delete(SimuModel_Inst);
        SimuModel_Inst = NULL;
    }
#endif

    return 0;
}



/*
** User_Cleanup ()
**
** Cleanup function of the User module
**
** Call:
** - one times at end of program, just befor exit
** - no realtime conditions
*/

void
User_Cleanup (void)
{
}

