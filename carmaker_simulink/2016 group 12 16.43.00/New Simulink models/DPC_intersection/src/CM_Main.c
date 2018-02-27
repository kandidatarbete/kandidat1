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
**  Functions
**  ---------
**
**	v	RefTime
**	v	OnlyOneSimulation
**
**	v	CycleNoTimeWCNext
**	v	CycleNoTimeWC
**
**	v	dtSync
**	v	TimeWCAbs
**	v	TimeWCRef
**
**
**		WaitForNextLoop ()
**		ExitFcn ()
**		App_Init_First ()
**		App_Init_Second ()
**		App_Init ()
**		App_Register ()
**		App_DeclQuants ()
**		App_TestRun_Start_StaticCond_Calc ()
**		App_TestRun_Start ()
**		App_TestRun_Start_Finalize ()
**		App_TestRun_Snapshot_Take ()
**		App_TestRun_Calc ()
**		App_TestRun_Calc_Part ()
**		App_TestRun_End ()
**		App_ShutDown ()
**		App_End ()
**		App_Cleanup ()
**
**		MainThread_Init ()
**		MainThread_BeginCycle ()
**		MainThread_DoCycle ()
**		MainThread_FinishCycle ()
**
**		MainThread_End ()
**
**	main ()
**		CM_Main_Begin ()			(CM4SL)
**		CM_Main_End ()				(CM4SL)
**
**
** Special options for debugging
** - DeltaT = const = SimCore.DeltaT
** - Initialisation not in a second Thread
**
******************************************************************************
*/

/******************************************************************************
** +++ ATTENTION +++ ATTENTION +++ ATTENTION +++ ATTENTION +++ ATTENTION +++
**
**     Please, don't modify this file!
**
** +++ ATTENTION +++ ATTENTION +++ ATTENTION +++ ATTENTION +++ ATTENTION +++
*******************************************************************************
*/

#include <Global.h>

#if defined(WIN32) && !defined(INTIME)
#  include <windows.h>
#  include <process.h>
#endif

#include <stdlib.h>
#include <string.h>
#include <errno.h>

#if defined(XENO)
#  include <unistd.h>
#  include <native/task.h>
#  include <native/timer.h>
#endif
#if defined(XENO)
#  include <RTGuard.h>
#endif

#include <CarMaker.h>
#include <ModelManager.h>
#  include <Vehicle/BodySensor.h>
#  include <Vehicle/DASensor.h>
#  include <Vehicle/FSSensor.h>
#  include <Vehicle/RPSensor.h>
#  include <Vehicle/TSSensor.h>
#  include <Vehicle/LineSensor.h>
#  include <Vehicle/PylonDetect.h>
#  include <Vehicle/CollisionDetect.h>
#  include <Vehicle/GNSSModule.h>
#  include <Car/Brake.h>
#  include <Car/Trailer_Brake.h>

#include "ADASRP.h"

#include "User.h"
#include "IOVec.h"

#if defined(WITH_XCP)
# include <CM_XCP.h>
#endif
#if defined(WITH_CCP)
# include <CM_CCP.h>
#endif

#if defined(LABCAR)
# include <LABCAR.h>
#endif

#if !defined(CM_HIL) || defined(INTIME) || defined(ARTE_INTIME)
/* Replacement functions and variables if module 'IO.c' is not present,
   i.e. when compiling for a non-realtime environment.  */
tIOVec	IO;

int  IO_Init_First    (void)        { return 0; }
int  IO_Init          (void)        { return 0; }
int  IO_Init_Finalize (void)        { return 0; }
int  IO_Param_Get     (tInfos *inf) { return 0; }

void IO_In      (unsigned CycleNo) { }
void IO_Out     (unsigned CycleNo) { }
void IO_Cleanup (void)             { }
#endif /* !CM_HIL || INTIME || ARTE_INTIME */


#if defined(XENO) || defined(INTIME) || defined(ARTE_INTIME) || defined(DSPACE) || defined(LVRT)
static double	RefTime;
#endif

static int OnlyOneSimulation = 0;




/*** Time Synchronisation	***********************************************/

#if !defined(CM_HIL)
static unsigned long	CycleNoTimeWCNext;
static unsigned long	CycleNoTimeWC;

static double	dtSync;
static double	TimeWCAbs;
static double	TimeWCRef;
#endif



#if defined(WIN32) || defined(ARTE_INTIME)
static __inline int
#else
static inline int
#endif
InSyncWithVDS (void)
{
    return SimCore.Anim.Sync_State!=SyncWaiting || SimCore.State!=SCState_Simulate;
}


/*
** WaitForNextLoop()
**
** Returns DeltaT
*/

static double
WaitForNextLoop (unsigned CycleNo)
{
    double DeltaT;

#if defined(CM_HIL)
# if defined(XENO)
    /*** Real-Time ****************************************************/

    const  int maxSlowCycles = 10;
    static int nSlowCycles = 0;
    static int budget = 1000;
    int GaveUpCPU = 0, waitres;

#  if defined(LABCAR)
    waitres = LCO_Wait();
#  else
    unsigned long nOverruns;
    waitres = rt_task_wait_period(&nOverruns);
#  endif
    if (waitres == 0) {
	nSlowCycles = 0;
    } else {
	if (++nSlowCycles > maxSlowCycles) {
	    /* Application is too slow.
	       Reduce priority to give up CPU time to other processes. */
	    usleep(1000);
	    GaveUpCPU = 1;
	    nSlowCycles = 0;
	}
    }

    if (SimCore.ConstDeltaT) {
	DeltaT = SimCore.DeltaT;
    } else {
	double PrevRefTime = RefTime;
	RefTime = SysGetTime();
	DeltaT = RefTime - PrevRefTime;
    }
    SimCore.TCPU.Cycle = DeltaT;
# endif /* defined(XENO) */

# if defined(XENO) || defined(INTIME) || defined(ARTE_INTIME) || defined(LVRT)
    /*** Check real time conditions */
    if (budget < 1000)
	++budget;

    if ((GaveUpCPU || DeltaT > SimCore.Cycle.tCTViolationWarn) && CycleNo > 2)
    {
	SimCore.Cycle.nCTViolation++;
	if (budget > 0) {
	    char sbuf[128];

	    if (GaveUpCPU) {
		Log("Cycle time overrun %d times in a row\n", maxSlowCycles);
		Log("Gave up CPU to prevent system lockup (CycleNo=%u)\n", CycleNo);
	    }
	    sprintf(sbuf, "Cycle too long dt=%.2fms (State=%s CycleNo=%u n=%d)",
		    DeltaT*1e3, SimCore_State_Str(SimCore.State), CycleNo,
		    SimCore.Cycle.nCTViolation);
	    if (DeltaT > SimCore.Cycle.tCTViolationErr) {
		LogErrStr (EC_CycleTime, sbuf);
	    } else {
		LogWarnStr(EC_CycleTime, sbuf);
	    }
	    SimCore_TCPU_Log();

	    budget -= 100;
	}

	if (SimCore.Cycle.nCTViolation > SimCore.Cycle.maxCTViolation) {
	    LogErrF(EC_General, "Realtime conditions violated more than %d times!",
		    SimCore.Cycle.maxCTViolation);
	    SimCore_MainThreadEmergency();
	}
    }
    if (!SimCore.Cycle.MeasuredDT)
	DeltaT = SimCore.DeltaT;
    TimeGlobal += DeltaT;
    SimCore.TimeWC = TimeGlobal;
# endif /* defined(XENO) || defined(INTIME) || defined(ARTE_INTIME) || defined(LVRT) */
#else
    /*** Non Real-Time	**********************************************/

    static unsigned nCyclesSkipped = 0;
    double TimeWC;

    DeltaT = SimCore.DeltaT;	/* := const */
    SimCore.TCPU.Cycle = (float)DeltaT;
    TimeWC = SysGetTime();
    if (SimCore.State>=SCState_StartSim && SimCore.State<=SCState_EndIdleSet) {
	if (SimCore.State == SCState_StartSim) {
	    dtSync += DeltaT / 500;		/* max speed while ramping up */
	} else {
	    dtSync += DeltaT / SimCore.TAccel;	/* simulation speed */
	}
    } else {
	dtSync += DeltaT / SimCore.TAccelIdle;	/* idle speed */
    }
    dtSync -= TimeWC-TimeWCAbs;

    TimeWCAbs      = TimeWC;
    SimCore.TimeWC = TimeWCAbs - TimeWCRef;

    if (dtSync > CycleTimer.dtFaster) {
	/* simulation too fast -> faster than allowed -> wait */
	if (dtSync > CycleTimer.dtSyncMax)
	    dtSync = CycleTimer.dtSyncMax;
	SysUSleep ((long)(dtSync*1000000));

    } else if (dtSync < CycleTimer.dtSlower) {
	/* simulation too slow -> much too slow -> resync reference */
	dtSync = CycleTimer.dtSlower;
    }

    if (!InSyncWithVDS()) {
	++nCyclesSkipped;
	return DeltaT;
    }
    CycleNo -= nCyclesSkipped;

    TimeGlobal += DeltaT;
#endif /* !defined(CM_HIL) */

    if (SimCore.State == SCState_Simulate) {
	SimCore.Time += SimCore.DeltaT;
	SimCore.AposPollTime =	(int)(SimCore.Time*1e3+0.5)*1e-3;
    } else {
	SimCore.AposPollTime =	(int)((SimCore.AposPollTime+SimCore.DeltaT)*1e3+0.5)*1e-3;
    }

    SimCore.CycleNo    = CycleNo;
    SimCore.MustTakeTS = CycleNo % SimCore.TSInterval == 0;

    return DeltaT;
}


static void
ProcessApoMessages (void)
{
    int ch, len, who;
    char MsgBuf[APO_ADMMAX];

    while (len=sizeof(MsgBuf), AposGetAppMsgFrom(&ch,&MsgBuf,&len,&who) != 0) {
#if defined(WITH_XCP)
	if (CM_XCP_ApoMsg_Eval(ch, MsgBuf, len) >= 0)
	    continue;
#endif
#if defined(WITH_CCP)
	if (CM_CCP_ApoMsg_Eval(ch, MsgBuf, len) >= 0)
	    continue;
#endif
	if (User_ApoMsg_Eval(ch, MsgBuf, len, who) < 0
	 && SimCore_ApoMsg_Eval(ch, MsgBuf, len, who) < 0) {
	    SimCore_UnknownApoMsgWarn(ch, MsgBuf, len);
	}
    }
}


/*
** ExitFcn ()
**
** Fast exit in special situations (signal handler, emergency loop).
*/

static int
ExitFcn (void)
{
    IO_Cleanup();
    AposExit();
    SysSSleep(0.5);
    LogClose ();
    LogCleanup ();
    SysCleanup ();
    return 0;
}


/*** ***	***************************************************************/




/*
** App_Init_First ()
**
*/

static int
App_Init_First (int argc, char **argv)
{
    const char *appname = AppStartInfo.App_Version;

    SysInit ();
    SysNonStdMath ();


    CMTh_Init ();
    LogInitSingleThread ("@LogBuffer@");

    SimCore_Init_First (argc, argv);
    SimCore_Apos_Init_First (10, 0, 0xffffffff);
    AposSetInfo ("CarMaker", appname, "Application is starting ...");

    IO_Init_First ();
    User_Init_First ();
#if defined(WITH_XCP)
    CM_XCP_Init_First(NULL);
#endif
#if defined(WITH_CCP)
    CM_CCP_Init_First(NULL);
#endif
    return 0;
}



/*
** App_Init_Second ()
**
*/

static int
App_Init_Second (void)
{
    if (SimCore_Init()< 0)
	return -1;
    return 0;
}



/*
** App_Init ()
**
*/

static int
App_Init (void)
{
    ExtInp_Init ();
    Env_Init ();
    TrfLight_Init ();
    DrivMan_Init ();
    VehicleControl_Init ();
    Vhcl_Init ();
    Traffic_Init ();
    PylonDetect_Init ();
    BdyFrame_Init ();
    BodySensor_Init ();
    DASensor_Init ();
    FSSensor_Init ();
    RPSensor_Init ();
    TSSensor_Init ();
    LineSensor_Init ();
    CollisionDetect_Init ();
    GNSSModule_Init ();

#if defined(CM_HIL) && !defined(INTIME) && !defined(ARTE_INTIME)
    FST_Init(SimCore.TestRig.ECUParam.Inf);
#endif
    if (IO_Init() < 0 || User_Init() < 0)
	return -1;
#if defined(WITH_XCP)
    if (CM_XCP_Init() != 0)
	return -1;
#endif
#if defined(WITH_CCP)
    if (CM_CCP_Init() != 0)
	return -1;
#endif
#if defined(LABCAR)
    LCO_Init();
#endif

    Plugins_Init ();

    return 0;
}



/*
** App_Register ()
**
*/

static int
App_Register (void)
{
    Vhcl_Register ();
    User_Register ();
    return 0;
}



/*
** App_DeclQuants ()
**
** Add quantities to the data dictionary (can displayed, saved)
** and export configuration
*/

static int
App_DeclQuants (void)
{
    SimCore_DeclQuants ();
    Env_DeclQuants ();
    TrfLight_DeclQuants ();
    DrivMan_DeclQuants ();
    VehicleControl_DeclQuants ();
    Vhcl_DeclQuants ();
    User_DeclQuants ();
#if defined(WITH_XCP)
    CM_XCP_DeclQuants();
#endif
#if defined(WITH_CCP)
    CM_CCP_DeclQuants();
#endif
#if defined(LABCAR)
    LCO_DeclQuants();
#endif
    App_ExportConfig ();
    return 0;
}



/*
** App_TestRun_Start_StaticCond_Calc ()
**
*/

static int
App_TestRun_Start_StaticCond_Calc (void)
{
    int rv = 0;

    if (SimCore_TestRun_Start_StaticCond_Calc() < 0)
	rv = -1;

    if (Vhcl_StaticCond_Calc() < 0)
	rv = -2;

    if (User_TestRun_Start_StaticCond_Calc() < 0)
	rv = -3;

    return rv;
}



/*
** App_TestRun_Start ()
**
*/

static void *
App_TestRun_Start (void *arg)
{
    int rv = 0;
    int nError = Log_nError;

    if (!SimCore.Reconfig.Active) {
	if (SimCore_TestRun_Start() < 0) {
	    rv = -1;
	    goto ErrorReturn;
	}
    } else {
	/* Save vehicle state for reconfiguration */
	if (Vehicle_Reconfig () < 0) {
	    rv = -26;
	    goto ErrorReturn;
	}

	/* Delete Animation Message buffer */
	SimCore_Anim_SendCmd (AnmCmd_SuppressAnim, 0);
	SimCore_Anim_InvalidateMsgs();
	SimCore_Instr_InvalidateMsgs();

	/* Delete all parts which are reconfigurated */
	Vhcl_Delete (0);
	VehicleControl_Delete ();
	PylonDetect_Delete ();

	/* Restart initialization of vehicle */
	if (SimCore_Vehicle_Start() < 0) {
	    rv = -1;
	    goto ErrorReturn;
	}
    }
    if (SimCore.TestRig.ECUParam.WasRead) {
#if defined(WITH_XCP)
	if (CM_XCP_Param_Get(SimCore.TestRig.ECUParam.Inf, "XCP") != 0)
	    rv = -6;
#endif
#if defined(WITH_CCP)
	if (CM_CCP_Param_Get(SimCore.TestRig.ECUParam.Inf, "CCP") != 0)
	    rv = -7;
#endif
    }

    /* Delete all body frames for body sensor calculation before new registration */
    BdyFrame_Delete ();

    if (User_TestRun_Start_atBegin() < 0) {
	rv = -2;
	goto ErrorReturn;
    }

    if (!SimCore.Reconfig.Active && ADASRP_StartClient() != 0) {
	rv = -3;
	goto ErrorReturn;
    }

    if (!SimCore.Reconfig.Active && ExtInp_File_New() < 0) {
	rv = -4;
	goto ErrorReturn;
    }
    if (Env_New() < 0) {
	rv = -5;
	goto ErrorReturn;
    }
    if (TrfLight_New() < 0) {
	rv = -6;
	goto ErrorReturn;
    }

    if (!SimCore.Reconfig.Active && DrivMan_New_PreVhcl() < 0) {
	rv = -7;
	goto ErrorReturn;
    }

    /* Vhcl_New() has to be called before DrivMan_New()
       (Vehicle.Cfg.UseIt, ...) */
    if (Vhcl_New() < 0) {
	rv = -8;
	goto ErrorReturn;
    }

    if (Traffic_New(SimCore.TestRun.Inf, Env.Road) < 0) {
	rv = -9;
	goto ErrorReturn;
    }

    if (BodySensor_New() < 0) {
	rv = -10;
	goto ErrorReturn;
    }
    if (DASensor_New() < 0) {
	rv = -11;
	goto ErrorReturn;
    }
    if (FSSensor_New() < 0) {
	rv = -12;
	goto ErrorReturn;
    }
    if (RPSensor_New(SimCore.Vhcl.Inf) < 0) {
	rv = -13;
	goto ErrorReturn;
    }
    if (TSSensor_New(SimCore.Vhcl.Inf) < 0) {
	rv = -14;
	goto ErrorReturn;
    }
    if (LineSensor_New(SimCore.Vhcl.Inf) < 0) {
	rv = -15;
	goto ErrorReturn;
    }
    if (CollisionDetect_New (SimCore.Vhcl.Inf) < 0) {
	rv = -16;
	goto ErrorReturn;
    }

    if (GNSSModule_New(SimCore.Vhcl.Inf) < 0) {
	rv = -27;
	goto ErrorReturn;
    }
    if (PylonDetect_New (SimCore.Vhcl.Inf) < 0) {
	rv = -17;
	goto ErrorReturn;
    }

    if (!SimCore.Reconfig.Active && DrivMan_New() < 0) {
	rv = -18;
	goto ErrorReturn;
    } else if (SimCore.Reconfig.Active && DrivMan_Reconfig() < 0) {
	rv = -19;
	goto ErrorReturn;
    }
    if (VehicleControl_New(SimCore.TestRun.Inf) < 0) {
	rv = -20;
	goto ErrorReturn;
    }

    SimCore_TestRun_Start_UpdateQuants ();

    if (User_TestRun_Start_atEnd() < 0) {
	rv = -21;
	goto ErrorReturn;
    }
#if defined(WITH_XCP)
    if (CM_XCP_TestRun_Start(NULL, NULL) != 0) {
	rv = -22;
	goto ErrorReturn;
    }
#endif
#if defined(WITH_CCP)
    if (CM_CCP_TestRun_Start(NULL, NULL) != 0) {
	rv = -23;
	goto ErrorReturn;
    }
#endif

#if defined(LABCAR)
    if (!SimCore.Reconfig.Active && LCO_TestRun_Init() < 0) {
	rv = -24;
	goto ErrorReturn;
    }
#endif

    if (Plugins_New() < 0) {
	rv = -28;
	goto ErrorReturn;
    }

    if (SimCore.NextState >= SCState_End)
	goto OkReturn; /* initialization has been terminated ahead of time */

    /*** model check pre */
    if (AppStartInfo.ModelCheck & AppStart_ModelCheck_BeforePre) {
	Vhcl_ModelCheck_BeforePre(SimCore.ModelCheck.Inf);
	if ((AppStartInfo.ModelCheck &= ~AppStart_ModelCheck_BeforePre) == 0) {
	    SimStop();
	    goto OkReturn;
	}
    }

    if (SimCore.NextState >= SCState_End)
	goto OkReturn; /* initialization has been terminated ahead of time */

    /*** static conditions */
    if (App_TestRun_Start_StaticCond_Calc() != 0) {
	rv = -25;
	goto ErrorReturn;
    }

    if (SimCore.NextState >= SCState_End)
	goto OkReturn; /* initialization has been terminated ahead of time */

    /*** model check 2nd */
    if (AppStartInfo.ModelCheck & AppStart_ModelCheck_AfterPre) {
	Vhcl_ModelCheck_AfterPre(SimCore.ModelCheck.Inf);
	if ((AppStartInfo.ModelCheck &= ~AppStart_ModelCheck_AfterPre) == 0)
	    SimStop();
    }


 OkReturn:
    if (SimCore_TestRun_Start_End() < 0)
	goto ErrorReturn_2nd;
    SimCore.Start.Ok = 1;
    goto DoReturn;


 ErrorReturn:
    SimCore_TestRun_Start_End ();

 ErrorReturn_2nd:
    /* gone to this target if SimCore_TestRun_Start_End() failed */
    if (nError == Log_nError) {
	/* error was detected, but not reported */
	LogErrF (EC_Init, "Failure occured while starting %s (rv=%d)",
		 AppStartInfo.ModelCheck ? "modelcheck" : "testrun", rv);
    }
    SimCore.Reconfig.Active = 0;
    SimCore.Start.Ok = 0;


 DoReturn:
    if (SimCore.Start.Ok) {
	if (SimCore.NextState == SCState_Start        /* running in main loop    */
	 || SimCore.NextState == SCState_StartWait) { /* running in start thread */
	    SimCore.Anim.Wait_UntilWC = SimCore.TimeWC + SimCore.Anim.Wait_Timeout;
	    SimCore.Anim.Wait_Timeout = SimCore.Anim.Wait_Timeout_Default;
	    SimCore_State_Set(SCState_StartWaitAnim);
	}
    } else {
	SimCore_State_Set(SCState_End);
    }
    SimCore.Start.Tid = 0;

    return NULL;
}


/*
** App_TestRun_Start_Finalize ()
**
** called:
** - last cycle of TestRun start phase
** - RT context
*/

static int
App_TestRun_Start_Finalize (void)
{
    int rv = 0;

    if (User_TestRun_Start_Finalize() < 0)		  /* optional */
	rv = -1;

    if (SimCore_TestRun_Start_Finalize() < 0)
	rv = -2;

    return rv;
}



/*
** App_TestRun_Snapshot_Take ()
*/

static int
App_TestRun_Snapshot_Take (void)
{
    return Vhcl_Snapshot_Take();
}



/*
** App_TestRun_Calc ()
**
** Return Values:
**   0	ok
**  >0	DrivMan wants to end simulation
**  <0	error
**
** called:
** - each cycle
** - RT context
*/



static int
App_TestRun_Calc (double dt)
{
    int i;
    int rv = 0;


    if (Env_Calc(dt) < 0)
        rv = -1;

    if (TrfLight_Calc() < 0)
	rv = -1;

    if ((i=DrivMan_Calc(dt)) != 0) {
	if      (i < 0)		rv = -2;	/* := error	*/
	else if (i == 1)	rv =  1;	/* := end	*/
	else if (i == 2)	rv =  3;	/* := is idle	-> User_Check_IsIdle() */
	else /* if (i > 0) */	rv =  1;	/* := end	*/
    }
    if (User_DrivMan_Calc(dt) < 0)
	rv = -2;
    SimCore_TCPU_TakeTS(&SimCore.TS.DrivMan);

    if (SimCore.State == SCState_Simulate) {
	if (Traffic_Calc(dt) < 0)
	    rv = -5;
    }
    if (User_Traffic_Calc(dt) < 0)
	rv = -5;

#if defined(LABCAR)
    LCO_Write(DVA_DM);
#endif
    Plugins_CalcBefore (DVA_DM, dt);
    DVA_HandleWriteAccess(DVA_DM);
    Plugins_CalcAfter (DVA_DM, dt);
    SimCore_TCPU_TakeTS(&SimCore.TS.Traffic);

    if (VehicleControl_Calc() < 0)
	rv = -6;

#if defined(LABCAR)
    LCO_Write(DVA_VC);
#endif
    Plugins_CalcBefore (DVA_VC, dt);
    DVA_HandleWriteAccess(DVA_VC);
    Plugins_CalcAfter (DVA_VC, dt);
    if (VehicleControl_CalcPost() < 0)
	rv = -6;
    SimCore_TCPU_TakeTS(&SimCore.TS.VehicleControl);

    Brake.IF.Pedal = VehicleControl.Brake;
    Brake.IF.Park  = VehicleControl.BrakePark;
    if (Vehicle_nTrailer > 0) {
	TrBrake.IF.Pedal  = VehicleControl.Brake;
	TrBrake.IF.Park   = VehicleControl.BrakePark;
	TrBrake2.IF.Pedal = VehicleControl.Brake;
	TrBrake2.IF.Park  = VehicleControl.BrakePark;
    }

    if (Vhcl_Calc(dt) < 0)
        rv = -3;

    if (BdyFrame_Calc() < 0)
	rv = -12;

    if (BodySensor_Calc(dt) < 0)
        rv = -7;

    if (DASensor_Calc(dt) < 0)
        rv = -8;
    if (FSSensor_Calc(dt) < 0)
        rv = -9;
    if (RPSensor_Calc(dt) < 0)
        rv = -10;
    if (TSSensor_Calc(dt) < 0)
        rv = -11;
    if (LineSensor_Calc(dt) < 0)
        rv = -14;
    if (CollisionDetect_Calc(dt) < 0)
	rv = -15;
    if (GNSSModule_Calc(dt) < 0)
        rv = -16;
    if (PylonDetect_Calc(dt) < 0)
	rv = -13;

    SimCore_TCPU_TakeTS(&SimCore.TS.Sensors);

    if (User_Calc(dt) < 0)                      /* optional */
        rv = -4;
#if defined(WITH_XCP)
    if (CM_XCP_Calc() != 0) {
	if (SimCore.State == SCState_StartSim)
	    SimCore.Start.IsReady = 0;
    }
#endif
#if defined(WITH_CCP)
    if (CM_CCP_Calc() != 0) {
	if (SimCore.State == SCState_StartSim)
	    SimCore.Start.IsReady = 0;
    }
#endif
    SimCore_TCPU_TakeTS(&SimCore.TS.User);

    return rv;
}




/*
** App_TestRun_End ()
**
** call always at end of testrun
** if OnlyOneSimulation==0, models are ended at the beginning
** of next TestRun
*/

static void *
App_TestRun_End (void *arg)
{
    /* Remark: Not called if reconfiguration of the vehicle during
       the simulation is active (SimCore.Reconfig.Active=1) */

    if (AppStartInfo.Snapshot & Snapshot_Take)
	Vhcl_Snapshot_Export2Inf();

    User_TestRun_End ();

    ADASRP_StopClient ();

    Vhcl_Delete    (1);
    DrivMan_Delete ();
    Traffic_Delete ();
    PylonDetect_Delete ();

    RPSensor_Delete ();
    TSSensor_Delete ();
    LineSensor_Delete ();

    /* At the end free the Road-Handle after free all model own RoadEval-Handles before */
    Env_Delete ();

    SimCore_TestRun_End ();
    SimCore.End.Tid = 0;

    SimCore_TCPU_LogStats();

    return NULL;
}




static int
App_ShutDown (int ShutDownForced)
{
    int i;
    const int n = 1000;		/* max. n loops for testrig  shut down */

    for (i=0; i < n; i++) {
	if (User_ShutDown(ShutDownForced) == 1 || ShutDownForced)
	    break;
	SysUSleep (10);
    }
    return 0;
}



/*
** App_End ()
**
** called once before application exits
*/

static void
App_End (void)
{
#if defined(WITH_XCP)
    CM_XCP_End();
#endif
#if defined(WITH_CCP)
    CM_CCP_End();
#endif
    User_End ();

    DrivMan_Delete ();
    ExtInp_File_Delete ();
    TrfLight_Delete ();
    Vhcl_Delete (0);
    VehicleControl_Delete();
    Traffic_Delete ();
    PylonDetect_Delete ();
    CollisionDetect_Delete ();
     /* Am Ende Env_Delete wegen tRoad-Handle nach Freigabe aller tRoadEval-Handles */
    Env_Delete ();

    SimCore_End ();
    LogClose ();
}



/*
** App_Cleanup ()
**
** called once before application exits
*/

static void
App_Cleanup (void)
{
    Plugins_Cleanup ();
    CMTh_Cleanup  ();
#if defined(CM_HIL) && !defined(INTIME) && !defined(ARTE_INTIME)
    FST_Cleanup ();
#endif
#if defined(WITH_XCP)
    CM_XCP_Cleanup();
#endif
#if defined(WITH_CCP)
    CM_CCP_Cleanup();
#endif
    IO_Cleanup ();
    User_Cleanup ();
#if defined(LABCAR)
    LCO_Cleanup();
#endif
    DASensor_Cleanup ();
    FSSensor_Cleanup ();
    RPSensor_Cleanup ();
    TSSensor_Cleanup ();
    LineSensor_Cleanup ();
    GNSSModule_Cleanup ();
    BodySensor_Cleanup ();
    BdyFrame_CleanUp  ();
    Traffic_Cleanup ();
    Vhcl_Cleanup ();
    DrivMan_Cleanup ();
    Env_Cleanup ();
    TrfLight_Cleanup ();
    ExtInp_Cleanup ();
    SimCore_Cleanup ();
    LogCleanup ();

    AposPoll (0.0);
    AposExit ();

}







/*** Main Program *************************************************************/


/*
** MainThread_Init ()
**
** Initialize the main thread of the application
*/

static int
MainThread_Init (void)
{
    int	 rv =	0;
    int nError = Log_nError;

    /* install signal handler */
    if (!AppStartInfo.DontHandleSignals)
	SimCore_ConfigSignalHandler (ExitFcn);

    /*** Parameters for the application
     * - initialize dictionary
     * - ...
     */
    if (App_Init() < 0) {
	rv = -1;
	goto EndReturn;
    }

    /*** Model Registration */
    if (App_Register() < 0)
	rv = -1;

    /*** Add quantities to the data dictionary (can displayed, saved)
     *   and export configuration
     */
    if (App_DeclQuants() < 0)
	rv = -1;

    /*** finalize initialization:
     * - close dictionary
     * - export system configuration  ...
     */
    if (SimCore_Init_Finalize() < 0)
	rv = -1;

    /* everything ok until now? */
    if (nError != Log_nError && rv >= 0)
	rv = -1;
    if (rv < 0)
	goto EndReturn;


    /*** load idle parameters/testrun or start the selected testrun */
    if (AppStartInfo.TestRunName == NULL) {
	SimCore_State_Set(SCState_Start);
	SimCore_State_Switch();
	App_TestRun_Start(NULL);
	if (SimCore.Start.Ok) {
	    SimCore_State_Set(SCState_Idle);
	    SimCore_State_Switch();
	}
    } else {
	OnlyOneSimulation = 1;
	SimCore.TestRun.OnErrSaveHist = AppStartInfo.OnErrSaveHist;
        SimCore.TestRun.SaveMode =      AppStartInfo.SaveMode;
	SimStart(SimCore.TestRun.Project,
		 SimCore.TestRun.User,
		 AppStartInfo.TestRunName,
		 AppStartInfo.TestRunFName,
		 AppStartInfo.TestRunVariation);
	SimCore_State_Switch();
	/* Attention: SimState is SCState_SimStart, no params loaded. */
    }

    if (IO_Init_Finalize() < 0)
	rv = -1;


#if defined(XENO)
    /*** Timing of application main loop, timer device ***/
    rt_task_set_periodic (NULL, TM_NOW, 1000000);
#endif


    if (SimCore.State == SCState_Idle) {
	char tmp[64];
	AposSetInfo (NULL, NULL, "Idle");
	LogSpecial (0, "IDLE", "%s", GetLocalDateStr(tmp, 12, "%H:%M:%S", NULL));
    }

    /* any errors up to now ? */
    if (nError != Log_nError && rv >= 0)
	rv = -1;

  EndReturn:
#if defined(XENO) || defined(INTIME) || defined(ARTE_INTIME) || defined(LVRT)
    RefTime = SysGetTime();
#endif
#if !defined(CM_HIL)
    TimeWCAbs = 	SysGetTime();
    TimeWCRef = 	TimeWCAbs;
    dtSync =		0.0;
#endif

    /* activate scheduling and priority */
    CMTh_ReconfigurePriority (CMTh_Kind_Main);

    return rv;
}



static int
MainThread_BeginCycle (unsigned CycleNo)
{
#if defined(LABCAR)
    if (LCO_SendRecv() <= 0)
	return 1;
#endif
    /*** wait until cycle time passed */
    if ((DeltaT=WaitForNextLoop(CycleNo)) <= 0)
	return 1;

    SimCore_State_Switch();

    /*** Start Loop */
    SimCore_TCPU_TakeTS(&SimCore.TS.LoopStart);
    DVA_SetTime(SimCore.Time, SimCore.CycleNo);

    if (LogPoll(TimeGlobal-SimCore.Start.TimeGlob) != 1
     && SimCore.State > SCState_Start && SimCore.State < SCState_End) {
	SimCore_State_Set(SCState_End);
	SimCore_State_Switch();
    }


    if (InSyncWithVDS()) {
	/*** External Inputs */
	if (SimCore.State == SCState_Simulate)
	    ExtInp_File_Calc (SimCore.Time, (tExtInp_Mode)DrivMan.ExtInp.Mode);

	ADASRP_Receive();

	/*** Input from hardware */
	IO_In(CycleNo);
#if defined(WITH_XCP)
	CM_XCP_In();
#endif
#if defined(WITH_CCP)
	CM_CCP_In();
#endif
#if defined(LABCAR)
	LCO_Write(DVA_IO_In);
#endif
	Plugins_CalcBefore (DVA_IO_In, SimCore.DeltaT);
	DVA_HandleWriteAccess(DVA_IO_In);
	Plugins_CalcAfter (DVA_IO_In, SimCore.DeltaT);
	User_In(CycleNo);
    }

    SimCore_TCPU_TakeTS(&SimCore.TS.In);
    if (SimCore.MustTakeTS) {
	SimCore.TS.DrivMan
	    = SimCore.TS.Traffic
	    = SimCore.TS.VehicleControl
	    = SimCore.TS.Vehicle
	    = SimCore.TS.Trailer
	    = SimCore.TS.Brake
	    = SimCore.TS.PowerTrain
	    = SimCore.TS.Sensors
	    = SimCore.TS.User
	    = SimCore.TS.In;
    }

    return 0;
}



static int
MainThread_DoCycle (unsigned CycleNo)
{
    static int RampingDone;
    int rv;

#if !defined(CM_HIL)
    if (SimCore.Anim.Sync_State!=SyncOff && SimCore.State==SCState_Simulate) {
	if (SimCore_SyncVDS_Eval() != 0)
	    return 0;		/* Skip cycle. */
    }
#endif

    switch (SimCore.State) {
      case SCState_Start:
	/*** initialize test run start */
	SimCore_TestRun_Prepare();
	RampingDone = 0;

#if defined(XENO)
	RTGuard_Restart();
	SimCore.Cycle.nCTViolation = 0;
#endif

	SimCore_State_Set(SCState_StartWait);

	if (!OnlyOneSimulation
	 && CMTh_AttribGet(CMTh_Kind_TestRun_Start, CMTh_Attrb_MultiThread)) {
	    SimCore.Start.Tid = 1;
	    if ((rv=CMTh_Create(CMTh_Kind_TestRun_Start, App_TestRun_Start)) != 0) {
		LogErrF (EC_General, "Can't create SimStart thread (err %d, %s)",
			 rv, strerror(errno));
		SimCore.Start.Tid = 0;
	    }
	} else {
	    App_TestRun_Start (NULL);
	}
	break;

      case SCState_StartWait:
	/* do nothing, wait for App_TestRun_Start() to return */
	break;

      case SCState_StartWaitAnim:
	if (SimCore.TimeWC >= SimCore.Anim.Wait_UntilWC)
	    SimCore_State_Set (SCState_StartSim);
	break;

      case SCState_StartSim:
	/* ramp up, start engine and wait until engine speed is stable */
	if (!RampingDone) {
	    int nError = Log_nError;
	    RampingDone = User_TestRun_RampUp(DeltaT);
	    if (Log_nError != nError)
		SimCore_State_Set(SCState_End);
	} else {
	    SimCore.Start.IsReady = 1;
	    if (App_TestRun_Calc(DeltaT) < 0)
		SimCore_State_Set(SCState_End);
	    ADASRP_CheckReady();
	}

	if (SimCore.NextState == SCState_StartSim) {
	    if (RampingDone && SimCore.Start.IsReady) {
		SimCore_State_Set(SCState_StartLastCycle);
	    } else {
		if (SimCore.TimeWC - SimCore.Start.TimeWC > SimCore.Start.TimeLimit) {
		    LogErrF (EC_General, "Start hasn't finished within %g seconds."
			     " Testrun aborted.", SimCore.Start.TimeLimit);
		    SimCore_State_Set (SCState_End);
		}
	    }
	}
	break;

      case SCState_StartLastCycle:
	/* last pre-simulation calculation */
#if defined(XENO)
	RTGuard_Begin();
#endif
	if (App_TestRun_Calc(DeltaT) >= 0
	 && App_TestRun_Start_Finalize() >= 0) {
#if !defined(CM_HIL)
	    SimCore_SyncVDS_Restart();
#endif
	    SimCore_State_Set(SimCore.TAccel<=0.01 ? SCState_Pause : SCState_Simulate);
	    SimCore.Reconfig.Active = 0;
	} else {
	    SimCore_State_Set (SCState_End);
	}
	SimCore_TCPU_StartStats();
	break;


      case SCState_Idle:
	App_TestRun_Calc (DeltaT);
	break;


      case SCState_Simulate:
	if ((rv=App_TestRun_Calc(DeltaT)) != 0)
	    SimCore_State_Set (SCState_End);
	break;


      case SCState_Pause:
	    //wait for continue
	break;


      case SCState_End:
	if (SimCore.Start.Tid)
	    break;
	SimCore.End.TimeWC = SimCore.TimeWC;
	SimCore_TCPU_StopStats();
	User_TestRun_End_First();
	if (AppStartInfo.Snapshot & Snapshot_Take)
	    App_TestRun_Snapshot_Take();

	QuantAudit_Finish();

	/* do GetIdle? skip GetIdle, if start wasn't ok or Skip is set */
	if (SimCore.Start.Ok) {
	    SimCore_State_Set(SimCore.GetIdle.Skip ? SCState_EndIdleSet : SCState_EndIdleGet);
	} else {
	    SimCore_State_Set(SCState_EndClean);
	}

	/* leave vehicle if error occurs */
	if (SimCore.Start.nError < Log_nError) {
	    if (!DrivMan.OperatorActive)
		DrivMan.OperatorActive = 1;
	    DrivMan.OperationState_trg = OperState_Absent;
	}

	/* activate data storing? */
	if ((Log_TriggerSave || AppStartInfo.TimeStat) && SimCore.TestRun.OnErrSaveHist > 0)
	    DStoreSaveBegin(SimCore.Erg, SimCore.TestRun.OnErrSaveHist, 0);

	/* end data storing: no additional data vectors */
	if (DStoreActive(SimCore.Erg))
	    DStoreSaveEnd(SimCore.Erg, 0);

	if (SimCore.NextState != SCState_EndIdleGet)
	    break;
	/* fall through */


      case SCState_EndIdleGet:
	/* run down the models into idle conditions (without a real simulation) */
	rv = App_TestRun_Calc(DeltaT);
	if (rv < 0) 	/* error -> abort */
	    SimCore_State_Set(SCState_EndIdleSet);

	if (SimCore.TimeWC - SimCore.End.TimeWC - SimCore.GetIdle.TimeOffset
	    > SimCore.GetIdle.TimeLimit) {
	    LogErrF (EC_General, "Couldn't get idle within %g seconds. "
		     "GetIdle aborted, ignition switched off.",
		     SimCore.GetIdle.TimeLimit);

	    /* leave vehicle */
	    if (!DrivMan.OperatorActive)
		DrivMan.OperatorActive = 1;
	    DrivMan.OperationState_trg = OperState_Absent;

	    DStoreSaveEnd(SimCore.Erg, 1);
	    SimCore_State_Set(SCState_EndIdleSet);
	}

	if (!User_Check_IsIdle(rv == 3))
	    break;

	/*** check data storage state */
	{
	    unsigned long vs, bs, fs, n;
	    unsigned long long ss;
	    int active;
	    DStoreExtSaveStatus(SimCore.Erg, &vs, &bs, &fs, &ss, &n, &active);

	    if (active) {
		static unsigned long long SavedSizeLast;
		/* reset get idle time limit if storage is active and working */
		if (SavedSizeLast != ss) {
		    SavedSizeLast = ss;
		    SimCore.GetIdle.TimeOffset = SimCore.TimeWC - SimCore.End.TimeWC;
		}
		break;
	    }
	}

	SimCore_State_Set(SCState_EndIdleSet);
	break;


      case SCState_EndIdleSet:
	App_TestRun_Calc(DeltaT);
	SimCore_State_Set(SCState_EndClean);
	break;


      case SCState_EndClean:
	SimCore_State_Set(SCState_EndWait);
	if (CMTh_AttribGet(CMTh_Kind_TestRun_End, CMTh_Attrb_MultiThread)) {
	    SimCore.End.Tid = 1;
	    if ((rv=CMTh_Create(CMTh_Kind_TestRun_End, App_TestRun_End)) != 0) {
		LogErrF(EC_General, "Can't create end thread (returns %d, %s)",
			rv, strerror(errno));
		SimCore.End.Tid = 0;
	    }
	} else {
	    App_TestRun_End(NULL);
	}
	/* fall through */


      case SCState_EndWait:
	/* Wait until App_TestRun_End() finished */
	if (SimCore.End.Tid)
	    break;

#if defined(XENO)
	RTGuard_End();
#endif
	/* Stop simulation after the selected testrun */
	if (OnlyOneSimulation) {
	    SimCore_State_Set(SCState_Idle);
	    goto EndReturn;
	}
	SimCore_State_Set(SCState_EndLastCycle);
	break;


      case SCState_EndLastCycle:
	if (SimCore.Shutdown.Request != ShutdownNot) {
	    SimCore_State_Set(SCState_ShutDown);
	} else {
	    SimCore_State_Set(SCState_Idle);
	    AposSetInfo (NULL, NULL, "Idle");
	}
	break;

      case SCState_ShutDown:
	goto EndReturn;

      default:
	break;
    }
    return 0;

  EndReturn:
    return 1;
}



static void
MainThread_FinishCycle (unsigned CycleNo)
{
    /* output to the hardware */
    if (InSyncWithVDS()) {
	SessionCmds_Eval();
	User_Out (CycleNo);		/* -> IO */
#if defined(LABCAR)
	LCO_Write(DVA_IO_Out);
#endif
	Plugins_CalcBefore (DVA_IO_Out, SimCore.DeltaT);
	DVA_HandleWriteAccess(DVA_IO_Out);
	Plugins_CalcAfter (DVA_IO_Out, SimCore.DeltaT);
	IO_Out(CycleNo);		/* IO -> Hardware */

	ADASRP_Send();

	if (SimCore.State == SCState_Simulate) {
	    QuantAudit_Observe();
	    /* output to file */
	    if (SimCore.TestRun.SaveMode != 0 && DStorePutVec(SimCore.Erg) != 0) {
		LogWarnStr(EC_DStore,
			   "DataStorage: SaveBuffer overflow. Storage is stopped.");
		if (!DStoreActive(SimCore.Erg))
		    DStoreSaveEnd(SimCore.Erg, 0);
	    }
	}
    }

    SimCore_TCPU_TakeTS(&SimCore.TS.Out);


    /* APO-Server: Poll -- evaluate messages from clients */
    AposPoll(SimCore.AposPollTime);
    SimCore_TCPU_TakeTS(&SimCore.TS.AposPoll);

    ProcessApoMessages();

    /* APO-Server: Send, generate and send messages to clients */
#if defined(WITH_XCP)
    CM_XCP_ApoMsg_Send(TimeGlobal, CycleNo);
#endif
#if defined(WITH_CCP)
    CM_CCP_ApoMsg_Send(TimeGlobal, CycleNo);
#endif
#if defined(CM_HIL)
    User_ApoMsg_Send    (TimeGlobal, CycleNo);
    SimCore_ApoMsg_Send (TimeGlobal, CycleNo);
#else
    /* Messages synchronized to TimeWC, not to simulation time.
       Use "virtual CycleNo" for message generation. */
    CycleNoTimeWC = (unsigned long)(SimCore.TimeWC * 1e3);
    for (; CycleNoTimeWCNext < CycleNoTimeWC; CycleNoTimeWCNext++) {
	User_ApoMsg_Send    (TimeGlobal, CycleNoTimeWCNext);
	SimCore_ApoMsg_Send (TimeGlobal, CycleNoTimeWCNext);
    }
    CycleNoTimeWCNext = CycleNoTimeWC;
#endif

    SimCore_TCPU_TakeTS(&SimCore.TS.AposEvalSend);

    /* Calculate cpu-time, needed by different program sections */
    SimCore_TCPU_Eval ();

#  if defined(XENO)
    /* Check for switches to secondary mode */
    RTGuard_Eval();
#  endif
}



/*
** MainThread_End ()
**
** End the main thread of the application
*/

static void
MainThread_End (void)
{
}








/******************************************************************************/





/*
** main ()
**
** main program
*/

int
main (int argc, char **argv)
{
    int nError;

    /*** First initialisation of basic modules/structures
     * - failures are not allowed
     * - lowlevel initialization
     */
    App_Init_First (argc, argv);
    nError = Log_nError;

    /*** evaluate command line */
    if ((argv=SimCore_ScanCmdLine(&argc, argv)) != NULL
     && (argv=User_ScanCmdLine(argc, argv)) != NULL) {
	if (argv[0] != NULL) {
	    if (AppStartInfo.TestRunName != NULL)
		free (AppStartInfo.TestRunName);
	    AppStartInfo.TestRunName = PathConv2IPN(argv[0]);
	    OnlyOneSimulation = 1;
	    SimCore_SetMultiThreadMode (0);
	}
    }

    /*** Second initialisation of modules/structures */
    if (App_Init_Second() < 0 || Log_nError > nError) {
	if (OnlyOneSimulation)
	    return 1;
	SimCore_MainThreadEmergency ();
    }

    /* start the main application thread */
    if (MainThread_Init() >= 0) {
	unsigned CycleNo = 0;
	while (MainThread_BeginCycle(CycleNo) == 0
	    && MainThread_DoCycle(CycleNo)    == 0) {
	    MainThread_FinishCycle (CycleNo);
	    CycleNo++;
	}
    } else {
	if (!OnlyOneSimulation)
	    SimCore_MainThreadEmergency ();
    }

    /* shut testrig down */
    App_ShutDown (0);	/* shutdown desired	*/
    App_ShutDown (1);	/* shutdown forced	*/

    /* end of application */
    App_End ();
    MainThread_End ();
    App_Cleanup ();

    return SimCore_Handle_AppRestart ();
}







