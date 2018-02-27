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
** Connection to I/O hardware of the CarMaker/HIL test stand
**
** Connected test rig: ???
**
******************************************************************************
**
** Functions
** ---------
**
** - iGetCal ()
** - CalIn ()
** - CalInF ()
** - CalOut ()
** - CalOutF ()
** - LimitInt ()
** - IO_Init_First ()
** - IO_Init_Finalize ()
** - IO_Init ()
** - IO_Param_Get ()
** - IO_In ()
** - IO_Out ()
** - IO_Cleanup ()
**
******************************************************************************
*/

#include <Global.h>

#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <string.h>

#include <CarMaker.h>

#include <mio.h>
#if defined(CM_HIL)
# include <FailSafeTester.h>
#endif
# include <PowerUTA.h>

#include "IOVec.h"
#if defined(WITH_IO_CAN)
# include "IO_CAN.h"
#endif
#if defined(WITH_SOMEIP)
# include <sip_rbs.h>
#endif
#if defined(WITH_FLEXRAY)
# include <flex.h>
# include <flex_rbs.h>
#endif
#if defined(WITH_XCP)
# include <CM_XCP.h>
#endif
#if defined(WITH_CCP)
# include <CM_CCP.h>
#endif


/*** CAN Ids */
#define CAN_ID_App1		0x101
#define CAN_ID_App2		0x201





/*** I/O calibration */
struct tCalVec	Cal;


/*** I/O vector */
tIOVec IO;


/*** I/O configuration */

/* int IO_None; DON'T - Variable is predefined by CarMaker! */
int IO_DemoApp;
#if defined(WITH_FLEXRAY)
int IO_DemoFR;
#endif
#if defined(WITH_SOMEIP)
int IO_DemoSIP;
#endif

static struct tIOConfig IOConfiguration[] = {
/* This table should contain one line for each IO_xyz-flag in IOVec.h */
/*  { <Flagvar>,	<Name for -io>,	<Description for -help> },     */
    { &IO_None,		"none",		"No I/O" }, /* Always keep this first line! */
    { &IO_DemoApp,	"demoapp",	"Example configuration" },
#if defined(WITH_FLEXRAY)
    { &IO_DemoFR,	"demofr",	"FlexRay Example" },
#endif
#if defined(WITH_SOMEIP)
    { &IO_DemoSIP,	"demosip",	"SOME/IP Example" },
#endif
    { NULL, NULL, NULL } /* End of table */
};



/**** Additional useful functions *********************************************/


/*
** iGetCal ()
**
** Get calibration parameters
*/

void
iGetCal (tInfos *Inf, const char *key, tCal *cal, int optional)
{
    int  i, j;
    char *item;

    memset (cal, 0, sizeof (*cal));
    cal->Min       =  1e37;
    cal->Max       = -1e37;
    cal->LimitLow  =  1e37;
    cal->LimitHigh = -1e37;
    cal->Factor    =  1.0;
    cal->Offset    =  0.0;
    cal->Rezip     =  0;

    if ((item = iGetStrOpt(Inf, key, NULL)) != NULL) {
	i = sscanf(item, "%g %g %g %g %d",
		   &cal->LimitLow, &cal->LimitHigh,
		   &cal->Factor,   &cal->Offset, &j);
	cal->Rezip = j;
	if (i != 5)
	    LogErrF (EC_Init, "Entry not correct for key '%s'", key);
    } else {
	if (!optional) {
	    LogErrF (EC_Init, "No entry for key '%s'", key);
	    return;
	}
	cal->LimitLow  = -1e37;
	cal->LimitHigh =  1e37;
    }
    cal->Min = cal->LimitHigh;
    cal->Max = cal->LimitLow;
}



/*
** CalIn ()
**
** Analog input -> calibration infos -> physical quantity
** Converts an integer value (normally the value from an analog input module)
** to the corresponding physical value.
** The result is limited by LimitLow and LimitHigh.
*/

float
CalIn (tCal *cal, const int Value)
{
    float Result;

    Result = (Value-cal->Offset) * cal->Factor;

    if (cal->Rezip)
        Result = 1/Result;

    if      (Result < cal->Min) cal->Min = Result;
    else if (Result > cal->Max) cal->Max = Result;

    if      (Result < cal->LimitLow)  Result = cal->LimitLow;
    else if (Result > cal->LimitHigh) Result = cal->LimitHigh;

    return Result;
}



/*
** CalInF ()
**
** Analog input -> calibration infos -> physical quantity
** Converts a float value (e.g. the voltage from an analog input module)
** to the corresponding physical value.
** The result is limited by LimitLow and LimitHigh.
*/

float
CalInF (tCal *cal, const float Value)
{
    float Result;

    Result = (Value-cal->Offset) * cal->Factor;

    if (cal->Rezip)
	Result = 1/Result;

    if      (Result < cal->Min)  cal->Min = Result;
    else if (Result > cal->Max)  cal->Max = Result;

    if      (Result < cal->LimitLow)   Result = cal->LimitLow;
    else if (Result > cal->LimitHigh)  Result = cal->LimitHigh;

    return Result;
}



/*
** CalOut ()
**
** physical quantity -> calibration infos -> integer value
** The physical value is limited by LimitLow and LimitHigh and then
** converted to the corresponding integer value (for an analog output module).
*/

int
CalOut (tCal *cal, float Value)
{
    if      (Value < cal->Min)	      cal->Min = Value;
    else if (Value > cal->Max)        cal->Max = Value;

    if      (Value < cal->LimitLow)   Value = cal->LimitLow;
    else if (Value > cal->LimitHigh)  Value = cal->LimitHigh;

    if (cal->Rezip)
        return (int) (1.0 / (Value*cal->Factor) + cal->Offset);
    else
	return (int) (Value/cal->Factor + cal->Offset);
}



/*
** CalOutF ()
**
** physical quantity -> calibration infos -> analog output
** The physical value is limited by LimitLow and LimitHigh and then converted
** to the corresponding analog value (e.g. voltage for an analog output module).
*/

float
CalOutF (tCal *cal, float Value)
{
    if (Value < cal->Min)
	cal->Min = Value;
    else if (Value > cal->Max)
	cal->Max = Value;

    if (Value < cal->LimitLow)
	Value = cal->LimitLow;
    else if (Value > cal->LimitHigh)
	Value = cal->LimitHigh;

    if (cal->Rezip)
	return 1.0 / (Value*cal->Factor) + cal->Offset;
    else
	return Value/cal->Factor + cal->Offset;
}



int
LimitInt (float fValue, int Min, int Max)
{
    int Value = (int)fValue;
    if      (Value < Min) return Min;
    else if (Value > Max) return Max;
    return   Value;
}



/*****************************************************************************/


/*
** IO_Init_First ()
**
** First, low level initialization of the IO module
**
** Call:
** - one times at start of program
** - no realtime conditions
*/

int
IO_Init_First (void)
{
    memset (&IO,  0, sizeof(IO));
    memset (&Cal, 0, sizeof(Cal));

    IO_SetConfigurations(IOConfiguration);

#if defined(WITH_IO_CAN)
    /* Init I/O data structures */
    IO_CAN_Init_First();
#endif

#if defined(WITH_FLEXRAY)
    FC_Init_First();
    RBS_Init_First();
#endif
#if defined(WITH_SOMEIP)
    SIP_Init_First();
#endif

    return 0;
}



/*
** IO_Init ()
**
** initialization
** - i/o hardware
** - add variables to data dictionary
**
** call:
** - single call at program start
*/

int
IO_Init (void)
{
    tDDictEntry *e;

    Log("I/O Configuration: %s\n", IO_ListNames(NULL, 1));

    /* create new error classes */
    LogDefineErrClass (EC_App1, LogAction_abort10s, "Example error class 1", 1);
    LogDefineErrClass (EC_App2, LogAction_abort10s, "Example error class 2", 1);

    /*** add I/O independent variables to the data dictionary (dependent see below) */
    DDefFloat(NULL, "IO.ax",      	"m/s^2",   &IO.ax,         DVA_IO_In);
    DDefFloat(NULL, "IO.ay",      	"m/s^2",   &IO.ay,         DVA_IO_In);
    DDefFloat(NULL, "IO.YawRate",    	"rad/s",   &IO.YawRate,    DVA_IO_In);
    DDefFloat(NULL, "IO.StWhlAngle", 	"rad",     &IO.StWhlAngle, DVA_IO_In);

    /* hardware configuration "none" */
    if (IO_None)
	return 0;

    int nErrors = Log_nError;

    /*** MIO initialization */
    if (MIO_Init(NULL) < 0) {
	LogErrF(EC_General, "MIO initialization failed. I/O disabled (1)");
	IO_SelectNone();
	return -1;
    }
    // MIO_ModuleShow ();

    /* hardware configuration "demoapp" */
    if (IO_DemoApp) {
	/* ModuleSetUp */
	// MIO_M35_Config (Slot_AD);	/* 12bit Analog In,  (16 Channels) */
	// MIO_M62_Config (Slot_DA);	/* 12bit Analog Out, (16 Channels) */

	/* CAN */
	MIO_M51_Config (Slot_CAN, -1 /* default IRQlevel */);

	// MIO_M31_Config (Slot_Din);	/* 16bit Digital In */
	// MIO_M400_Config (Slot_FG);	/* Digital FG */
	// MIO_M43_Config (Slot_Rel);	/* 8bit Relais Output */

	/* check for errors */
	if (nErrors != Log_nError) {
	    LogErrF(EC_General, "MIO initization failed. I/O disabled (2)");
	    IO_SelectNone();
	    return -1;
	}

	MIO_M51_SetCommParam (Slot_CAN, 0, 500000, 70, 2, 0);
	/* configure acceptance filter for Rx CAN messages (here: accept all) */
	MIO_M51_EnableIds (Slot_CAN, 0, 0 /* start ID */, 2048 /* num IDs */);
	/* when working with extended CAN messages, activate transparent mode */
	// MIO_M51_SetTransMode (Slot_CAN, 0, 1);

	/** Config Frequency Generator **/
	// MIO_M400_SetMode (Slot_FG, 0, 1, 0);
	// MIO_M400_SetMode (Slot_FG, 1, 1, 0);
	// MIO_M400_SetMode (Slot_FG, 2, 1, 0);
	// MIO_M400_SetMode (Slot_FG, 3, 1, 0);
	// MIO_M400_SetSignalsPerTurn (Slot_FG, 0, 48 /* number of teeth */);
	// MIO_M400_SetSignalsPerTurn (Slot_FG, 1, 48 /* number of teeth */);
	// MIO_M400_SetSignalsPerTurn (Slot_FG, 2, 48 /* number of teeth */);
	// MIO_M400_SetSignalsPerTurn (Slot_FG, 3, 48 /* number of teeth */);
	// MIO_M400_SetDutyCycle (Slot_FG, 0, 0.5);
	// MIO_M400_SetDutyCycle (Slot_FG, 1, 0.5);
	// MIO_M400_SetDutyCycle (Slot_FG, 2, 0.5);
	// MIO_M400_SetDutyCycle (Slot_FG, 3, 0.5);

	/*** FailSafeTester */
	FST_ConfigureCAN();

	/* Power UTA */
	PowerUTA_Init (&IO.PwrUTA, 20 /* Umax */, 76 /* Imax */, NULL);
	IO.PwrSupply.I_Set = 20;
	IO.PwrSupply.U_Set = 12;

	PowerUTA_DeclQuants (&IO.PwrUTA, "PwrSupply");
	tDDefault * df = DDefaultCreate("PwrSupply");
	e = DDefChar(df, ".PWR",	"", 	&IO.PwrSupply.PWR,    DVA_IO_Out);
	DDefStates(e, 2, 0);
	DDefFloat (df, ".U_Read",	"V",	&IO.PwrSupply.U_Read, DVA_IO_In);
	DDefFloat (df, ".I_Read",	"A",	&IO.PwrSupply.I_Read, DVA_IO_In);
	DDefFloat (df, ".U_Set",	"V",	&IO.PwrSupply.U_Set,  DVA_IO_Out);
	DDefFloat (df, ".I_Set",	"A",	&IO.PwrSupply.I_Set,  DVA_IO_Out);
	DDefaultDelete(df);


	/*** add variables to the data dictionary */
	e = DDefChar (NULL, "IO.BLS", "", &IO.BLS, DVA_None);
	DDefStates(e, 2, 0);

	DDefFloat  (NULL, "IO.HydValve_In_FL",		"",       &IO.HydValve[ 0], DVA_IO_In);
	DDefFloat  (NULL, "IO.HydValve_In_FR",		"",       &IO.HydValve[ 1], DVA_IO_In);
	DDefFloat  (NULL, "IO.HydValve_In_RL",		"",       &IO.HydValve[ 2], DVA_IO_In);
	DDefFloat  (NULL, "IO.HydValve_In_RR",		"",       &IO.HydValve[ 3], DVA_IO_In);
	DDefFloat  (NULL, "IO.HydValve_Out_FL",		"",       &IO.HydValve[ 4], DVA_IO_Out);
	DDefFloat  (NULL, "IO.HydValve_Out_FR",		"",       &IO.HydValve[ 5], DVA_IO_Out);
	DDefFloat  (NULL, "IO.HydValve_Out_RL",		"",       &IO.HydValve[ 6], DVA_IO_Out);
	DDefFloat  (NULL, "IO.HydValve_Out_RR",		"",       &IO.HydValve[ 7], DVA_IO_Out);
	DDefFloat  (NULL, "IO.HydValve_PV_FRRL",	"",       &IO.HydValve[ 8], DVA_None);
	DDefFloat  (NULL, "IO.HydValve_PV_FLRR",	"",       &IO.HydValve[ 9], DVA_None);
	DDefFloat  (NULL, "IO.HydValve_SV_FRRL",	"",       &IO.HydValve[10], DVA_None);
	DDefFloat  (NULL, "IO.HydValve_SV_FLRR",	"",       &IO.HydValve[11], DVA_None);

	e = DDefChar(NULL, "IO.PumpIsOn",		"",       &IO.PumpIsOn,     DVA_None);
	DDefStates(e, 2, 0);
	e = DDefChar(NULL, "IO.Rel_SW",			"",       &IO.Rel_SW,       DVA_None);
	DDefStates(e, 2, 0);
	DDefFloat  (NULL, "IO.BooSignal",		"",   	  &IO.BooSignal,    DVA_None);
	DDefFloat  (NULL, "IO.PuRetVolt",		"V",	  &IO.PuRetVolt,    DVA_None);
	DDefFloat  (NULL, "IO.pMC",			"bar",	  &IO.pMC,          DVA_None);
	DDefFloat  (NULL, "IO.pWB_FL",			"bar",	  &IO.pWB[0],       DVA_None);
	DDefFloat  (NULL, "IO.pWB_FR",			"bar",	  &IO.pWB[1],       DVA_None);
	DDefFloat  (NULL, "IO.pWB_RL",			"bar",	  &IO.pWB[2],       DVA_None);
	DDefFloat  (NULL, "IO.pWB_RR",			"bar",	  &IO.pWB[3],       DVA_None);

        DDefFloat (NULL, "IO.WheelSpd_FL",		"rad/s",  &IO.WheelSpd[0],  DVA_IO_Out);
        DDefFloat (NULL, "IO.WheelSpd_FR",		"rad/s",  &IO.WheelSpd[1],  DVA_IO_Out);
	DDefFloat (NULL, "IO.WheelSpd_RL",		"rad/s",  &IO.WheelSpd[2],  DVA_IO_Out);
	DDefFloat (NULL, "IO.WheelSpd_RR",		"rad/s",  &IO.WheelSpd[3],  DVA_IO_Out);

#if defined(WITH_IO_CAN)
	IO_CAN_Init();
#endif
    }

#if defined(WITH_FLEXRAY)
    /* hardware configuration "demofr" */
    if (IO_DemoFR) {
        if (FC_Init())
            return -1;
        if (RBS_Init())
            return -1;
    }
#endif
#if defined(WITH_SOMEIP)
    /* hardware configuration "demosip" */
    if (IO_DemoSIP) {
        if (SIP_Init())
            return -1;
    }
#endif

    return 0;
}


/*
** IO_Init_Finalize ()
**
** last (deferred) I/O initialization step
**
** call:
** - single call at program start in CarMaker_FinishStartup()
*/

int
IO_Init_Finalize (void)
{
    if (IO_DemoApp) {
    }

#if defined(WITH_FLEXRAY)
    if (IO_DemoFR) {
        RBS_MapQuants();
        if (FC_Start())
            return -1;
        if (RBS_Start())
            return -1;
    }
#endif
#if defined(WITH_SOMEIP)
    if (IO_DemoSIP) {
        SIP_MapQuants();
        if (SIP_Start())
            return -1;
    }
#endif

    return 0;
}


/*
** IO_Param_Get ()
**
** Get i/o configuration parameters
** - calibration
** - constant values
** - ids
*/

int
IO_Param_Get (tInfos *inf)
{
    unsigned nError = GetInfoErrorCount ();

    /* ignition off */
    SetKl15 (0);

    if (IO_None)
    	return 0;

    if (IO_DemoApp) {
	/* Read ECUParameters calibration info for use with the
	   CalIn(), CalOut() and LimitInt() functions defined above. */

	/* Power UTA */
	iGetCal(inf, "CalPowerUTA.Imon", &Cal.Imon, 1);
	iGetCal(inf, "CalPowerUTA.Umon", &Cal.Umon, 1);
	iGetCal(inf, "CalPowerUTA.Iout", &Cal.Iout, 1);
	iGetCal(inf, "CalPowerUTA.Uout", &Cal.Uout, 1);

	/* iGetCal(inf, "Calibrate.ax",         &Cal.ax, 0);         */
	/* iGetCal(inf, "Calibrate.ay",         &Cal.ay, 0);         */
	/* iGetCal(inf, "Calibrate.YawRate",    &Cal.YawRate, 0);    */
	/* iGetCal(inf, "Calibrate.StWhlAngle", &Cal.StWhlAngle, 0); */

	/* IO.CAN_Version =  iGetLongOpt(inf,  "IO.CAN_Version", 0); */
	/* IO.GearBoxCode =  iGetLongOpt(inf,  "IO.GearBoxCode", 0); */
	/* IO.LWS_Id = 	     iGetLongOpt(inf,  "IO.LWS_Id",      0); */
	/* IO.EngineCode =   1;                                      */
    }

#if defined(WITH_FLEXRAY)
    if (IO_DemoFR) {
        FC_Param_Get(inf, NULL);
        RBS_Param_Get(inf, NULL);
    }
#endif
#if defined(WITH_SOMEIP)
    if (IO_DemoSIP)
        SIP_Param_Get(inf, NULL);
#endif

    return nError != GetInfoErrorCount() ? -1 : 0;
}



/*
** IO_In ()
**
** reading signals from hardware / ECU
**
** CycleNo: simulation cycle counter, incremented every loop/millisecond
**
** call:
** - in the main loop
** - first function call in main loop, after waiting for next loop
** - just before User_In()
** - pay attention to realtime condition
*/

void
IO_In (unsigned CycleNo)
{
    CAN_Msg	Msg;

    IO.DeltaT =	SimCore.DeltaT;
    IO.T =	TimeGlobal;

    if (IO_None)
	return;

    /*** FailSafeTester messages */
    if (FST_IsActive()) {
	while (MIO_M51_Recv(FST_CAN_Slot, FST_CAN_Ch, &Msg) == 0)
	    FST_MsgIn (CycleNo, &Msg);
    }

    if (IO_DemoApp) {
	 /* Power UTA */
	IO.PwrSupply.U_Read = CalInF(&Cal.Umon, IO.PwrUTA.Umon);
	IO.PwrSupply.I_Read = CalInF(&Cal.Imon, IO.PwrUTA.Imon);

	/* process Rx CAN messages */
#if defined(WITH_IO_CAN)
	IO_CAN_RecvLoop(Slot_CAN, 0, CycleNo);
#else
	while (1) {
	    if (MIO_M51_Recv(Slot_CAN, 0, &Msg) != 0)
		break;
#if defined(WITH_XCP)
	    if (CM_XCP_Dispatch_CAN_Msg(&Msg) == 0)
		continue;
#endif
#if defined(WITH_CCP)
	    if (CM_CCP_Dispatch_CAN_Msg(&Msg) == 0)
		continue;
#endif
	    switch (Msg.MsgId) {
	      case CAN_ID_App1:
		/* IO.xx = Msg.Data[1]<<8 | Msg.Data[0]; */
		break;
	      case CAN_ID_App2:
		/* IO.xx = Msg.Data[3]<<24 | Msg.Data[2]<<16; */
		break;
	    }
	}
#endif /* !WITH_IO_CAN */
    }

#if defined(WITH_FLEXRAY)
    if (IO_DemoFR) {
        FC_In(CycleNo);
        RBS_In(CycleNo);
    }
#endif
#if defined(WITH_SOMEIP)
    if (IO_DemoSIP)
        SIP_In(CycleNo);
#endif
}



/*
** IO_Out ()
**
** writing signals to hardware / ECU
**
** CycleNo: simulation cycle counter, incremented every loop/millisecond
**
** call:
** - in the main loop
** - last function call in main loop
** - just after User_Out()
** - pay attention to realtime condition
*/

void
IO_Out (unsigned CycleNo)
{
    if (IO_None)
	return;

    /*** Messages to the FailSafeTester */
    FST_MsgOut(CycleNo);

    if (IO_DemoApp) {
	/* Power UTA */
	IO.PwrUTA.Standby = !IO.PwrSupply.PWR;
	IO.PwrUTA.Uout    = CalOutF(&Cal.Uout, IO.PwrSupply.U_Set);
	IO.PwrUTA.Iout    = CalOutF(&Cal.Iout, IO.PwrSupply.I_Set);

	/*
	 * MIO_xx();
	 *
	 * MIO_M62_Set (Slot_DA, 0, CalOut(&Cal.PActUnit[0], IO.PActUnit[0]));
	 * MIO_M62_Set (Slot_DA, 2, CalOut(&Cal.YawRate,     IO.YawRate    ));
	 * MIO_M62_Set (Slot_DA, 3, CalOut(&Cal.ay,          IO.ay         ));
	 * ...
	 * MIO_M51_Send (Slot_CAN, 0, &M2);
	 */
    
        /*** Frequency Generator */
        // MIO_M400_SetWheelspeed (Slot_FG, 0, fabs(IO.WheelSpd[0]));
        // MIO_M400_SetWheelspeed (Slot_FG, 1, fabs(IO.WheelSpd[1]));
        // MIO_M400_SetWheelspeed (Slot_FG, 2, fabs(IO.WheelSpd[2]));
        // MIO_M400_SetWheelspeed (Slot_FG, 3, fabs(IO.WheelSpd[3]));
#if 0
	if (CycleNo % 100 == 0) {
	    CAN_Msg Msg = {CAN_ID_App1, 0, 0, 2};
	    int angle;
	    angle = 23;
	    Msg.Data[0] =  angle       & 0xff;
	    Msg.Data[1] = (angle >> 8) & 0xff;
	    MIO_M51_Send (Slot_CAN, 0, &Msg);
	}
#endif
#if defined(WITH_IO_CAN)
	IO_CAN_Send(Slot_CAN, 0, CycleNo);
#endif
    }

#if defined(WITH_FLEXRAY)
    if (IO_DemoFR)
        RBS_Out(CycleNo);
#endif
#if defined(WITH_SOMEIP)
    if (IO_DemoSIP)
        SIP_Out(CycleNo);
#endif
}



/*
** IO_Cleanup ()
**
** Uninits all MIO hardware:
** - puts M-Modules into reset state
** - frees unneeded memory
*/

void
IO_Cleanup (void)
{
    if (IO_None)
	goto EndReturn;

#if defined(WITH_FLEXRAY)
    if (IO_DemoFR) {
        RBS_Cleanup();
        FC_Cleanup();
    }
#endif
#if defined(WITH_SOMEIP)
    if (IO_DemoSIP)
        SIP_Cleanup();
#endif
    MIO_ResetModules();
    MIO_DeleteAll();

  EndReturn:
    PowerUTA_Finish(&IO.PwrUTA);
    return;
}
