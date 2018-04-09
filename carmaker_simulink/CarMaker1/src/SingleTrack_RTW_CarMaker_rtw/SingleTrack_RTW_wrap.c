/***************************************************** target specific file ***/
/*  Wrapper module for Simulink models                                        */
/*  ------------------------------------------------------------------------  */
/*  Copyright (c) IPG Automotive GmbH      www.ipg.de   Fon: +49.721.98520-0  */
/*  Bannwaldallee 60      D-76185 Karlsruhe   Germany   Fax: +49.721.98520-99 */
/******************************************************************************/

#ifndef IS_CAR
# define IS_CAR
#endif

#include "Global.h"

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <math.h>

#include "simstruc.h"
#include "rt_sim.h"
#include "simstruc_types.h"

#include "InfoUtils.h"
#include "DataDict.h"
#include "SimCore.h"
#include "Log.h"
#include "MatSupp.h"
#include "DrivMan.h"
#include "ModelManager.h"

#include "Vehicle.h"
#include "VehicleControl.h"
#include "Vehicle/MBSUtils.h"
#include "Car/Brake.h"
#include "Car/Tire.h"
#include "Car/PowerTrain.h"

#include "SingleTrack_RTW.h"
#include "SingleTrack_RTW_wrap.h"


#define QUOTE1(name)	#name
#define QUOTE(name)	QUOTE1(name)		/* need to expand name */

#ifndef CLASSIC_INTERFACE
# define EXPAND_CONCAT(name1,name2) name1 ## name2
# define CONCAT(name1,name2) EXPAND_CONCAT(name1,name2)
# define MODEL_INITIALIZE CONCAT(MODEL,_initialize)
# define MODEL_STEP       CONCAT(MODEL,_step)
# define MODEL_TERMINATE  CONCAT(MODEL,_terminate)
# define RT_MDL_TYPE      CONCAT(MODEL,_M_TYPE)
#endif

extern const char SingleTrack_RTW_LibIdent[];
const char SingleTrack_RTW_LibIdent[] = "(@@)" QUOTE(MODEL) " " ARCH " 1.0 " BUILDDATE;

static const char Modelname[] = QUOTE(MODEL);
static const tModelClass Modelclass = ModelClass_Vehicle;
static tMatSuppSampling SampleParams;

/* Skip calculation of the vehicle during preprocessing, just like in CM4SL? */
static int SkipPreprocessing = 0;

/* Vehicle tire data.
   Tires will always be initialized according to the testrun/vehicle dataset,
   but whether they are actually used (i.e. calculated) depends entirely on the
   presence of tire blocks in the vehicle Simulink model. */
static tTire Tires[VEHICLE_NWHLS];

static void
DoOneStep (rtModel_SingleTrack_RTW *rtm)
{
#ifdef CLASSIC_INTERFACE
    real_T tnext;

    tnext = rt_SimGetNextSampleHit(rtmGetTimingData(rtm),
				   rtmGetNumSampleTimes(rtm));
    rtsiSetSolverStopTime(rtmGetRTWSolverInfo(rtm),tnext);

    rtmiOutputs(rtmGetRTWRTModelMethodsInfo(rtm),0);
    rtmiUpdate(rtmGetRTWRTModelMethodsInfo(rtm),0);
    rt_SimUpdateDiscreteTaskSampleHits(rtmGetNumSampleTimes(rtm),
				       rtmGetTimingData(rtm),
				       rtmGetSampleHitPtr(rtm),
				       rtmGetTPtr(rtm));

    if (rtmGetSampleTime(rtm,0) == CONTINUOUS_SAMPLE_TIME) {
	if (rtmGetNumContStates(rtm) > 0) {
	    rt_ODEUpdateContinuousStates(rtmGetRTWSolverInfo(rtm));
	} else {
	    rtsiSetT(rtmGetRTWSolverInfo(rtm), rtsiGetSolverStopTime(rtmGetRTWSolverInfo(rtm)));
	}
    }
#else
    MODEL_STEP(rtm);
#endif
}


/*
 * Define dictionary entries made from tunable parameters.
 * The function will actually be called twice, first with tuns==NULL.
 */
static void
DeclParameterQuants (struct tMatSuppTunables *tuns)
{
    /*MatSupp_TunDDictDefScalar(tuns, "MyParam", INFOMAT_DOUBLE, "kappa", "kg/s");*/
}


static void
SingleTrack_RTW_DeclQuants (void *MP)
{
    rtModel_SingleTrack_RTW *rtm = (rtModel_SingleTrack_RTW *)MP;
    int i;

    /*Log("%s_DeclQuants()\n", Modelname);*/

    if (rtm == NULL) {
	/* Remember body frames defined in this model for later registration. */
        MdlBdyFrame_Add(SingleTrack_RTW_BdyFrameDefines);

	/* Define dict entries for non-dynamically allocated variables. */
	if ((i = MatSupp_DeclQuants(SingleTrack_RTW_DictDefines)) >= 0) {
	    LogErrF(EC_Init, "Model '%s': Error defining quantity '%s'\n",
		    Modelname, SingleTrack_RTW_DictDefines[i].Name);
	}

	/* Define dict entries for tunable parameters (with dummy address). */
	DeclParameterQuants(NULL);
    } else {
	/* Define dict entries for dynamically allocated variables. */
    }
}


/*
 * SingleTrack_RTW_SetParams() will be invoked indirectly by the generated
 * model C code each time SingleTrack_RTW_New() is called.
 */
void
SingleTrack_RTW_SetParams (rtModel_SingleTrack_RTW *rtm, struct tMatSuppTunables *tuns,
		   struct tInfos *Inf)
{
    /*Log("%s_SetParams()\n", Modelname);*/

    /*
     * Parameter tuning - Part 1
     * This is the place to modify parameters of a storage class
     * other than 'SimulinkGlobal'.
     */

    if (tuns == NULL)
        return;

    /* Define dict entries for tunable parameters (address update). */
    DeclParameterQuants(tuns);

    /*
     * Parameter tuning - Part 2
     * This is the place to modify parameters of storage class
     * 'SimulinkGlobal', e.g. using the CarMaker target's tunable parameter
     * interface.
     */

    const char *prefix = Model_Class2Str(Modelclass);
    MatSupp_TunReadAllOpt(tuns, Inf, prefix);
    /*MatSupp_TunReadAll(tuns, ...);*/
    /*MatSupp_TunReadDef(tuns, ...);*/
    /*MatSupp_TunRead(tuns, ...);*/
}


static void
CfgIF2VehicleModel (void *MP)
{
    rtModel_SingleTrack_RTW *rtm = (rtModel_SingleTrack_RTW *)MP;
    ExternalInputs_SingleTrack_RTW  *in  = (ExternalInputs_SingleTrack_RTW *) rtmGetU(rtm);

    in->CfgInFromCM.MinStartPos		= Vehicle.Cfg.MinStartPos;
    in->CfgInFromCM.PoI_Pos_1.x		= Vehicle.Cfg.PoI_Pos_1[0];
    in->CfgInFromCM.PoI_Pos_1.y		= Vehicle.Cfg.PoI_Pos_1[1];
    in->CfgInFromCM.PoI_Pos_1.z		= Vehicle.Cfg.PoI_Pos_1[2];
    in->CfgInFromCM.PoI2AxleFront	= Vehicle.Cfg.PoI2AxleFront;
    in->CfgInFromCM.PoI2AxleRear	= Vehicle.Cfg.PoI2AxleRear;
    in->CfgInFromCM.PoI2CarEndLeft	= Vehicle.Cfg.PoI2CarEndLeft;
    in->CfgInFromCM.PoI2CarEndRight	= Vehicle.Cfg.PoI2CarEndRight;
    in->CfgInFromCM.CoG2AxleFront	= Vehicle.Cfg.CoG2AxleFront;
    in->CfgInFromCM.CoG2AxleRear	= Vehicle.Cfg.CoG2AxleRear;
    in->CfgInFromCM.WheelBase		= Vehicle.Cfg.WheelBase;
    in->CfgInFromCM.TrackWidth		= Vehicle.Cfg.TrackWidth;
    in->CfgInFromCM.OverhangFront	= Vehicle.Cfg.OverhangFront;
    in->CfgInFromCM.OverhangRear	= Vehicle.Cfg.OverhangRear;
    in->CfgInFromCM.MassTotal		= Vehicle.Cfg.MassTotal;
    in->CfgInFromCM.Inertia.Ixx		= Vehicle.Cfg.Inertia[0][0];
    in->CfgInFromCM.Inertia.Ixy		= Vehicle.Cfg.Inertia[0][1];
    in->CfgInFromCM.Inertia.Ixz		= Vehicle.Cfg.Inertia[0][2];
    in->CfgInFromCM.Inertia.Iyx		= Vehicle.Cfg.Inertia[1][0];
    in->CfgInFromCM.Inertia.Iyy		= Vehicle.Cfg.Inertia[1][1];
    in->CfgInFromCM.Inertia.Iyz		= Vehicle.Cfg.Inertia[1][2];
    in->CfgInFromCM.Inertia.Izx		= Vehicle.Cfg.Inertia[2][0];
    in->CfgInFromCM.Inertia.Izy		= Vehicle.Cfg.Inertia[2][1];
    in->CfgInFromCM.Inertia.Izz		= Vehicle.Cfg.Inertia[2][2];
    in->CfgInFromCM.nWheels		= Vehicle.Cfg.nWheels;
    in->CfgInFromCM.WhlRadius		= Vehicle.Cfg.WhlRadius;
    in->CfgInFromCM.iDiff		= Vehicle.Cfg.iDiff;
    in->CfgInFromCM.nFGears		= Vehicle.Cfg.nFGears;
    in->CfgInFromCM.nBGears		= Vehicle.Cfg.nBGears;
    in->CfgInFromCM.iFGear.Gear1	= Vehicle.Cfg.iFGear[1];
    in->CfgInFromCM.iFGear.Gear2	= Vehicle.Cfg.iFGear[2];
    in->CfgInFromCM.iFGear.Gear3	= Vehicle.Cfg.iFGear[3];
    in->CfgInFromCM.iFGear.Gear4	= Vehicle.Cfg.iFGear[4];
    in->CfgInFromCM.iFGear.Gear5	= Vehicle.Cfg.iFGear[5];
    in->CfgInFromCM.iFGear.Gear6	= Vehicle.Cfg.iFGear[6];
    in->CfgInFromCM.iFGear.Gear7	= Vehicle.Cfg.iFGear[7];
    in->CfgInFromCM.iFGear.Gear8	= Vehicle.Cfg.iFGear[8];
    in->CfgInFromCM.iFGear.Gear9	= Vehicle.Cfg.iFGear[9];
    in->CfgInFromCM.iFGear.Gear10	= Vehicle.Cfg.iFGear[10];
    in->CfgInFromCM.iFGear.Gear11	= Vehicle.Cfg.iFGear[11];
    in->CfgInFromCM.iFGear.Gear12	= Vehicle.Cfg.iFGear[12];
    in->CfgInFromCM.iFGear.Gear13	= Vehicle.Cfg.iFGear[13];
    in->CfgInFromCM.iFGear.Gear14	= Vehicle.Cfg.iFGear[14];
    in->CfgInFromCM.iFGear.Gear15	= Vehicle.Cfg.iFGear[15];
    in->CfgInFromCM.iFGear.Gear16	= Vehicle.Cfg.iFGear[16];
    in->CfgInFromCM.iBGear.Gear1	= Vehicle.Cfg.iBGear[1];
    in->CfgInFromCM.iBGear.Gear2	= Vehicle.Cfg.iBGear[2];
    in->CfgInFromCM.iBGear.Gear3	= Vehicle.Cfg.iBGear[3];
    in->CfgInFromCM.iBGear.Gear4	= Vehicle.Cfg.iBGear[4];
    in->CfgInFromCM.GBKind		= Vehicle.Cfg.GBKind;
    in->CfgInFromCM.PTKind		= Vehicle.Cfg.PTKind;
    in->CfgInFromCM.SteerByTorque	= Vehicle.Cfg.SteerByTorque;
    in->CfgInFromCM.StartEngineWithSST	= Vehicle.Cfg.StartEngineWithSST;

    in->CfgInFromCM.Whl_Iyy.FL		= Vehicle.Cfg.Whl_Iyy[0];
    in->CfgInFromCM.Whl_Iyy.FR		= Vehicle.Cfg.Whl_Iyy[1];
    in->CfgInFromCM.Whl_Iyy.RL		= Vehicle.Cfg.Whl_Iyy[2];
    in->CfgInFromCM.Whl_Iyy.RR		= Vehicle.Cfg.Whl_Iyy[3];
#if defined (IS_TRUCK)
    in->CfgInFromCM.Whl_Iyy.RL2		= Vehicle.Cfg.Whl_Iyy[4];
    in->CfgInFromCM.Whl_Iyy.RR2		= Vehicle.Cfg.Whl_Iyy[5];
    in->CfgInFromCM.Whl_Iyy.FL2		= Vehicle.Cfg.Whl_Iyy[6];
    in->CfgInFromCM.Whl_Iyy.FR2		= Vehicle.Cfg.Whl_Iyy[7];
#endif
    in->CfgInFromCM.Engine_rotv_Idle	= Vehicle.Cfg.Engine_rotv_Idle;
    in->CfgInFromCM.Engine_rotv_Max	= Vehicle.Cfg.Engine_rotv_Max;
    in->CfgInFromCM.Velocity_Max	= Vehicle.Cfg.Velocity_Max;
    in->CfgInFromCM.Aero_Frcx		= Vehicle.Cfg.Aero_Frcx;

    in->CfgInFromCM.DrivenFront2	= Vehicle.Cfg.DrivenFront2;
    in->CfgInFromCM.DrivenRear2		= Vehicle.Cfg.DrivenRear2;

    in->CfgInFromCM.Whl_rotv.FL		= Vehicle.Cfg.Whl_rotv[0];
    in->CfgInFromCM.Whl_rotv.FR		= Vehicle.Cfg.Whl_rotv[1];
    in->CfgInFromCM.Whl_rotv.RL		= Vehicle.Cfg.Whl_rotv[2];
    in->CfgInFromCM.Whl_rotv.RR		= Vehicle.Cfg.Whl_rotv[3];
#if defined (IS_TRUCK)
    in->CfgInFromCM.Whl_rotv.RL2	= Vehicle.Cfg.Whl_rotv[4];
    in->CfgInFromCM.Whl_rotv.RR2	= Vehicle.Cfg.Whl_rotv[5];
    in->CfgInFromCM.Whl_rotv.FL2	= Vehicle.Cfg.Whl_rotv[6];
    in->CfgInFromCM.Whl_rotv.FR2	= Vehicle.Cfg.Whl_rotv[7];
#endif
    in->CfgInFromCM.GearBox_rotv_out	= Vehicle.Cfg.GearBox_rotv_out;
    in->CfgInFromCM.Engine_rotv		= Vehicle.Cfg.Engine_rotv;
    in->CfgInFromCM.Roll		= Vehicle.Cfg.Roll;
    in->CfgInFromCM.Yaw			= Vehicle.Cfg.Yaw;
    in->CfgInFromCM.Pitch		= Vehicle.Cfg.Pitch;
    in->CfgInFromCM.Fr1_Pos.x		= Vehicle.Cfg.Fr1_Pos[0];
    in->CfgInFromCM.Fr1_Pos.y		= Vehicle.Cfg.Fr1_Pos[1];
    in->CfgInFromCM.Fr1_Pos.z		= Vehicle.Cfg.Fr1_Pos[2];
    in->CfgInFromCM.Distance		= Vehicle.Cfg.Distance;
}


static void *
SingleTrack_RTW_New (struct tInfos *Inf)
{
    rtModel_SingleTrack_RTW *rtm;
    double rtmSampleTime;

    /*Log("%s_New()\n", Modelname);*/

    MatSupp_ResetQuants(SingleTrack_RTW_DictDefines);

    rtm = MODEL(Inf);

#ifdef CLASSIC_INTERFACE
    rtmSetT(rtm, 0.0);
    rtmSetTFinal(rtm, -1 /*run forever*/);

    rtmiInitializeSizes(rtmGetRTWRTModelMethodsInfo(rtm));
    rtmiInitializeSampleTimes(rtmGetRTWRTModelMethodsInfo(rtm));
    rt_SimInitTimingEngine(rtmGetNumSampleTimes(rtm),
			   rtmGetStepSize(rtm),
			   rtmGetSampleTimePtr(rtm),
			   rtmGetOffsetTimePtr(rtm),
			   rtmGetSampleHitPtr(rtm),
			   rtmGetSampleTimeTaskIDPtr(rtm),
			   rtmGetTStart(rtm),
			   &rtmGetSimTimeStep(rtm),
			   &rtmGetTimingData(rtm));
    if (rtmGetNumContStates(rtm) > 0) {
	rt_ODECreateIntegrationData(rtmGetRTWSolverInfo(rtm));
    } else {
	rtsiSetSolverName(rtmGetRTWSolverInfo(rtm), "FixedStepDiscrete");
    }
    rtsiSetVariableStepSolver(rtmGetRTWSolverInfo(rtm), 0);
#endif

    rtmSampleTime = (double) rtmGetStepSize(rtm);
    if (MatSupp_Sampling (&SampleParams, SimCore.DeltaT, rtmSampleTime) !=0) {
	LogErrF(EC_Init, "Model '%s': The sample times of the plugin model and the application have to be integer multiples\n",Modelname);
	return NULL;
    }

    CfgIF2VehicleModel(rtm);

#ifdef CLASSIC_INTERFACE
    rtmiStart(rtmGetRTWRTModelMethodsInfo(rtm));
#else
    MODEL_INITIALIZE(rtm);
#endif

    return rtm; /* Will be passed as MP to the other functions. */
}


static void
SingleTrack_RTW_Delete (void *MP)
{
    rtModel_SingleTrack_RTW *rtm = (rtModel_SingleTrack_RTW *)MP;

    /*Log("%s_Delete()\n", Modelname);*/

#ifdef CLASSIC_INTERFACE
    rt_SimDestroyTimingEngine(rtmGetTimingData(rtm));
    if (rtmGetNumContStates(rtm) > 0)
	rt_ODEDestroyIntegrationData(rtmGetRTWSolverInfo(rtm));
    rtmiTerminate(rtmGetRTWRTModelMethodsInfo(rtm));
#else
    MODEL_TERMINATE(rtm);
#endif

    int TireNo;
    for (TireNo=0; TireNo<Vehicle.Cfg.nWheels; TireNo++)
	VhclModel_Tire_Delete(TireNo);
}


static int
SingleTrack_RTW_Calc (void *MP, double dt)
{
    rtModel_SingleTrack_RTW *rtm = (rtModel_SingleTrack_RTW *)MP;
    int osCount=0;
    //ExternalInputs_SingleTrack_RTW  *in  = (ExternalInputs_SingleTrack_RTW *) rtmGetU(rtm);
    //ExternalOutputs_SingleTrack_RTW *out = (ExternalOutputs_SingleTrack_RTW *)rtmGetY(rtm);

    if (SampleParams.UnderSampFac) {	// Undersampling
     	if (++SampleParams.UnderSampCount == SampleParams.UnderSampFac) {
	    SampleParams.UnderSampCount=0;
	    DoOneStep(rtm);
     	}
    } else { 				// Oversampling (1..OverSampFac)
	do {
	    DoOneStep(rtm);
	} while (++osCount < SampleParams.OverSampFac);
    }

    return 0;
}


/******************************************************************************/


static void *
VehicleModel_New (struct tInfos *Inf)
{
    void *MP;

    /* Get vehicle configuration parameters from vehicle infofile */
    if (Vehicle_GetVhclCfg (Inf, VhclClass_Car_Id) < 0)
	return NULL;

    /* Initialize tires */
    int TireNo, TireFailure = 0;
    for (TireNo=0; TireNo<Vehicle.Cfg.nWheels; TireNo++) {
	const int side = TireNo%2==0 ? 1 : -1;	/* 1 left, -1 right */
	tTire *tire = &Tires[TireNo];

	if (VhclModel_Tire_New(tire, Inf, TireNo, side, 0) != 0)
	    TireFailure = 1;
    }
    if (TireFailure)
	return NULL;

    /* Register Vehicle.Fr1A for sensors and animation. */
    BdyFrame_Register(BdyFrame_MountFrameStr(Vehicle_Fr1A), &Vehicle.Fr1A);

    /* Provide vehicle geometry information for animation tool IPGMovie. */
    if (Vehicle_CreateVhclMsgFromCfg (VhclClass_Car_Id) < 0)
	return NULL;

    if ((MP = SingleTrack_RTW_New(Inf)) == NULL)
	return NULL;

    SkipPreprocessing = iGetBoolOpt(Inf, "Vehicle.SkipPreprocessing", 0);

    /* Disable postprocessing after end of simulation, i.e. DON'T pilot the
       vehicle to a stand-still state, freeze actual vehicle state instead. */
    SimCore.GetIdle.Skip = 1;

    if (SkipPreprocessing) {
	/* Provide roughly meaningful values for the first invocation of IPGDriver
	   in state "Simulate", as the first call to our vehicle model's calc()-
	   function only comes afterwards when not taking part in preprocessing. */
	Vehicle.PoI_Pos[0] = DrivMan.Cfg.RoadPos[0];
	Vehicle.PoI_Pos[1] = DrivMan.Cfg.RoadPos[1];
	Vehicle.PoI_Pos[2] = DrivMan.Cfg.RoadPos[2];
	Vehicle.PoI_Vel[0] = DrivMan.Cfg.Velocity * DrivMan.Cfg.RoadDir[0];
	Vehicle.PoI_Vel[1] = DrivMan.Cfg.Velocity * DrivMan.Cfg.RoadDir[1];
    }

    return MP;
}


static void
CM2VehicleModel (ExternalInputs_SingleTrack_RTW *in)
{
    in->FromCM.VhclCtrl.SST                 = VehicleControl.SST;
    in->FromCM.VhclCtrl.Key                 = VehicleControl.Key;
    in->FromCM.VhclCtrl.UserSignal.s0       = VehicleControl.UserSignal[0];
    in->FromCM.VhclCtrl.UserSignal.s1       = VehicleControl.UserSignal[1];
    in->FromCM.VhclCtrl.UserSignal.s2       = VehicleControl.UserSignal[2];
    in->FromCM.VhclCtrl.UserSignal.s3       = VehicleControl.UserSignal[3];
    in->FromCM.VhclCtrl.UserSignal.s4       = VehicleControl.UserSignal[4];

    in->FromCM.VhclCtrl.GearNo              = VehicleControl.GearNo;
    in->FromCM.VhclCtrl.SelectorCtrl        = VehicleControl.SelectorCtrl;

    in->FromCM.VhclCtrl.Brake               = VehicleControl.Brake;
    in->FromCM.VhclCtrl.BrakePark           = VehicleControl.BrakePark;
    in->FromCM.VhclCtrl.BrakeLever          = VehicleControl.BrakeLever;
    in->FromCM.VhclCtrl.Clutch              = VehicleControl.Clutch;
    in->FromCM.VhclCtrl.Gas                 = VehicleControl.Gas;

    in->FromCM.VhclCtrl.Steering.Ang        = VehicleControl.Steering.Ang;
    in->FromCM.VhclCtrl.Steering.AngVel     = VehicleControl.Steering.AngVel;
    in->FromCM.VhclCtrl.Steering.AngAcc     = VehicleControl.Steering.AngAcc;
    in->FromCM.VhclCtrl.Steering.Trq        = VehicleControl.Steering.Trq;
    in->FromCM.VhclCtrl.Steering.SteerByTrq = DrivMan.ActualMan.SteerBy==DMSteerBy_Trq;

    in->FromCM.VhclCtrl.Rider.RollAng       = VehicleControl.Rider.RollAng;
    in->FromCM.VhclCtrl.Rider.RollAngVel    = VehicleControl.Rider.RollAngVel;

    in->FromCM.VhclCtrl.Lights.LowBeam      = VehicleControl.Lights.LowBeam;
    in->FromCM.VhclCtrl.Lights.HighBeam     = VehicleControl.Lights.HighBeam;
    in->FromCM.VhclCtrl.Lights.Daytime      = VehicleControl.Lights.Daytime;
    in->FromCM.VhclCtrl.Lights.ParkL        = VehicleControl.Lights.ParkL;
    in->FromCM.VhclCtrl.Lights.ParkR        = VehicleControl.Lights.ParkR;
    in->FromCM.VhclCtrl.Lights.IndL         = VehicleControl.Lights.IndL;
    in->FromCM.VhclCtrl.Lights.IndR         = VehicleControl.Lights.IndR;
    in->FromCM.VhclCtrl.Lights.FogFrontL    = VehicleControl.Lights.FogFrontL;
    in->FromCM.VhclCtrl.Lights.FogFrontR    = VehicleControl.Lights.FogFrontR;
    in->FromCM.VhclCtrl.Lights.FogRear      = VehicleControl.Lights.FogRear;
    in->FromCM.VhclCtrl.Lights.Brake        = VehicleControl.Lights.Brake;
    in->FromCM.VhclCtrl.Lights.Reverse      = VehicleControl.Lights.Reverse;
    in->FromCM.VhclCtrl.Lights.bm           = VehicleControl.Lights.bm;

    in->FromCM.Vhcl.Wheel.FL.Trq_Brake = Vehicle.FL.Trq_Brake;
    in->FromCM.Vhcl.Wheel.FL.Trq_B2WC  = Vehicle.FL.Trq_B2WC;
    in->FromCM.Vhcl.Wheel.FL.rot       = Vehicle.FL.rot;
    in->FromCM.Vhcl.Wheel.FL.rotv      = Vehicle.FL.rotv;
    in->FromCM.Vhcl.Wheel.FL.Trq_DL2WC = Vehicle.FL.Trq_DL2WC;

    in->FromCM.Vhcl.Wheel.FR.Trq_Brake = Vehicle.FR.Trq_Brake;
    in->FromCM.Vhcl.Wheel.FR.Trq_B2WC  = Vehicle.FR.Trq_B2WC;
    in->FromCM.Vhcl.Wheel.FR.rot       = Vehicle.FR.rot;
    in->FromCM.Vhcl.Wheel.FR.rotv      = Vehicle.FR.rotv;
    in->FromCM.Vhcl.Wheel.FR.Trq_DL2WC = Vehicle.FR.Trq_DL2WC;

    in->FromCM.Vhcl.Wheel.RL.Trq_Brake = Vehicle.RL.Trq_Brake;
    in->FromCM.Vhcl.Wheel.RL.Trq_B2WC  = Vehicle.RL.Trq_B2WC;
    in->FromCM.Vhcl.Wheel.RL.rot       = Vehicle.RL.rot;
    in->FromCM.Vhcl.Wheel.RL.rotv      = Vehicle.RL.rotv;
    in->FromCM.Vhcl.Wheel.RL.Trq_DL2WC = Vehicle.RL.Trq_DL2WC;

    in->FromCM.Vhcl.Wheel.RR.Trq_Brake = Vehicle.RR.Trq_Brake;
    in->FromCM.Vhcl.Wheel.RR.Trq_B2WC  = Vehicle.RR.Trq_B2WC;
    in->FromCM.Vhcl.Wheel.RR.rot       = Vehicle.RR.rot;
    in->FromCM.Vhcl.Wheel.RR.rotv      = Vehicle.RR.rotv;
    in->FromCM.Vhcl.Wheel.RR.Trq_DL2WC = Vehicle.RR.Trq_DL2WC;

#if defined (IS_TRUCK)
    in->FromCM.Vhcl.Wheel.RL2.Trq_Brake = Vehicle.RL2.Trq_Brake;
    in->FromCM.Vhcl.Wheel.RL2.Trq_B2WC  = Vehicle.RL2.Trq_B2WC;
    in->FromCM.Vhcl.Wheel.RL2.rot       = Vehicle.RL2.rot;
    in->FromCM.Vhcl.Wheel.RL2.rotv      = Vehicle.RL2.rotv;
    in->FromCM.Vhcl.Wheel.RL2.Trq_DL2WC = Vehicle.RL2.Trq_DL2WC;

    in->FromCM.Vhcl.Wheel.RR2.Trq_Brake = Vehicle.RR2.Trq_Brake;
    in->FromCM.Vhcl.Wheel.RR2.Trq_B2WC  = Vehicle.RR2.Trq_B2WC;
    in->FromCM.Vhcl.Wheel.RR2.rot       = Vehicle.RR2.rot;
    in->FromCM.Vhcl.Wheel.RR2.rotv      = Vehicle.RR2.rotv;
    in->FromCM.Vhcl.Wheel.RR2.Trq_DL2WC = Vehicle.RR2.Trq_DL2WC;

    in->FromCM.Vhcl.Wheel.FL2.Trq_Brake = Vehicle.FL2.Trq_Brake;
    in->FromCM.Vhcl.Wheel.FL2.Trq_B2WC  = Vehicle.FL2.Trq_B2WC;
    in->FromCM.Vhcl.Wheel.FL2.rot       = Vehicle.FL2.rot;
    in->FromCM.Vhcl.Wheel.FL2.rotv      = Vehicle.FL2.rotv;
    in->FromCM.Vhcl.Wheel.FL2.Trq_DL2WC = Vehicle.FL2.Trq_DL2WC;

    in->FromCM.Vhcl.Wheel.FR2.Trq_Brake = Vehicle.FR2.Trq_Brake;
    in->FromCM.Vhcl.Wheel.FR2.Trq_B2WC  = Vehicle.FR2.Trq_B2WC;
    in->FromCM.Vhcl.Wheel.FR2.rot       = Vehicle.FR2.rot;
    in->FromCM.Vhcl.Wheel.FR2.rotv      = Vehicle.FR2.rotv;
    in->FromCM.Vhcl.Wheel.FR2.Trq_DL2WC = Vehicle.FR2.Trq_DL2WC;
#endif

    in->FromCM.Vhcl.Trq_DL2Bdy1.x   = Vehicle.Trq_DL2Bdy1[0];
    in->FromCM.Vhcl.Trq_DL2Bdy1.y   = Vehicle.Trq_DL2Bdy1[1];
    in->FromCM.Vhcl.Trq_DL2Bdy1.z   = Vehicle.Trq_DL2Bdy1[2];
    in->FromCM.Vhcl.Trq_DL2Bdy1B.x  = Vehicle.Trq_DL2Bdy1B[0];
    in->FromCM.Vhcl.Trq_DL2Bdy1B.y  = Vehicle.Trq_DL2Bdy1B[1];
    in->FromCM.Vhcl.Trq_DL2Bdy1B.z  = Vehicle.Trq_DL2Bdy1B[2];
    in->FromCM.Vhcl.Trq_DL2BdyEng.x = Vehicle.Trq_DL2BdyEng[0];
    in->FromCM.Vhcl.Trq_DL2BdyEng.y = Vehicle.Trq_DL2BdyEng[1];
    in->FromCM.Vhcl.Wind_vel_0.x    = Vehicle.Wind_vel_0[0];
    in->FromCM.Vhcl.Wind_vel_0.y    = Vehicle.Wind_vel_0[1];
    in->FromCM.Vhcl.Wind_vel_0.z    = Vehicle.Wind_vel_0[2];
}


static void
VehicleModel2CM (const ExternalOutputs_SingleTrack_RTW *out)
{
    Vehicle.sRoad        = out->ToCM.Motion.sRoad;
    Vehicle.sRoadAero    = out->ToCM.Motion.sRoadAero;
    Vehicle.tRoad        = out->ToCM.Motion.tRoad;
    Vehicle.Distance     = out->ToCM.Motion.Distance;
    Vehicle.v            = out->ToCM.Motion.v;

    Vehicle.PoI_Pos[0]   = out->ToCM.PoI.Pos.x;
    Vehicle.PoI_Pos[1]   = out->ToCM.PoI.Pos.y;
    Vehicle.PoI_Pos[2]   = out->ToCM.PoI.Pos.z;
    Vehicle.PoI_Vel[0]   = out->ToCM.PoI.Vel.x;
    Vehicle.PoI_Vel[1]   = out->ToCM.PoI.Vel.y;
    Vehicle.PoI_Vel[2]   = out->ToCM.PoI.Vel.z;
    Vehicle.PoI_Acc[0]   = out->ToCM.PoI.Acc.x;
    Vehicle.PoI_Acc[1]   = out->ToCM.PoI.Acc.y;
    Vehicle.PoI_Acc[2]   = out->ToCM.PoI.Acc.z;

    if (GCSOn) {
	GCS_ConvFr0toGCS(Vehicle.PoI_Pos, &Vehicle.PoI_GCS);
    } else {
	Vehicle.PoI_GCS.Elev = Vehicle.PoI_Pos[2];
    }

    Vehicle.PoI_Vel_1[0] = out->ToCM.PoI.Vel_1.x;
    Vehicle.PoI_Vel_1[1] = out->ToCM.PoI.Vel_1.y;
    Vehicle.PoI_Vel_1[2] = out->ToCM.PoI.Vel_1.z;
    Vehicle.PoI_Acc_1[0] = out->ToCM.PoI.Acc_1.x;
    Vehicle.PoI_Acc_1[1] = out->ToCM.PoI.Acc_1.y;
    Vehicle.PoI_Acc_1[2] = out->ToCM.PoI.Acc_1.z;

    Vehicle.Hitch_Pos[0] = out->ToCM.Motion.Hitch_Pos.x;
    Vehicle.Hitch_Pos[1] = out->ToCM.Motion.Hitch_Pos.y;
    Vehicle.Hitch_Pos[2] = out->ToCM.Motion.Hitch_Pos.z;

    Vehicle.Fr1A.t_0[0]		= out->ToCM.Fr1A.t_0.x;
    Vehicle.Fr1A.t_0[1]		= out->ToCM.Fr1A.t_0.y;
    Vehicle.Fr1A.t_0[2]		= out->ToCM.Fr1A.t_0.z;
    Vehicle.Fr1A.v_0[0]		= out->ToCM.Fr1A.v_0.x;
    Vehicle.Fr1A.v_0[1]		= out->ToCM.Fr1A.v_0.y;
    Vehicle.Fr1A.v_0[2]		= out->ToCM.Fr1A.v_0.z;
    Vehicle.Fr1A.a_0[0]		= out->ToCM.Fr1A.a_0.x;
    Vehicle.Fr1A.a_0[1]		= out->ToCM.Fr1A.a_0.y;
    Vehicle.Fr1A.a_0[2]		= out->ToCM.Fr1A.a_0.z;
    Vehicle.Fr1A.omega_0[0]	= out->ToCM.Fr1A.omega_0.x;
    Vehicle.Fr1A.omega_0[1]	= out->ToCM.Fr1A.omega_0.y;
    Vehicle.Fr1A.omega_0[2]	= out->ToCM.Fr1A.omega_0.z;
    Vehicle.Fr1A.alpha_0[0]	= out->ToCM.Fr1A.alpha_0.x;
    Vehicle.Fr1A.alpha_0[1]	= out->ToCM.Fr1A.alpha_0.y;
    Vehicle.Fr1A.alpha_0[2]	= out->ToCM.Fr1A.alpha_0.z;
    Vehicle.Fr1A.Tr2Fr0[0][0]	= out->ToCM.Fr1A.Tr2Fr0.xx;
    Vehicle.Fr1A.Tr2Fr0[0][1]	= out->ToCM.Fr1A.Tr2Fr0.xy;
    Vehicle.Fr1A.Tr2Fr0[0][2]	= out->ToCM.Fr1A.Tr2Fr0.xz;
    Vehicle.Fr1A.Tr2Fr0[1][0]	= out->ToCM.Fr1A.Tr2Fr0.yx;
    Vehicle.Fr1A.Tr2Fr0[1][1]	= out->ToCM.Fr1A.Tr2Fr0.yy;
    Vehicle.Fr1A.Tr2Fr0[1][2]	= out->ToCM.Fr1A.Tr2Fr0.yz;
    Vehicle.Fr1A.Tr2Fr0[2][0]	= out->ToCM.Fr1A.Tr2Fr0.zx;
    Vehicle.Fr1A.Tr2Fr0[2][1]	= out->ToCM.Fr1A.Tr2Fr0.zy;
    Vehicle.Fr1A.Tr2Fr0[2][2]	= out->ToCM.Fr1A.Tr2Fr0.zz;

    Vehicle.Yaw          = out->ToCM.Motion.Yaw;
    Vehicle.YawRate      = out->ToCM.Motion.YawRate;
    Vehicle.YawAcc       = out->ToCM.Motion.YawAcc;
    Vehicle.Roll         = out->ToCM.Motion.Roll;
    Vehicle.RollVel      = out->ToCM.Motion.RollVel;
    Vehicle.RollAcc      = out->ToCM.Motion.RollAcc;
    Vehicle.Pitch        = out->ToCM.Motion.Pitch;
    Vehicle.PitchVel     = out->ToCM.Motion.PitchVel;
    Vehicle.PitchAcc     = out->ToCM.Motion.PitchAcc;

    Vehicle.FL.t[0]             = out->ToCM.WheelOut.FL.t.x;
    Vehicle.FL.t[1]             = out->ToCM.WheelOut.FL.t.y;
    Vehicle.FL.t[2]             = out->ToCM.WheelOut.FL.t.z;
    Vehicle.FL.r_zxy[0]         = out->ToCM.WheelOut.FL.r_zxy.x;
    Vehicle.FL.r_zxy[1]         = out->ToCM.WheelOut.FL.r_zxy.y;
    Vehicle.FL.r_zxy[2]         = out->ToCM.WheelOut.FL.r_zxy.z;
    Vehicle.FL.LongSlip         = out->ToCM.WheelOut.FL.LongSlip;
    Vehicle.FL.SideSlip         = out->ToCM.WheelOut.FL.SideSlip;
    Vehicle.FL.Fx               = out->ToCM.WheelOut.FL.Fx;
    Vehicle.FL.Fy               = out->ToCM.WheelOut.FL.Fy;
    Vehicle.FL.Fz               = out->ToCM.WheelOut.FL.Fz;
    Vehicle.FL.FxTwin           = out->ToCM.WheelOut.FL.FxTwin;
    Vehicle.FL.FyTwin           = out->ToCM.WheelOut.FL.FyTwin;
    Vehicle.FL.FzTwin           = out->ToCM.WheelOut.FL.FzTwin;
    Vehicle.FL.Trq_T2W          = out->ToCM.WheelOut.FL.Trq_T2W;
    Vehicle.FL.Trq_WhlBearing   = out->ToCM.WheelOut.FL.Trq_WhlBearing;
    Vehicle.FL.vBelt            = out->ToCM.WheelOut.FL.vBelt;
    Vehicle.FL.Trq_BrakeReg     = out->ToCM.WheelOut.FL.Trq_BrakeReg;
    Vehicle.FL.Trq_BrakeReg_max = out->ToCM.WheelOut.FL.Trq_BrakeReg_max;

    Vehicle.FR.t[0]             = out->ToCM.WheelOut.FR.t.x;
    Vehicle.FR.t[1]             = out->ToCM.WheelOut.FR.t.y;
    Vehicle.FR.t[2]             = out->ToCM.WheelOut.FR.t.z;
    Vehicle.FR.r_zxy[0]         = out->ToCM.WheelOut.FR.r_zxy.x;
    Vehicle.FR.r_zxy[1]         = out->ToCM.WheelOut.FR.r_zxy.y;
    Vehicle.FR.r_zxy[2]         = out->ToCM.WheelOut.FR.r_zxy.z;
    Vehicle.FR.LongSlip         = out->ToCM.WheelOut.FR.LongSlip;
    Vehicle.FR.SideSlip         = out->ToCM.WheelOut.FR.SideSlip;
    Vehicle.FR.Fx               = out->ToCM.WheelOut.FR.Fx;
    Vehicle.FR.Fy               = out->ToCM.WheelOut.FR.Fy;
    Vehicle.FR.Fz               = out->ToCM.WheelOut.FR.Fz;
    Vehicle.FR.FxTwin           = out->ToCM.WheelOut.FR.FxTwin;
    Vehicle.FR.FyTwin           = out->ToCM.WheelOut.FR.FyTwin;
    Vehicle.FR.FzTwin           = out->ToCM.WheelOut.FR.FzTwin;
    Vehicle.FR.Trq_T2W          = out->ToCM.WheelOut.FR.Trq_T2W;
    Vehicle.FR.Trq_WhlBearing   = out->ToCM.WheelOut.FR.Trq_WhlBearing;
    Vehicle.FR.vBelt            = out->ToCM.WheelOut.FR.vBelt;
    Vehicle.FR.Trq_BrakeReg     = out->ToCM.WheelOut.FR.Trq_BrakeReg;
    Vehicle.FR.Trq_BrakeReg_max = out->ToCM.WheelOut.FR.Trq_BrakeReg_max;

    Vehicle.RL.t[0]             = out->ToCM.WheelOut.RL.t.x;
    Vehicle.RL.t[1]             = out->ToCM.WheelOut.RL.t.y;
    Vehicle.RL.t[2]             = out->ToCM.WheelOut.RL.t.z;
    Vehicle.RL.r_zxy[0]         = out->ToCM.WheelOut.RL.r_zxy.x;
    Vehicle.RL.r_zxy[1]         = out->ToCM.WheelOut.RL.r_zxy.y;
    Vehicle.RL.r_zxy[2]         = out->ToCM.WheelOut.RL.r_zxy.z;
    Vehicle.RL.LongSlip         = out->ToCM.WheelOut.RL.LongSlip;
    Vehicle.RL.SideSlip         = out->ToCM.WheelOut.RL.SideSlip;
    Vehicle.RL.Fx               = out->ToCM.WheelOut.RL.Fx;
    Vehicle.RL.Fy               = out->ToCM.WheelOut.RL.Fy;
    Vehicle.RL.Fz               = out->ToCM.WheelOut.RL.Fz;
    Vehicle.RL.FxTwin           = out->ToCM.WheelOut.RL.FxTwin;
    Vehicle.RL.FyTwin           = out->ToCM.WheelOut.RL.FyTwin;
    Vehicle.RL.FzTwin           = out->ToCM.WheelOut.RL.FzTwin;
    Vehicle.RL.Trq_T2W          = out->ToCM.WheelOut.RL.Trq_T2W;
    Vehicle.RL.Trq_WhlBearing   = out->ToCM.WheelOut.RL.Trq_WhlBearing;
    Vehicle.RL.vBelt            = out->ToCM.WheelOut.RL.vBelt;
    Vehicle.RL.Trq_BrakeReg     = out->ToCM.WheelOut.RL.Trq_BrakeReg;
    Vehicle.RL.Trq_BrakeReg_max = out->ToCM.WheelOut.RL.Trq_BrakeReg_max;

    Vehicle.RR.t[0]             = out->ToCM.WheelOut.RR.t.x;
    Vehicle.RR.t[1]             = out->ToCM.WheelOut.RR.t.y;
    Vehicle.RR.t[2]             = out->ToCM.WheelOut.RR.t.z;
    Vehicle.RR.r_zxy[0]         = out->ToCM.WheelOut.RR.r_zxy.x;
    Vehicle.RR.r_zxy[1]         = out->ToCM.WheelOut.RR.r_zxy.y;
    Vehicle.RR.r_zxy[2]         = out->ToCM.WheelOut.RR.r_zxy.z;
    Vehicle.RR.LongSlip         = out->ToCM.WheelOut.RR.LongSlip;
    Vehicle.RR.SideSlip         = out->ToCM.WheelOut.RR.SideSlip;
    Vehicle.RR.Fx               = out->ToCM.WheelOut.RR.Fx;
    Vehicle.RR.Fy               = out->ToCM.WheelOut.RR.Fy;
    Vehicle.RR.Fz               = out->ToCM.WheelOut.RR.Fz;
    Vehicle.RR.FxTwin           = out->ToCM.WheelOut.RR.FxTwin;
    Vehicle.RR.FyTwin           = out->ToCM.WheelOut.RR.FyTwin;
    Vehicle.RR.FzTwin           = out->ToCM.WheelOut.RR.FzTwin;
    Vehicle.RR.Trq_T2W          = out->ToCM.WheelOut.RR.Trq_T2W;
    Vehicle.RR.Trq_WhlBearing   = out->ToCM.WheelOut.RR.Trq_WhlBearing;
    Vehicle.RR.vBelt            = out->ToCM.WheelOut.RR.vBelt;
    Vehicle.RR.Trq_BrakeReg     = out->ToCM.WheelOut.RR.Trq_BrakeReg;
    Vehicle.RR.Trq_BrakeReg_max = out->ToCM.WheelOut.RR.Trq_BrakeReg_max;

#if defined (IS_TRUCK)
    Vehicle.RL2.t[0]             = out->ToCM.WheelOut.RL2.t.x;
    Vehicle.RL2.t[1]             = out->ToCM.WheelOut.RL2.t.y;
    Vehicle.RL2.t[2]             = out->ToCM.WheelOut.RL2.t.z;
    Vehicle.RL2.r_zxy[0]         = out->ToCM.WheelOut.RL2.r_zxy.x;
    Vehicle.RL2.r_zxy[1]         = out->ToCM.WheelOut.RL2.r_zxy.y;
    Vehicle.RL2.r_zxy[2]         = out->ToCM.WheelOut.RL2.r_zxy.z;
    Vehicle.RL2.LongSlip         = out->ToCM.WheelOut.RL2.LongSlip;
    Vehicle.RL2.SideSlip         = out->ToCM.WheelOut.RL2.SideSlip;
    Vehicle.RL2.Fx               = out->ToCM.WheelOut.RL2.Fx;
    Vehicle.RL2.Fy               = out->ToCM.WheelOut.RL2.Fy;
    Vehicle.RL2.Fz               = out->ToCM.WheelOut.RL2.Fz;
    Vehicle.RL2.FxTwin           = out->ToCM.WheelOut.RL2.FxTwin;
    Vehicle.RL2.FyTwin           = out->ToCM.WheelOut.RL2.FyTwin;
    Vehicle.RL2.FzTwin           = out->ToCM.WheelOut.RL2.FzTwin;
    Vehicle.RL2.Trq_T2W          = out->ToCM.WheelOut.RL2.Trq_T2W;
    Vehicle.RL2.Trq_WhlBearing   = out->ToCM.WheelOut.RL2.Trq_WhlBearing;
    Vehicle.RL2.vBelt            = out->ToCM.WheelOut.RL2.vBelt;
    Vehicle.RL2.Trq_BrakeReg     = out->ToCM.WheelOut.RL2.Trq_BrakeReg;
    Vehicle.RL2.Trq_BrakeReg_max = out->ToCM.WheelOut.RL2.Trq_BrakeReg_max;

    Vehicle.RR2.t[0]             = out->ToCM.WheelOut.RR2.t.x;
    Vehicle.RR2.t[1]             = out->ToCM.WheelOut.RR2.t.y;
    Vehicle.RR2.t[2]             = out->ToCM.WheelOut.RR2.t.z;
    Vehicle.RR2.r_zxy[0]         = out->ToCM.WheelOut.RR2.r_zxy.x;
    Vehicle.RR2.r_zxy[1]         = out->ToCM.WheelOut.RR2.r_zxy.y;
    Vehicle.RR2.r_zxy[2]         = out->ToCM.WheelOut.RR2.r_zxy.z;
    Vehicle.RR2.LongSlip         = out->ToCM.WheelOut.RR2.LongSlip;
    Vehicle.RR2.SideSlip         = out->ToCM.WheelOut.RR2.SideSlip;
    Vehicle.RR2.Fx               = out->ToCM.WheelOut.RR2.Fx;
    Vehicle.RR2.Fy               = out->ToCM.WheelOut.RR2.Fy;
    Vehicle.RR2.Fz               = out->ToCM.WheelOut.RR2.Fz;
    Vehicle.RR2.FxTwin           = out->ToCM.WheelOut.RR2.FxTwin;
    Vehicle.RR2.FyTwin           = out->ToCM.WheelOut.RR2.FyTwin;
    Vehicle.RR2.FzTwin           = out->ToCM.WheelOut.RR2.FzTwin;
    Vehicle.RR2.Trq_T2W          = out->ToCM.WheelOut.RR2.Trq_T2W;
    Vehicle.RR2.Trq_WhlBearing   = out->ToCM.WheelOut.RR2.Trq_WhlBearing;
    Vehicle.RR2.vBelt            = out->ToCM.WheelOut.RR2.vBelt;
    Vehicle.RR2.Trq_BrakeReg     = out->ToCM.WheelOut.RR2.Trq_BrakeReg;
    Vehicle.RR2.Trq_BrakeReg_max = out->ToCM.WheelOut.RR2.Trq_BrakeReg_max;

    Vehicle.FL2.t[0]             = out->ToCM.WheelOut.FL2.t.x;
    Vehicle.FL2.t[1]             = out->ToCM.WheelOut.FL2.t.y;
    Vehicle.FL2.t[2]             = out->ToCM.WheelOut.FL2.t.z;
    Vehicle.FL2.r_zxy[0]         = out->ToCM.WheelOut.FL2.r_zxy.x;
    Vehicle.FL2.r_zxy[1]         = out->ToCM.WheelOut.FL2.r_zxy.y;
    Vehicle.FL2.r_zxy[2]         = out->ToCM.WheelOut.FL2.r_zxy.z;
    Vehicle.FL2.LongSlip         = out->ToCM.WheelOut.FL2.LongSlip;
    Vehicle.FL2.SideSlip         = out->ToCM.WheelOut.FL2.SideSlip;
    Vehicle.FL2.Fx               = out->ToCM.WheelOut.FL2.Fx;
    Vehicle.FL2.Fy               = out->ToCM.WheelOut.FL2.Fy;
    Vehicle.FL2.Fz               = out->ToCM.WheelOut.FL2.Fz;
    Vehicle.FL2.FxTwin           = out->ToCM.WheelOut.FL2.FxTwin;
    Vehicle.FL2.FyTwin           = out->ToCM.WheelOut.FL2.FyTwin;
    Vehicle.FL2.FzTwin           = out->ToCM.WheelOut.FL2.FzTwin;
    Vehicle.FL2.Trq_T2W          = out->ToCM.WheelOut.FL2.Trq_T2W;
    Vehicle.FL2.Trq_WhlBearing   = out->ToCM.WheelOut.FL2.Trq_WhlBearing;
    Vehicle.FL2.vBelt            = out->ToCM.WheelOut.FL2.vBelt;
    Vehicle.FL2.Trq_BrakeReg     = out->ToCM.WheelOut.FL2.Trq_BrakeReg;
    Vehicle.FL2.Trq_BrakeReg_max = out->ToCM.WheelOut.FL2.Trq_BrakeReg_max;

    Vehicle.FR2.t[0]             = out->ToCM.WheelOut.FR2.t.x;
    Vehicle.FR2.t[1]             = out->ToCM.WheelOut.FR2.t.y;
    Vehicle.FR2.t[2]             = out->ToCM.WheelOut.FR2.t.z;
    Vehicle.FR2.r_zxy[0]         = out->ToCM.WheelOut.FR2.r_zxy.x;
    Vehicle.FR2.r_zxy[1]         = out->ToCM.WheelOut.FR2.r_zxy.y;
    Vehicle.FR2.r_zxy[2]         = out->ToCM.WheelOut.FR2.r_zxy.z;
    Vehicle.FR2.LongSlip         = out->ToCM.WheelOut.FR2.LongSlip;
    Vehicle.FR2.SideSlip         = out->ToCM.WheelOut.FR2.SideSlip;
    Vehicle.FR2.Fx               = out->ToCM.WheelOut.FR2.Fx;
    Vehicle.FR2.Fy               = out->ToCM.WheelOut.FR2.Fy;
    Vehicle.FR2.Fz               = out->ToCM.WheelOut.FR2.Fz;
    Vehicle.FR2.FxTwin           = out->ToCM.WheelOut.FR2.FxTwin;
    Vehicle.FR2.FyTwin           = out->ToCM.WheelOut.FR2.FyTwin;
    Vehicle.FR2.FzTwin           = out->ToCM.WheelOut.FR2.FzTwin;
    Vehicle.FR2.Trq_T2W          = out->ToCM.WheelOut.FR2.Trq_T2W;
    Vehicle.FR2.Trq_WhlBearing   = out->ToCM.WheelOut.FR2.Trq_WhlBearing;
    Vehicle.FR2.vBelt            = out->ToCM.WheelOut.FR2.vBelt;
    Vehicle.FR2.Trq_BrakeReg     = out->ToCM.WheelOut.FR2.Trq_BrakeReg;
    Vehicle.FR2.Trq_BrakeReg_max = out->ToCM.WheelOut.FR2.Trq_BrakeReg_max;
#endif

    Vehicle.Steering.Ang    = out->ToCM.Steering.Ang;
    Vehicle.Steering.AngVel = out->ToCM.Steering.AngVel;
    Vehicle.Steering.AngAcc = out->ToCM.Steering.AngAcc;
    Vehicle.Steering.Trq    = out->ToCM.Steering.Trq;

    if (BrakeDisabled) {
	Brake.IF.Trq_WB[0] = out->ToCM.Brake.FL.Trq_WB;
	Brake.IF.Trq_WB[1] = out->ToCM.Brake.FR.Trq_WB;
	Brake.IF.Trq_WB[2] = out->ToCM.Brake.RL.Trq_WB;
	Brake.IF.Trq_WB[3] = out->ToCM.Brake.RR.Trq_WB;
#if defined (IS_TRUCK)
	Brake.IF.Trq_WB[4] = out->ToCM.Brake.RL2.Trq_WB;
	Brake.IF.Trq_WB[5] = out->ToCM.Brake.RR2.Trq_WB;
	Brake.IF.Trq_WB[6] = out->ToCM.Brake.FL2.Trq_WB;
	Brake.IF.Trq_WB[7] = out->ToCM.Brake.FR2.Trq_WB;
#endif

	Brake.IF.Trq_PB[0] = out->ToCM.Brake.FL.Trq_PB;
	Brake.IF.Trq_PB[1] = out->ToCM.Brake.FR.Trq_PB;
	Brake.IF.Trq_PB[2] = out->ToCM.Brake.RL.Trq_PB;
	Brake.IF.Trq_PB[3] = out->ToCM.Brake.RR.Trq_PB;
#if defined (IS_TRUCK)
	Brake.IF.Trq_PB[4] = out->ToCM.Brake.RL2.Trq_PB;
	Brake.IF.Trq_PB[5] = out->ToCM.Brake.RR2.Trq_PB;
	Brake.IF.Trq_PB[6] = out->ToCM.Brake.FL2.Trq_PB;
	Brake.IF.Trq_PB[7] = out->ToCM.Brake.FR2.Trq_PB;
#endif

	Brake.IF.Trq_Reg_trg[0] = out->ToCM.Brake.FL.Trq_BrakeReg_trg;
	Brake.IF.Trq_Reg_trg[1] = out->ToCM.Brake.FR.Trq_BrakeReg_trg;
	Brake.IF.Trq_Reg_trg[2] = out->ToCM.Brake.RL.Trq_BrakeReg_trg;
	Brake.IF.Trq_Reg_trg[3] = out->ToCM.Brake.RR.Trq_BrakeReg_trg;
#if defined (IS_TRUCK)
	Brake.IF.Trq_Reg_trg[4] = out->ToCM.Brake.RL2.Trq_BrakeReg_trg;
	Brake.IF.Trq_Reg_trg[5] = out->ToCM.Brake.RR2.Trq_BrakeReg_trg;
	Brake.IF.Trq_Reg_trg[6] = out->ToCM.Brake.FL2.Trq_BrakeReg_trg;
	Brake.IF.Trq_Reg_trg[7] = out->ToCM.Brake.FR2.Trq_BrakeReg_trg;
#endif
    }

    if (PowerTrainDisabled) {
	PowerTrain.IF.Trq_Supp2Bdy1[0] = out->ToCM.PT.Trq_Supp2Bdy1.x;
	PowerTrain.IF.Trq_Supp2Bdy1[1] = out->ToCM.PT.Trq_Supp2Bdy1.y;
	PowerTrain.IF.Trq_Supp2Bdy1[2] = out->ToCM.PT.Trq_Supp2Bdy1.z;
	PowerTrain.IF.Trq_Supp2Bdy1B[0] = out->ToCM.PT.Trq_Supp2Bdy1B.x;
	PowerTrain.IF.Trq_Supp2Bdy1B[1] = out->ToCM.PT.Trq_Supp2Bdy1B.y;
	PowerTrain.IF.Trq_Supp2Bdy1B[2] = out->ToCM.PT.Trq_Supp2Bdy1B.z;
	PowerTrain.IF.Trq_Supp2BdyEng[0] = out->ToCM.PT.Trq_Supp2BdyEng.x;
	PowerTrain.IF.Trq_Supp2BdyEng[1] = out->ToCM.PT.Trq_Supp2BdyEng.y;

	PowerTrain.IF.Ignition       = out->ToCM.PT.Ignition != 0;
	PowerTrain.IF.OperationState = out->ToCM.PT.OperationState;
	PowerTrain.IF.OperationError = out->ToCM.PT.OperationError;
	PowerTrain.IF.Engine_rotv    = out->ToCM.PT.Engine_rotv;
	PowerTrain.IF.GearNo         = out->ToCM.PT.GearBox_GearNo;
	PowerTrain.IF.DL_iDiff_mean  = out->ToCM.PT.DL_iDiff_mean;

	PowerTrain.IF.WheelOut[0].Trq_B2W	= out->ToCM.PT.Wheel.FL.Trq_B2W;
	PowerTrain.IF.WheelOut[0].rot		= out->ToCM.PT.Wheel.FL.rot;
	PowerTrain.IF.WheelOut[0].rotv		= out->ToCM.PT.Wheel.FL.rotv;
	PowerTrain.IF.WheelOut[0].Trq_Supp2WC      = out->ToCM.PT.Wheel.FL.Trq_Supp2WC;
	PowerTrain.IF.WheelOut[0].Trq_BrakeReg     = out->ToCM.PT.Wheel.FL.Trq_BrakeReg;
	PowerTrain.IF.WheelOut[0].Trq_BrakeReg_max = out->ToCM.PT.Wheel.FL.Trq_BrakeReg_max;

	PowerTrain.IF.WheelOut[1].Trq_B2W	= out->ToCM.PT.Wheel.FR.Trq_B2W;
	PowerTrain.IF.WheelOut[1].rot		= out->ToCM.PT.Wheel.FR.rot;
	PowerTrain.IF.WheelOut[1].rotv		= out->ToCM.PT.Wheel.FR.rotv;
	PowerTrain.IF.WheelOut[1].Trq_Supp2WC      = out->ToCM.PT.Wheel.FR.Trq_Supp2WC;
	PowerTrain.IF.WheelOut[1].Trq_BrakeReg     = out->ToCM.PT.Wheel.FR.Trq_BrakeReg;
	PowerTrain.IF.WheelOut[1].Trq_BrakeReg_max = out->ToCM.PT.Wheel.FR.Trq_BrakeReg_max;

	PowerTrain.IF.WheelOut[2].Trq_B2W	= out->ToCM.PT.Wheel.RL.Trq_B2W;
	PowerTrain.IF.WheelOut[2].rot		= out->ToCM.PT.Wheel.RL.rot;
	PowerTrain.IF.WheelOut[2].rotv		= out->ToCM.PT.Wheel.RL.rotv;
	PowerTrain.IF.WheelOut[2].Trq_Supp2WC      = out->ToCM.PT.Wheel.RL.Trq_Supp2WC;
	PowerTrain.IF.WheelOut[2].Trq_BrakeReg     = out->ToCM.PT.Wheel.RL.Trq_BrakeReg;
	PowerTrain.IF.WheelOut[2].Trq_BrakeReg_max = out->ToCM.PT.Wheel.RL.Trq_BrakeReg_max;

	PowerTrain.IF.WheelOut[3].Trq_B2W	= out->ToCM.PT.Wheel.RR.Trq_B2W;
	PowerTrain.IF.WheelOut[3].rot		= out->ToCM.PT.Wheel.RR.rot;
	PowerTrain.IF.WheelOut[3].rotv		= out->ToCM.PT.Wheel.RR.rotv;
	PowerTrain.IF.WheelOut[3].Trq_Supp2WC      = out->ToCM.PT.Wheel.RR.Trq_Supp2WC;
	PowerTrain.IF.WheelOut[3].Trq_BrakeReg     = out->ToCM.PT.Wheel.RR.Trq_BrakeReg;
	PowerTrain.IF.WheelOut[3].Trq_BrakeReg_max = out->ToCM.PT.Wheel.RR.Trq_BrakeReg_max;

#if defined (IS_TRUCK)
	PowerTrain.IF.WheelOut[4].Trq_B2W	= out->ToCM.PT.Wheel.RL2.Trq_B2W;
	PowerTrain.IF.WheelOut[4].rot		= out->ToCM.PT.Wheel.RL2.rot;
	PowerTrain.IF.WheelOut[4].rotv		= out->ToCM.PT.Wheel.RL2.rotv;
	PowerTrain.IF.WheelOut[4].Trq_Supp2WC      = out->ToCM.PT.Wheel.RL2.Trq_Supp2WC;
	PowerTrain.IF.WheelOut[4].Trq_BrakeReg     = out->ToCM.PT.Wheel.RL2.Trq_BrakeReg;
	PowerTrain.IF.WheelOut[4].Trq_BrakeReg_max = out->ToCM.PT.Wheel.RL2.Trq_BrakeReg_max;

	PowerTrain.IF.WheelOut[5].Trq_B2W	= out->ToCM.PT.Wheel.RR2.Trq_B2W;
	PowerTrain.IF.WheelOut[5].rot		= out->ToCM.PT.Wheel.RR2.rot;
	PowerTrain.IF.WheelOut[5].rotv		= out->ToCM.PT.Wheel.RR2.rotv;
	PowerTrain.IF.WheelOut[5].Trq_Supp2WC      = out->ToCM.PT.Wheel.RR2.Trq_Supp2WC;
	PowerTrain.IF.WheelOut[5].Trq_BrakeReg     = out->ToCM.PT.Wheel.RR2.Trq_BrakeReg;
	PowerTrain.IF.WheelOut[5].Trq_BrakeReg_max = out->ToCM.PT.Wheel.RR2.Trq_BrakeReg_max;

	PowerTrain.IF.WheelOut[6].Trq_B2W	= out->ToCM.PT.Wheel.FL2.Trq_B2W;
	PowerTrain.IF.WheelOut[6].rot		= out->ToCM.PT.Wheel.FL2.rot;
	PowerTrain.IF.WheelOut[6].rotv		= out->ToCM.PT.Wheel.FL2.rotv;
	PowerTrain.IF.WheelOut[6].Trq_Supp2WC      = out->ToCM.PT.Wheel.FL2.Trq_Supp2WC;
	PowerTrain.IF.WheelOut[6].Trq_BrakeReg     = out->ToCM.PT.Wheel.FL2.Trq_BrakeReg;
	PowerTrain.IF.WheelOut[6].Trq_BrakeReg_max = out->ToCM.PT.Wheel.FL2.Trq_BrakeReg_max;

	PowerTrain.IF.WheelOut[7].Trq_B2W	= out->ToCM.PT.Wheel.FR2.Trq_B2W;
	PowerTrain.IF.WheelOut[7].rot		= out->ToCM.PT.Wheel.FR2.rot;
	PowerTrain.IF.WheelOut[7].rotv		= out->ToCM.PT.Wheel.FR2.rotv;
	PowerTrain.IF.WheelOut[7].Trq_Supp2WC      = out->ToCM.PT.Wheel.FR2.Trq_Supp2WC;
	PowerTrain.IF.WheelOut[7].Trq_BrakeReg     = out->ToCM.PT.Wheel.FR2.Trq_BrakeReg;
	PowerTrain.IF.WheelOut[7].Trq_BrakeReg_max = out->ToCM.PT.Wheel.FR2.Trq_BrakeReg_max;
#endif
    }
}


static int
VehicleModel_Calc (void *MP, double dt)
{
    rtModel_SingleTrack_RTW *rtm = (rtModel_SingleTrack_RTW *)MP;
    ExternalInputs_SingleTrack_RTW  *in  = (ExternalInputs_SingleTrack_RTW *) rtmGetU(rtm);
    ExternalOutputs_SingleTrack_RTW *out = (ExternalOutputs_SingleTrack_RTW *)rtmGetY(rtm);
    int ret;

    if (SkipPreprocessing) {
	if (SCState_Start<=SimCore.State && SimCore.State<=SCState_StartLastCycle)
	    return 0;
    }

    CM2VehicleModel(in);

    ret = SingleTrack_RTW_Calc(MP, dt);

    VehicleModel2CM(out);

    return ret;
}


int
SingleTrack_RTW_Register (void)
{
    tModelClassDescr m;

    /*Log("%s_Register()\n", Modelname);*/

    memset(&m, 0, sizeof(m));

    /* Parameter file identification number.
       You may change CompatVersionId to the the lowest parameter
       file version understood by your model code. */
    m.Vehicle.VersionId		= PARAMID;
    m.Vehicle.CompatVersionId	= m.Vehicle.VersionId;

    m.Vehicle.DeclQuants	= SingleTrack_RTW_DeclQuants;
    m.Vehicle.New		= VehicleModel_New;
    m.Vehicle.Calc		= VehicleModel_Calc;
    m.Vehicle.Delete		= SingleTrack_RTW_Delete;
    /* Should only be used if the model doesn't read params from extra files */
    // m.Vehicle.ParamsChanged	= ParamsChanged_IgnoreCheck;

    return Model_Register(Modelclass, Modelname, &m);
}

