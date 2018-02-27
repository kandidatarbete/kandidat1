/***************************************************** target specific file ***/
/*  CarMaker wrapper module for Simulink models                               */
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
#include "ModelManager.h"
#include "LinMap.h"
#include "MathUtils.h"

#include "Vehicle/MBSUtils.h"

#if defined (IS_CAR) || defined (IS_TRUCK)
# include "Car/Car.h"
# include "Car/Brake.h"
#elif defined(IS_MCYCLE)
# include "MCycle/MCycle.h"
# include "MCycle/Brake.h"
#endif

#include "ESP_RTW.h"
#include "ESP_RTW_wrap.h"


#define QUOTE1(name)	#name
#define QUOTE(name)	QUOTE1(name)		/* need to expand name */

extern const char ESP_RTW_LibIdent[];
const char ESP_RTW_LibIdent[] = "(@@)" QUOTE(MODEL) " " ARCH " 1.0 " BUILDDATE;

static const char Modelname[] = QUOTE(MODEL);
static const tModelClass Modelclass = ModelClass_HydBrakeControl;
static tMatSuppSampling SampleParams;


static void
DoOneStep (rtModel_ESP_RTW *rtm)
{
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
ESP_RTW_DeclQuants (void *MP)
{
    rtModel_ESP_RTW *rtm = (rtModel_ESP_RTW *)MP;
    int i;

    /*Log("%s_DeclQuants()\n", Modelname);*/

    if (rtm == NULL) {
	/* Remember body frames defined in this model for later registration. */
        MdlBdyFrame_Add(ESP_RTW_BdyFrameDefines);

	/* Define dict entries for non-dynamically allocated variables. */
	if ((i = MatSupp_DeclQuants(ESP_RTW_DictDefines)) >= 0) {
	    LogErrF(EC_Init, "Model '%s': Error defining quantity '%s'\n",
		    Modelname, ESP_RTW_DictDefines[i].Name);
	}

	/* Define dict entries for tunable parameters (with dummy address). */
	DeclParameterQuants(NULL);
    } else {
	/* Define dict entries for dynamically allocated variables. */
    }
}


/*
 * ESP_RTW_SetParams() will be invoked indirectly by the generated
 * model C code each time ESP_RTW_New() is called.
 */
void
ESP_RTW_SetParams (rtModel_ESP_RTW *rtm, struct tMatSuppTunables *tuns,
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
CfgLinMaptoBus (tLM *CfgInput, int arraydim, char* key, double *BusInputx, double *BusInputz)
{
    int xdim, i;
    tLM *lm;

    if (CfgInput != NULL) {
	lm = LMCopy(CfgInput);
	xdim = LMXGetDim(CfgInput);
	
	if (xdim > arraydim) {
	    Log("Model '%s': %s mapping will be resized to %i points.\n", Modelname, key, arraydim);
	    LMDelete(lm);
	    lm = LMResizeAequi(CfgInput, arraydim);
	    xdim = LMXGetDim(lm);
	}
	
	for (i=0; i<xdim; i++) {
	    BusInputx[i] = LMXGetVal(lm, i);
	    BusInputz[i] = LMEval(lm, BusInputx[i]);
	}
	for (i=xdim; i<arraydim; i++) {
	    BusInputx[i] = BusInputx[xdim-1];
	    BusInputz[i] = BusInputz[xdim-1];
	}
	       LMDelete(lm);
    }
}


static void *
assignCfgIF (struct tInfos *Inf, struct tHydBrakeCU_CfgIF *CfgIF,
             const char *KindKey, void *MP)
{
    rtModel_ESP_RTW *rtm = (rtModel_ESP_RTW *)MP;
    int arraydim, count = GetInfoErrorCount();
    ExternalInputs_ESP_RTW  *in  = (ExternalInputs_ESP_RTW *) rtmGetU(rtm);

    /* CfgIF Input */
    in->CfgInFromCM.VehicleClassId	= CfgIF->VhclClassId;
    in->CfgInFromCM.nWheels		= CfgIF->nWheels;
    in->CfgInFromCM.BrakeKind		= CfgIF->Kind;

#if defined (IS_CAR) || defined (IS_TRUCK)
    arraydim = sizeof(in->CfgInFromCM.Trq_stat.FL.x) / sizeof(in->CfgInFromCM.Trq_stat.FL.x[0]);
    CfgLinMaptoBus (CfgIF->Trq_stat[0], arraydim, "Brake FL: Static brake torque",
    		    in->CfgInFromCM.Trq_stat.FL.x, in->CfgInFromCM.Trq_stat.FL.z);

    arraydim = sizeof(in->CfgInFromCM.Trq_stat.FR.x) / sizeof(in->CfgInFromCM.Trq_stat.FR.x[0]);
    CfgLinMaptoBus (CfgIF->Trq_stat[1], arraydim, "Brake FR: Static brake torque",
    		    in->CfgInFromCM.Trq_stat.FR.x, in->CfgInFromCM.Trq_stat.FR.z);

    arraydim = sizeof(in->CfgInFromCM.Trq_stat.RL.x) / sizeof(in->CfgInFromCM.Trq_stat.RL.x[0]);
    CfgLinMaptoBus (CfgIF->Trq_stat[2], arraydim, "Brake RL: Static brake torque",
    		    in->CfgInFromCM.Trq_stat.RL.x, in->CfgInFromCM.Trq_stat.RL.z);

    arraydim = sizeof(in->CfgInFromCM.Trq_stat.RR.x) / sizeof(in->CfgInFromCM.Trq_stat.RR.x[0]);
    CfgLinMaptoBus (CfgIF->Trq_stat[3], arraydim, "Brake RR: Static brake torque",
    		    in->CfgInFromCM.Trq_stat.RR.x, in->CfgInFromCM.Trq_stat.RR.z);
#endif
#if defined (IS_TRUCK)
    arraydim = sizeof(in->CfgInFromCM.Trq_stat.RL2.x) / sizeof(in->CfgInFromCM.Trq_stat.RL2.x[0]);
    CfgLinMaptoBus (CfgIF->Trq_stat[4], arraydim, "Brake RL2: Static brake torque",
    		    in->CfgInFromCM.Trq_stat.RL2.x, in->CfgInFromCM.Trq_stat.RL2.z);

    arraydim = sizeof(in->CfgInFromCM.Trq_stat.RR2.x) / sizeof(in->CfgInFromCM.Trq_stat.RR2.x[0]);
    CfgLinMaptoBus (CfgIF->Trq_stat[5], arraydim, "Brake RR2: Static brake torque",
    		    in->CfgInFromCM.Trq_stat.RR2.x, in->CfgInFromCM.Trq_stat.RR2.z);

    arraydim = sizeof(in->CfgInFromCM.Trq_stat.FL2.x) / sizeof(in->CfgInFromCM.Trq_stat.FL2.x[0]);
    CfgLinMaptoBus (CfgIF->Trq_stat[6], arraydim, "Brake FL2: Static brake torque",
    		    in->CfgInFromCM.Trq_stat.FL2.x, in->CfgInFromCM.Trq_stat.FL2.z);

    arraydim = sizeof(in->CfgInFromCM.Trq_stat.FR2.x) / sizeof(in->CfgInFromCM.Trq_stat.FR2.x[0]);
    CfgLinMaptoBus (CfgIF->Trq_stat[7], arraydim, "Brake FR2: Static brake torque",
    		    in->CfgInFromCM.Trq_stat.FR2.x, in->CfgInFromCM.Trq_stat.FR2.z);
#endif

    if (GetInfoErrorCount() != count) {
    	goto ErrorReturn;
    }

    return rtm;

  ErrorReturn:
    return NULL;
}


static void *
ESP_RTW_New (struct tInfos *Inf, struct tHydBrakeCU_CfgIF *CfgIF, const char *KindKey)
{
    rtModel_ESP_RTW *rtm;
    double rtmSampleTime;
    const char *ModelKind;
    int VersionId = 0;

    if ((ModelKind = SimCore_GetKindInfo(Inf, Modelclass, KindKey,
	 				 0, PARAMID, &VersionId)) == NULL)
	return NULL;

    /*Log("%s_New()\n", Modelname);*/

    MatSupp_ResetQuants(ESP_RTW_DictDefines);

    rtm = MODEL(Inf);

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

    rtmSampleTime = (double) rtmGetStepSize(rtm);
    if (MatSupp_Sampling (&SampleParams, SimCore.DeltaT, rtmSampleTime) !=0) {
	LogErrF(EC_Init, "Model '%s': The sample times of the plugin model and the application have to be integer multiples\n",Modelname);
	return NULL;
    }

    /* assign CfgIF struct */
    if ((assignCfgIF(Inf, CfgIF, KindKey, rtm)) == NULL)  {
	LogErrF(EC_Init, "Model '%s': failed to assign CfgIF\n",Modelname);
	return NULL;
    };

    rtmiStart(rtmGetRTWRTModelMethodsInfo(rtm));

    return rtm; /* Will be passed as MP to the other functions. */
}


static void
ESP_RTW_Delete (void *MP)
{
    rtModel_ESP_RTW *rtm = (rtModel_ESP_RTW *)MP;

    /*Log("%s_Delete()\n", Modelname);*/

    rt_SimDestroyTimingEngine(rtmGetTimingData(rtm));
    if (rtmGetNumContStates(rtm) > 0)
	rt_ODEDestroyIntegrationData(rtmGetRTWSolverInfo(rtm));
    rtmiTerminate(rtmGetRTWRTModelMethodsInfo(rtm));
}


static int
ESP_RTW_Calc (void *MP, tHydBrakeCU_IF *IF, double dt)
{
    rtModel_ESP_RTW *rtm = (rtModel_ESP_RTW *)MP;
    int osCount=0;
    ExternalInputs_ESP_RTW  *in  = (ExternalInputs_ESP_RTW *) rtmGetU(rtm);
    ExternalOutputs_ESP_RTW *out = (ExternalOutputs_ESP_RTW *)rtmGetY(rtm);


    in->FromCM.Pedal = IF->Pedal;
    in->FromCM.Park  = IF->Park;
    in->FromCM.T_env = IF->T_env;

#if defined (IS_CAR) || defined (IS_TRUCK)
    in->FromCM.Trq_BrakeReg_max.FL = IF->Trq_Reg_max[0];
    in->FromCM.Trq_BrakeReg.FL     = IF->Trq_Reg[0];
    in->FromCM.Trq_WB.FL           = IF->Trq_WB[0];
    in->FromCM.Trq_PB.FL           = IF->Trq_PB[0];
    in->FromCM.Trq_BrakeReg_max.FR = IF->Trq_Reg_max[1];
    in->FromCM.Trq_BrakeReg.FR     = IF->Trq_Reg[1];
    in->FromCM.Trq_WB.FR           = IF->Trq_WB[1];
    in->FromCM.Trq_PB.FR           = IF->Trq_PB[1];
    in->FromCM.Trq_BrakeReg_max.RL = IF->Trq_Reg_max[2];
    in->FromCM.Trq_BrakeReg.RL     = IF->Trq_Reg[2];
    in->FromCM.Trq_WB.RL           = IF->Trq_WB[2];
    in->FromCM.Trq_PB.RL           = IF->Trq_PB[2];
    in->FromCM.Trq_BrakeReg_max.RR = IF->Trq_Reg_max[3];
    in->FromCM.Trq_BrakeReg.RR     = IF->Trq_Reg[3];
    in->FromCM.Trq_WB.RR           = IF->Trq_WB[3];
    in->FromCM.Trq_PB.RR           = IF->Trq_PB[3];

    in->FromCM.pWB.FL  = IF->pWB[0];
    in->FromCM.pWB.FR  = IF->pWB[1];
    in->FromCM.pWB.RL  = IF->pWB[2];
    in->FromCM.pWB.RR  = IF->pWB[3];
#endif
#if defined (IS_TRUCK)
    in->FromCM.Trq_BrakeReg_max.RL2 = IF->Trq_Reg_max[4];
    in->FromCM.Trq_BrakeReg.RL2     = IF->Trq_Reg[4];
    in->FromCM.Trq_WB.RL2           = IF->Trq_WB[4];
    in->FromCM.Trq_PB.RL2           = IF->Trq_PB[4];
    in->FromCM.Trq_BrakeReg_max.RR2 = IF->Trq_Reg_max[5];
    in->FromCM.Trq_BrakeReg.RR2     = IF->Trq_Reg[5];
    in->FromCM.Trq_WB.RR2           = IF->Trq_WB[5];
    in->FromCM.Trq_PB.RR2           = IF->Trq_PB[5];
    in->FromCM.Trq_BrakeReg_max.FL2 = IF->Trq_Reg_max[6];
    in->FromCM.Trq_BrakeReg.FL2     = IF->Trq_Reg[6];
    in->FromCM.Trq_WB.FL2           = IF->Trq_WB[6];
    in->FromCM.Trq_PB.FL2           = IF->Trq_PB[6];
    in->FromCM.Trq_BrakeReg_max.FR2 = IF->Trq_Reg_max[7];
    in->FromCM.Trq_BrakeReg.FR2     = IF->Trq_Reg[7];
    in->FromCM.Trq_WB.FR2           = IF->Trq_WB[7];
    in->FromCM.Trq_PB.FR2           = IF->Trq_PB[7];

    in->FromCM.pWB.RL2 = IF->pWB[4];
    in->FromCM.pWB.RR2 = IF->pWB[5];
    in->FromCM.pWB.FL2 = IF->pWB[6];
    in->FromCM.pWB.FR2 = IF->pWB[7];
#endif

    in->FromCM.Rel_SW  = IF->Rel_SW;
    in->FromCM.pMC     = IF->pMC;
    in->FromCM.PuRetVolt   = IF->PuRetVolt;
    in->FromCM.PedFrc      = IF->PedFrc;
    in->FromCM.PedTravel   = IF->PedTravel;
    in->FromCM.PistTravel  = IF->PistTravel;
    in->FromCM.DiaphTravel = IF->DiaphTravel;

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

    IF->Pedal = out->ToCM.Pedal;
    IF->Park  = out->ToCM.Park;

#if defined (IS_CAR) || defined (IS_TRUCK)
    IF->Trq_Reg_trg[0] = out->ToCM.Trq_BrakeReg_trg.FL;
    IF->Trq_Reg_trg[1] = out->ToCM.Trq_BrakeReg_trg.FR;
    IF->Trq_Reg_trg[2] = out->ToCM.Trq_BrakeReg_trg.RL;
    IF->Trq_Reg_trg[3] = out->ToCM.Trq_BrakeReg_trg.RR;
#endif
#if defined (IS_TRUCK)
    IF->Trq_Reg_trg[4] = out->ToCM.Trq_BrakeReg_trg.RL2;
    IF->Trq_Reg_trg[5] = out->ToCM.Trq_BrakeReg_trg.RR2;
    IF->Trq_Reg_trg[6] = out->ToCM.Trq_BrakeReg_trg.FL2;
    IF->Trq_Reg_trg[7] = out->ToCM.Trq_BrakeReg_trg.FR2;
#endif

    IF->V[0]  = out->ToCM.Valve.FL_Inlet;
    IF->V[1]  = out->ToCM.Valve.FR_Inlet;
    IF->V[2]  = out->ToCM.Valve.RL_Inlet;
    IF->V[3]  = out->ToCM.Valve.RR_Inlet;
    IF->V[4]  = out->ToCM.Valve.FL_Outlet;
    IF->V[5]  = out->ToCM.Valve.FR_Outlet;
    IF->V[6]  = out->ToCM.Valve.RL_Outlet;
    IF->V[7]  = out->ToCM.Valve.RR_Outlet;
    IF->V[8]  = out->ToCM.Valve.PV_1;
    IF->V[9]  = out->ToCM.Valve.PV_2;
    IF->V[10] = out->ToCM.Valve.SV_1;
    IF->V[11] = out->ToCM.Valve.SV_2;
    IF->V[12] = out->ToCM.Valve.V_1;
    IF->V[13] = out->ToCM.Valve.V_2;
    IF->V[14] = out->ToCM.Valve.V_3;
    IF->V[15] = out->ToCM.Valve.V_4;

    IF->PumpCtrl  = out->ToCM.PumpCtrl;
    IF->BooSignal = out->ToCM.BooSignal;


    return 0;
}


int
ESP_RTW_Register (void)
{
    tModelClassDescr m;

    /*Log("%s_Register()\n", Modelname);*/

    memset(&m, 0, sizeof(m));

    /* Parameter file identification number.
       You may change CompatVersionId to the the lowest parameter
       file version understood by your model code. */
    m.HydBrakeControl.VersionId		= PARAMID;
    m.HydBrakeControl.CompatVersionId	= m.Brake.VersionId;

    m.HydBrakeControl.DeclQuants	= ESP_RTW_DeclQuants;
    m.HydBrakeControl.New		= ESP_RTW_New;
    m.HydBrakeControl.Calc		= ESP_RTW_Calc;
    m.HydBrakeControl.Delete		= ESP_RTW_Delete;
    /* Should only be used if the model doesn't read params from extra files */
    // m.HydBrakeControl.ParamsChanged	= ParamsChanged_IgnoreCheck;

    return Model_Register(Modelclass, Modelname, &m);
}

