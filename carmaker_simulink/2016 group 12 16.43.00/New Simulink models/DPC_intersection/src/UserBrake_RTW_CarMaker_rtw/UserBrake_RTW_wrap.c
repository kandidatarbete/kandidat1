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

#include "infoc.h"
#include "DataDict.h"
#include "SimCore.h"
#include "Log.h"
#include "MatSupp.h"
#include "ModelManager.h"

#include "Vehicle.h"
#include "Vehicle/MBSUtils.h"

#if defined (IS_CAR) || defined (IS_TRUCK)
# include "Car/Car.h"
# include "Car/Brake.h"
#elif defined(IS_MCYCLE)
# include "MCycle/MCycle.h"
# include "MCycle/Brake.h"
#endif

#include "UserBrake_RTW.h"
#include "UserBrake_RTW_wrap.h"


#define QUOTE1(name)	#name
#define QUOTE(name)	QUOTE1(name)		/* need to expand name */

extern const char UserBrake_RTW_LibIdent[];
const char UserBrake_RTW_LibIdent[] = "(@@)" QUOTE(MODEL) " " ARCH " 1.0 " BUILDDATE;

static const char Modelname[] = QUOTE(MODEL);
static const tModelClass Modelclass = ModelClass_Brake;
static tMatSuppSampling SampleParams;


static void
DoOneStep (rtModel_UserBrake_RTW *rtm)
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
UserBrake_RTW_DeclQuants (void *MP)
{
    rtModel_UserBrake_RTW *rtm = (rtModel_UserBrake_RTW *)MP;
    int i;

    /*Log("%s_DeclQuants()\n", Modelname);*/

    if (rtm == NULL) {
	/* Remember body frames defined in this model for later registration. */
        MdlBdyFrame_Add(UserBrake_RTW_BdyFrameDefines);

	/* Define dict entries for non-dynamically allocated variables. */
	if ((i = MatSupp_DeclQuants(UserBrake_RTW_DictDefines)) >= 0) {
	    LogErrF(EC_Init, "Model '%s': Error defining quantity '%s'\n",
		    Modelname, UserBrake_RTW_DictDefines[i].Name);
	}

	/* Define dict entries for tunable parameters (with dummy address). */
	DeclParameterQuants(NULL);
    } else {
	/* Define dict entries for dynamically allocated variables. */
    }
}


/*
 * UserBrake_RTW_SetParams() will be invoked indirectly by the generated
 * model C code each time UserBrake_RTW_New() is called.
 */
void
UserBrake_RTW_SetParams (rtModel_UserBrake_RTW *rtm, struct tMatSuppTunables *tuns,
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


static void *
assignCfgIF (struct tInfos *Inf, struct tBrakeCfgIF *CfgIF,
             const char *KindKey, void *MP)
{
    rtModel_UserBrake_RTW *rtm = (rtModel_UserBrake_RTW *)MP;
    ExternalInputs_UserBrake_RTW  *in  = (ExternalInputs_UserBrake_RTW *) rtmGetU(rtm);

    /* CfgIF Input */
#if defined (IS_CAR) || defined (IS_TRUCK)
    in->CfgInFromCM.VehicleClassId	= CfgIF->VhclClassId;
#elif defined (IS_MCYCLE)
    in->CfgInFromCM.VehicleClassId	= CfgIF->vhclClassId;
#endif

    in->CfgInFromCM.nWheels		= CfgIF->nWheels;

    return rtm;
}


static void *
UserBrake_RTW_New (struct tInfos *Inf, tBrakeCfgIF *CfgIF, const char *KindKey)
{
    rtModel_UserBrake_RTW *rtm;
    double rtmSampleTime;
    const char *ModelKind;
    int VersionId = 0;

    if ((ModelKind = SimCore_GetKindInfo(Inf, Modelclass, KindKey,
	 				 0, PARAMID, &VersionId)) == NULL)
	return NULL;

    /*Log("%s_New()\n", Modelname);*/

    MatSupp_ResetQuants(UserBrake_RTW_DictDefines);

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
UserBrake_RTW_Delete (void *MP)
{
    rtModel_UserBrake_RTW *rtm = (rtModel_UserBrake_RTW *)MP;

    /*Log("%s_Delete()\n", Modelname);*/

    rt_SimDestroyTimingEngine(rtmGetTimingData(rtm));
    if (rtmGetNumContStates(rtm) > 0)
	rt_ODEDestroyIntegrationData(rtmGetRTWSolverInfo(rtm));
    rtmiTerminate(rtmGetRTWRTModelMethodsInfo(rtm));
}


static int
UserBrake_RTW_Calc (void *MP, tBrakeIF *IF, double dt)
{
    rtModel_UserBrake_RTW *rtm = (rtModel_UserBrake_RTW *)MP;
    int osCount=0;
    ExternalInputs_UserBrake_RTW  *in  = (ExternalInputs_UserBrake_RTW *) rtmGetU(rtm);
    ExternalOutputs_UserBrake_RTW *out = (ExternalOutputs_UserBrake_RTW *)rtmGetY(rtm);

    in->FromCM.Pedal        = IF->Pedal;
    in->FromCM.Park         = IF->Park;
    in->FromCM.T_env        = IF->T_env;

#if defined (IS_MCYCLE)
    in->FromCM.Use_pMCPedalInput	= IF->Use_pMCPedalInput;
    in->FromCM.Use_pMCLeverInput	= IF->Use_pMCLeverInput;
    in->FromCM.Lever	 		= IF->Lever;
    in->FromCM.pMCPedal_in 		= IF->pMCPedal_in;
    in->FromCM.pMCLever_in 		= IF->pMCLever_in;
    in->FromCM.PumpCtrl	 		= IF->PumpCtrl;
#elif defined (IS_CAR) || defined (IS_TRUCK)
    in->FromCM.WheelIn.FL.Trq_BrakeReg_max	 = IF->Trq_Reg_max[0];
    in->FromCM.WheelIn.FL.Trq_BrakeReg		 = IF->Trq_Reg[0];
    in->FromCM.WheelIn.FR.Trq_BrakeReg_max	 = IF->Trq_Reg_max[1];
    in->FromCM.WheelIn.FR.Trq_BrakeReg		 = IF->Trq_Reg[1];
    in->FromCM.WheelIn.RL.Trq_BrakeReg_max	 = IF->Trq_Reg_max[2];
    in->FromCM.WheelIn.RL.Trq_BrakeReg		 = IF->Trq_Reg[2];
    in->FromCM.WheelIn.RR.Trq_BrakeReg_max	 = IF->Trq_Reg_max[3];
    in->FromCM.WheelIn.RR.Trq_BrakeReg		 = IF->Trq_Reg[3];
#endif
#if defined (IS_TRUCK)
    in->FromCM.WheelIn.RL2.Trq_BrakeReg_max	 = IF->Trq_Reg_max[4];
    in->FromCM.WheelIn.RL2.Trq_BrakeReg		 = IF->Trq_Reg[4];
    in->FromCM.WheelIn.RR2.Trq_BrakeReg_max	 = IF->Trq_Reg_max[5];
    in->FromCM.WheelIn.RR2.Trq_BrakeReg		 = IF->Trq_Reg[5];
    in->FromCM.WheelIn.FL2.Trq_BrakeReg_max	 = IF->Trq_Reg_max[6];
    in->FromCM.WheelIn.FL2.Trq_BrakeReg		 = IF->Trq_Reg[6];
    in->FromCM.WheelIn.FR2.Trq_BrakeReg_max	 = IF->Trq_Reg_max[7];
    in->FromCM.WheelIn.FR2.Trq_BrakeReg		 = IF->Trq_Reg[7];
#endif

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

#if defined (IS_MCYCLE)
    IF->Trq_WB[0]  	= out->ToCM.WheelOut.F.Trq_WB;
    IF->Trq_PB[0]  	= out->ToCM.WheelOut.F.Trq_PB;
    IF->Trq_ext[0]  	= out->ToCM.WheelOut.F.Trq_ext;
    IF->Trq_WB[1]  	= out->ToCM.WheelOut.R.Trq_WB;
    IF->Trq_PB[1]  	= out->ToCM.WheelOut.R.Trq_PB;
    IF->Trq_ext[1]  	= out->ToCM.WheelOut.R.Trq_ext;

    IF->pMCPedal  	= out->ToCM.pMCPedal;
    IF->pMCLever  	= out->ToCM.pMCLever;

    IF->pWB[0]  	= out->ToCM.pWB.FL_l0;
    IF->Trq[0]  	= out->ToCM.Trq.FL_l0;
    IF->pWB[1]  	= out->ToCM.pWB.FL_l1;
    IF->Trq[1]  	= out->ToCM.Trq.FL_l1;
    IF->pWB[2]  	= out->ToCM.pWB.FL_p0;
    IF->Trq[2]  	= out->ToCM.Trq.FL_p0;
    IF->pWB[3]  	= out->ToCM.pWB.FR_l0;
    IF->Trq[3]  	= out->ToCM.Trq.FR_l0;
    IF->pWB[4]  	= out->ToCM.pWB.FR_l1;
    IF->Trq[4]  	= out->ToCM.Trq.FR_l1;
    IF->pWB[5]  	= out->ToCM.pWB.FR_p0;
    IF->Trq[5]  	= out->ToCM.Trq.FR_p0;
    IF->pWB[6]  	= out->ToCM.pWB.R_l0;
    IF->Trq[6]  	= out->ToCM.Trq.R_l0;
    IF->pWB[7]  	= out->ToCM.pWB.R_l1;
    IF->Trq[7]  	= out->ToCM.Trq.R_l1;
    IF->pWB[8]  	= out->ToCM.pWB.R_p0;
    IF->Trq[8]  	= out->ToCM.Trq.R_p0;
#elif defined (IS_CAR) || defined (IS_TRUCK)
    IF->Trq_WB[0]  	= out->ToCM.WheelOut.FL.Trq_WB;
    IF->Trq_PB[0]  	= out->ToCM.WheelOut.FL.Trq_PB;
    IF->Trq_Reg_trg[0] 	= out->ToCM.WheelOut.FL.Trq_BrakeReg_trg;
    IF->Trq_WB[1]  	= out->ToCM.WheelOut.FR.Trq_WB;
    IF->Trq_PB[1]  	= out->ToCM.WheelOut.FR.Trq_PB;
    IF->Trq_Reg_trg[1] 	= out->ToCM.WheelOut.FR.Trq_BrakeReg_trg;
    IF->Trq_WB[2]  	= out->ToCM.WheelOut.RL.Trq_WB;
    IF->Trq_PB[2]  	= out->ToCM.WheelOut.RL.Trq_PB;
    IF->Trq_Reg_trg[2]	= out->ToCM.WheelOut.RL.Trq_BrakeReg_trg;
    IF->Trq_WB[3]  	= out->ToCM.WheelOut.RR.Trq_WB;
    IF->Trq_PB[3]  	= out->ToCM.WheelOut.RR.Trq_PB;
    IF->Trq_Reg_trg[3] 	= out->ToCM.WheelOut.RR.Trq_BrakeReg_trg;
#endif
#if defined (IS_TRUCK)
    IF->Trq_WB[4]  	= out->ToCM.WheelOut.RL2.Trq_WB;
    IF->Trq_PB[4]  	= out->ToCM.WheelOut.RL2.Trq_PB;
    IF->Trq_Reg_trg[4] 	= out->ToCM.WheelOut.RL2.Trq_BrakeReg_trg;
    IF->Trq_WB[5]  	= out->ToCM.WheelOut.RR2.Trq_WB;
    IF->Trq_PB[5]  	= out->ToCM.WheelOut.RR2.Trq_PB;
    IF->Trq_Reg_trg[5] 	= out->ToCM.WheelOut.RR2.Trq_BrakeReg_trg;
    IF->Trq_WB[6]  	= out->ToCM.WheelOut.FL2.Trq_WB;
    IF->Trq_PB[6]  	= out->ToCM.WheelOut.FL2.Trq_PB;
    IF->Trq_Reg_trg[6] 	= out->ToCM.WheelOut.FL2.Trq_BrakeReg_trg;
    IF->Trq_WB[7]  	= out->ToCM.WheelOut.FR2.Trq_WB;
    IF->Trq_PB[7]  	= out->ToCM.WheelOut.FR2.Trq_PB;
    IF->Trq_Reg_trg[7]	= out->ToCM.WheelOut.FR2.Trq_BrakeReg_trg;
#endif

    return 0;
}


int
UserBrake_RTW_Register (void)
{
    tModelClassDescr m;

    /*Log("%s_Register()\n", Modelname);*/

    memset(&m, 0, sizeof(m));

    /* Parameter file identification number.
       You may change CompatVersionId to the the lowest parameter
       file version understood by your model code. */
    m.Brake.VersionId		= PARAMID;
    m.Brake.CompatVersionId	= m.Brake.VersionId;

    m.Brake.DeclQuants		= UserBrake_RTW_DeclQuants;
    m.Brake.New			= UserBrake_RTW_New;
    m.Brake.Calc		= UserBrake_RTW_Calc;
    m.Brake.Delete		= UserBrake_RTW_Delete;
    /* Should only be used if the model doesn't read params from extra files */
    // m.Brake.ParamsChanged	= ParamsChanged_IgnoreCheck;

    return Model_Register(Modelclass, Modelname, &m);
}

