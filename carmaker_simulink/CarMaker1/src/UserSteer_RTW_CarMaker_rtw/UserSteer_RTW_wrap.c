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

#include "infoc.h"
#include "DataDict.h"
#include "SimCore.h"
#include "Log.h"
#include "MatSupp.h"
#include "ModelManager.h"

#include "Vehicle/MBSUtils.h"

#if defined (IS_CAR) || defined (IS_TRUCK)
# include "Car/Car.h"
# include "Car/Steering.h"
#elif defined(IS_MCYCLE)
# include "MCycle/MCycle.h"
# include "MCycle/Steering.h"
#endif

#include "UserSteer_RTW.h"
#include "UserSteer_RTW_wrap.h"


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

extern const char UserSteer_RTW_LibIdent[];
const char UserSteer_RTW_LibIdent[] = "(@@)" QUOTE(MODEL) " " ARCH " 1.0 " BUILDDATE;

static const char Modelname[] = QUOTE(MODEL);
static const tModelClass Modelclass = ModelClass_Steering;
static tMatSuppSampling SampleParams;


static void
DoOneStep (rtModel_UserSteer_RTW *rtm)
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
UserSteer_RTW_DeclQuants (void *MP)
{
    rtModel_UserSteer_RTW *rtm = (rtModel_UserSteer_RTW *)MP;
    int i;

    /*Log("%s_DeclQuants()\n", Modelname);*/

    if (rtm == NULL) {
	/* Remember body frames defined in this model for later registration. */
        MdlBdyFrame_Add(UserSteer_RTW_BdyFrameDefines);

	/* Define dict entries for non-dynamically allocated variables. */
	if ((i = MatSupp_DeclQuants(UserSteer_RTW_DictDefines)) >= 0) {
	    LogErrF(EC_Init, "Model '%s': Error defining quantity '%s'\n",
		    Modelname, UserSteer_RTW_DictDefines[i].Name);
	}

	/* Define dict entries for tunable parameters (with dummy address). */
	DeclParameterQuants(NULL);
    } else {
	/* Define dict entries for dynamically allocated variables. */
    }
}


/*
 * UserSteer_RTW_SetParams() will be invoked indirectly by the generated
 * model C code each time UserSteer_RTW_New() is called.
 */
void
UserSteer_RTW_SetParams (rtModel_UserSteer_RTW *rtm, struct tMatSuppTunables *tuns,
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
UserSteer_RTW_New (struct tInfos *Inf, const char *KindKey)
{
    rtModel_UserSteer_RTW *rtm;
    double rtmSampleTime;
    const char *ModelKind;
    int VersionId = 0;

    if ((ModelKind = SimCore_GetKindInfo(Inf, Modelclass, KindKey,
	 				 0, PARAMID, &VersionId)) == NULL)
	return NULL;

    /*Log("%s_New()\n", Modelname);*/

    MatSupp_ResetQuants(UserSteer_RTW_DictDefines);

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

#ifdef CLASSIC_INTERFACE
    rtmiStart(rtmGetRTWRTModelMethodsInfo(rtm));
#else
    MODEL_INITIALIZE(rtm);
#endif

    return rtm; /* Will be passed as MP to the other functions. */
}


static void
UserSteer_RTW_Delete (void *MP)
{
    rtModel_UserSteer_RTW *rtm = (rtModel_UserSteer_RTW *)MP;

    /*Log("%s_Delete()\n", Modelname);*/

#ifdef CLASSIC_INTERFACE
    rt_SimDestroyTimingEngine(rtmGetTimingData(rtm));
    if (rtmGetNumContStates(rtm) > 0)
	rt_ODEDestroyIntegrationData(rtmGetRTWSolverInfo(rtm));
    rtmiTerminate(rtmGetRTWRTModelMethodsInfo(rtm));
#else
    MODEL_TERMINATE(rtm);
#endif
}


static int
UserSteer_RTW_Calc (void *MP, tSteeringIF *IF, double dt)
{
    rtModel_UserSteer_RTW *rtm = (rtModel_UserSteer_RTW *)MP;
    int osCount=0;
    ExternalInputs_UserSteer_RTW  *in  = (ExternalInputs_UserSteer_RTW *) rtmGetU(rtm);
    ExternalOutputs_UserSteer_RTW *out = (ExternalOutputs_UserSteer_RTW *)rtmGetY(rtm);

    in->FromCM.SteerBy		= IF->SteerBy;

    in->FromCM.Ang     		= IF->Ang;
    in->FromCM.AngVel  		= IF->AngVel;
    in->FromCM.AngAcc  		= IF->AngAcc;
    in->FromCM.Trq     		= IF->Trq;

    in->FromCM.Frc_L        	= IF->L.Frc;
    in->FromCM.Inert_L     	= IF->L.Inert;

    in->FromCM.Frc_R        	= IF->R.Frc;
    in->FromCM.Inert_R      	= IF->R.Inert;

    in->FromCM.AssistFrc 	= IF->AssistFrc;
    in->FromCM.AssistTrqCol	= IF->AssistTrqCol;
    in->FromCM.AssistTrqPin	= IF->AssistTrqPin;

    in->FromCM.AssistFrc_Ext 	= IF->AssistFrc_Ext;
    in->FromCM.AssistTrqCol_Ext	= IF->AssistTrqCol_Ext;
    in->FromCM.AssistTrqPin_Ext	= IF->AssistTrqPin_Ext;

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

    IF->Ang             = out->ToCM.Ang;
    IF->AngVel          = out->ToCM.AngVel;
    IF->AngAcc          = out->ToCM.AngAcc;
    IF->Trq             = out->ToCM.Trq;
    IF->TrqStatic       = out->ToCM.TrqStatic;

    IF->AssistFrc       = out->ToCM.AssistFrc;
    IF->AssistTrqCol    = out->ToCM.AssistTrqCol;
    IF->AssistTrqPin    = out->ToCM.AssistTrqPin;

    IF->L.q             = out->ToCM.q_L;
    IF->L.qp            = out->ToCM.qp_L;
    IF->L.qpp           = out->ToCM.qpp_L;
    IF->L.iSteer2q      = out->ToCM.iSteer2q_L;

    IF->R.q             = out->ToCM.q_R;
    IF->R.qp            = out->ToCM.qp_R;
    IF->R.qpp           = out->ToCM.qpp_R;
    IF->R.iSteer2q      = out->ToCM.iSteer2q_R;

    return 0;
}


int
UserSteer_RTW_Register (void)
{
    tModelClassDescr m;

    /*Log("%s_Register()\n", Modelname);*/

    memset(&m, 0, sizeof(m));

    /* Parameter file identification number.
       You may change CompatVersionId to the the lowest parameter
       file version understood by your model code. */
    m.Steering.VersionId		= PARAMID;
    m.Steering.CompatVersionId		= m.Steering.VersionId;

    m.Steering.DeclQuants		= UserSteer_RTW_DeclQuants;
    m.Steering.New			= UserSteer_RTW_New;
    m.Steering.Calc			= UserSteer_RTW_Calc;
    m.Steering.Delete			= UserSteer_RTW_Delete;
    /* Should only be used if the model doesn't read params from extra files */
    // m.Steering.ParamsChanged		= ParamsChanged_IgnoreCheck;

    return Model_Register(Modelclass, Modelname, &m);
}

