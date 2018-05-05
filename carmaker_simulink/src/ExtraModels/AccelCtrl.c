/*
******************************************************************************
**  CarMaker - Version 5.1.4
**  Vehicle Dynamics Simulation Toolkit
**
**  Copyright (C)   IPG Automotive GmbH
**                  Bannwaldallee 60             Phone  +49.721.98520.0
**                  76185 Karlsruhe              Fax    +49.721.98520.99
**                  Germany                      WWW    http://www.ipg.de
******************************************************************************
*/
#include <Global.h>

#if defined(INTIME)
#  include <rt.h>
#endif
#if defined(WIN32) && !defined(INTIME)
#  include <windows.h>
#endif
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "Log.h"
#include "DrivMan.h"
#include "DataDict.h"
#include "InfoUtils.h"
#include "SimCore.h"
#include "ModelManager.h"
#include "MathUtils.h"

#include "Car/DASensor.h"
#include "Vehicle.h"
#include "VehicleControl.h"
#include "Car/VehicleControlApps.h"


tAccelCtrl AccelCtrl;

static double c_i;
static tDesrAccelFunc User_DesrAccelFunc = NULL;


static void
AccelCtrl_DeclQuants (void *MP)
{
    tACC_ECU *ACC_ECU = &AccelCtrl.ACC_ECU;
    tDDictEntry *e;
    tDDefault   *df;

    if (MP==NULL)
	return;

    df = DDefaultCreate("AccelCtrl.");

    DDefDouble4 (df, "DesiredAx",          "m/s^2", &AccelCtrl.DesrAx,        DVA_DM);

    e = DDefChar (df, "ACC.IsActive",      "",      &ACC_ECU->IsActive,       DVA_DM);
    DDefStates(e, 2, 0);

    DDefDouble4 (df, "ACC.DesiredSpd",     "m/s",   &ACC_ECU->DesrSpd,        DVA_DM);
    DDefDouble4 (df, "ACC.DesiredDist",	   "m",     &ACC_ECU->DesrDist,       DVA_None);
    DDefDouble4 (df, "ACC.DesiredTGap",	   "s",     &ACC_ECU->DesrTGap,       DVA_DM);
    DDefDouble4 (df, "ACC.DesiredAx",	   "m/s^2", &ACC_ECU->DesrAx,         DVA_DM);
    DDefDouble4 (df, "ACC.Time2Collision", "s",     &ACC_ECU->Time2Collision, DVA_None);

    DDefaultDelete(df);
}


static void
DesrAccelFunc_ACC (double dt)
{
    double ax, ax_sc, delta_ds;
    tACC_ECU *ACC_ECU = &AccelCtrl.ACC_ECU;
    struct tDASensor *s = &DASensor[ACC_ECU->RefDASensorId];

    /* Driver Brake Limit */
    if (DrivMan.Brake > ACC_ECU->BrakeThreshold)
	ACC_ECU->IsActive = 0;

    /* Time until collision */
    if (s->relvTarget.NearPnt.dv_p < 0) {
	ACC_ECU->Time2Collision =  s->relvTarget.NearPnt.ds_p
			       / -s->relvTarget.NearPnt.dv_p;
    } else {
	ACC_ECU->Time2Collision = 0;
    }

    if (!ACC_ECU->IsActive) {
	/* ACC off -> set desired speed to current car speed */
	ACC_ECU->DesrSpd = Vehicle.v;
	ACC_ECU->DesrAx = 0.0;
	goto SetAccelCtrl;
    }

    /* ACC active */
    if (s->Targ_Dtct) {
	/* if target detected set desired distance,
       DesrDistance[m] = Target.v[m/s] * 3.6 / Desired Time Gap(Init= 1.8[s])
       or if target stand still DSMIN: 20[m] distance */
	ACC_ECU->DesrDist = M_MAX( ((Vehicle.v +
		s->relvTarget.NearPnt.dv_p) * ACC_ECU->DesrTGap), ACC_ECU->dsmin);
	
	/* Distance Control Algorithm: result = desired ax */
	delta_ds =s->relvTarget.NearPnt.ds_p- ACC_ECU->DesrDist; /* d_ist-d_soll */
	ax = (delta_ds)/(ACC_ECU->dc_kd)+s->relvTarget.NearPnt.dv_p/ACC_ECU->dc_kv;
	/* ax_sc = desired ax from Speed Control */
	ax_sc =(ACC_ECU->DesrSpd - Vehicle.v)/ ACC_ECU->sc_kv;

	/* Limitation */
	if (ax > ax_sc ) ax = ax_sc;
	if (ax > ACC_ECU->axmax) ax = ACC_ECU->axmax;
	if (ax < ACC_ECU->axmin) ax = ACC_ECU->axmin;
    } else {
	/* Speed Control Algorithm: result = desired ax */
	/*      s->relvTarget.ds = -1; */
	ax = (ACC_ECU->DesrSpd - Vehicle.v) / ACC_ECU->sc_kv;
	/* Limitation */
	if (ax > ACC_ECU->axmax) ax =  ACC_ECU->axmax;
	if (ax < -0.35) ax = -0.35;
    }

    ACC_ECU->DesrAx = ax;

    SetAccelCtrl:
	AccelCtrl.DesrAx = ACC_ECU->DesrAx;
	return;
}


static void *
AccelCtrl_New (struct tInfos *Inf, const char *kindkey)
{
    char *key, *s, buf[64];
    tACC_ECU *ACC_ECU = &AccelCtrl.ACC_ECU;

    c_i = 0.0;
    memset(&AccelCtrl, 0, sizeof(AccelCtrl));

    /** AccelCtrl */
    key="AccelCtrl";

    AccelCtrl.p_gain = iGetDblOpt(Inf, strcat(strcpy(buf, key), ".p_gain"), 0.001);
    AccelCtrl.i_gain = iGetDblOpt(Inf, strcat(strcpy(buf, key), ".i_gain"), 0.001);

    /* function pointer for ax-Calculation */
    s = iGetStrOpt(Inf, strcat(strcpy(buf, key), ".DesrAccelFunc"), "ACC");

    if (strcmp(s, "ACC")==0) {
	AccelCtrl.DesrAccelFunc = DesrAccelFunc_ACC;
    } else if (strcmp(s, "DVA")==0) {
	AccelCtrl.DesrAccelFunc = NULL;
    } else if (strcmp(s, "User")==0) {
	if (User_DesrAccelFunc != NULL)
	    AccelCtrl.DesrAccelFunc = User_DesrAccelFunc;
	else
	    AccelCtrl.DesrAccelFunc = NULL;
    } else {
	LogErrF(EC_Init, "AccelCtrl: no supported function '%s' for ax calculation", s);
	return NULL;
    }

    /** ACC */
    key="AccelCtrl.ACC";

    /* Active / switched on ? */
    ACC_ECU->IsActive = iGetLongOpt (Inf, strcat(strcpy(buf, key), ".IsActive"), 0);

    /* Limit of driver brake to deactivate ACC */
    ACC_ECU->BrakeThreshold = iGetDblOpt(Inf, strcat(strcpy(buf, key), ".BrakeThreshold"), 0.2);
    /* initial time gap / speed */
    ACC_ECU->DesrTGap = iGetDblOpt(Inf, strcat(strcpy(buf, key), ".DesrTGap"), 1.8);
    ACC_ECU->DesrSpd  = iGetDblOpt(Inf, strcat(strcpy(buf, key), ".DesrSpd"),  DrivMan.Cfg.Velocity);
    /* controller parameters */
    ACC_ECU->dc_kd = iGetDblOpt(Inf, strcat(strcpy(buf, key), ".DistCtrl.kd"), 36.0);
    ACC_ECU->dc_kv = iGetDblOpt(Inf, strcat(strcpy(buf, key), ".DistCtrl.kv"), 2.0);
    ACC_ECU->sc_kv = iGetDblOpt(Inf, strcat(strcpy(buf, key), ".SpdCtrl.kv"), 13.0);
    /* min/max values */
    ACC_ECU->axmin = iGetDblOpt(Inf, strcat(strcpy(buf, key), ".AxMin"), -2.5);
    ACC_ECU->axmax = iGetDblOpt(Inf, strcat(strcpy(buf, key), ".AxMax"), 1.0);
    ACC_ECU->dsmin = iGetDblOpt(Inf, strcat(strcpy(buf, key), ".DistMin"), 20.0);
    /* Name of the reference DASensor */
    s = iGetStrOpt(Inf, strcat(strcpy(buf, key), ".RefDASensorName"), "RadarL");
    if ((ACC_ECU->RefDASensorId = DASensor_FindIndexForName(s))<0) {
	LogErrF(EC_Init, "%s: no DASensor found with the name '%s'", key, s);
	return NULL;
    }

    return &AccelCtrl;
}


static int
AccelCtrl_Calc (void *MP, double dt)
{
    double c, delta_ax, c_p;

    if (SimCore.State != SCState_Simulate)
	return 0;

    /* Calculate target longitudinal acceleration ax */
    if (AccelCtrl.DesrAccelFunc != NULL)
	AccelCtrl.DesrAccelFunc(dt);

    /* Controller for converting desired ax to gas or brake */
    if (AccelCtrl.DesrAx == 0.0) {
	/* no control required */
	c_i = VehicleControl.Gas;
	return 0;
    }

    delta_ax = AccelCtrl.DesrAx - Vehicle.PoI_Acc_1[0];
    c_p  = AccelCtrl.p_gain * delta_ax;
    c_i += AccelCtrl.i_gain * delta_ax;
    c = c_p + c_i;	/* PI-Controller */

    /* Limitation */
    if (c >  1) c =  1;
    if (c < -1) c = -1;
    c_i = c - c_p;

    /* Gas or Brake */
    if (c >= 0){
	VehicleControl.Gas   =  c;
	VehicleControl.Brake =  0;
    } else {
	VehicleControl.Gas   =  0;
	VehicleControl.Brake =  -c;
    }

    return 0;
}


void
Set_UserDesrAccelFunc(tDesrAccelFunc DesrAccelFunc)
{
    User_DesrAccelFunc = DesrAccelFunc;
}


int
VC_Register_AccelCtrl (void)
{
    tModelClassDescr m;

    memset(&m, 0, sizeof(m));
    m.VehicleControl.New    = 		AccelCtrl_New;
    m.VehicleControl.Calc   = 		AccelCtrl_Calc;
    m.VehicleControl.DeclQuants   = 	AccelCtrl_DeclQuants;
    m.VehicleControl.Delete = 		NULL;
    /* Should only be used if the model doesn't read params from extra files */
    m.VehicleControl.ParamsChanged = 	ParamsChanged_IgnoreCheck;

    return Model_RegisterIPG(ModelClass_VehicleControl, "AccelCtrl", &m);
}
