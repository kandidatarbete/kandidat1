/* $Id: MyDriveLine.c,v 1.3 2016/03/18 14:32:58 fh Exp $ (c) IPG */
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
**
** Simple driveline Model
**
** Add the declaration of the register function to one of your header files,
** for example to User.h and call it in User_Register()
**
**    DriveLine_Register_MyModel ();
**
******************************************************************************
*/

#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "CarMaker.h"
#include "Car/Vehicle_Car.h"
#include "MyModels.h"

#define NWHEEL 4

static const char ThisModelClass[] = "PowerTrain.DL";
static const char ThisModelKind[]  = "MyModel";
static const int  ThisVersionId    = 1;


struct tMyModel {
    int nWheels;
    double Irot[NWHEEL];
    double rota[NWHEEL];
    double x[NWHEEL];
};


static void
MyModel_DeclQuants (void *MP)
{
}



static void *
MyModel_New (struct tInfos *Inf, struct tPTDriveLineCfgIF *CfgIF, const char *KindKey)
{
    struct tMyModel *mp = NULL;
    char MsgPre[64];
    const char *ModelKind;
    int iS, VersionId = 0;

    if ((ModelKind = SimCore_GetKindInfo(Inf, ModelClass_PTDriveLine, KindKey,
	 				 0, ThisVersionId, &VersionId)) == NULL)
	return NULL;

    mp = (struct tMyModel*)calloc(1,sizeof(*mp));

    sprintf (MsgPre, "%s %s", ThisModelClass, ThisModelKind);

    CfgIF->iDiff_mean = iGetDblOpt(Inf, "PowerTrain.DL.iDiff_mean", 4);
    CfgIF->DriveSourcePos[0] = Diff_Front;

    mp->nWheels = CfgIF->nWheels;

    if (mp->nWheels != NWHEEL) {
	LogErrF(EC_Init, "%s: model supports only a four wheel vehicle", MsgPre);
	goto ErrorReturn;
    }
 
    for (iS=0; iS < mp->nWheels; iS++) {
	mp->Irot[iS] = CfgIF->Wheel_Iyy[iS];
    }

    return mp;

    ErrorReturn:
	free (mp);
	return NULL;
}

static int
MyModel_Calc (void *MP, struct tPTDriveLineIF *IF, double dt)
{
    struct tMyModel *mp = MP;
    const tPTDriveLineCfgIF *CfgIF = IF->CfgIF;
    double Trq_Ext2W[NWHEEL];
    double Trq;
    int iS, WithBrake=0;

    for (iS=0; iS < 2; iS++) {
	Trq = IF->DriveIn[0].Trq_in;
	IF->WheelOut[iS].Trq_Drive = Trq*0.5*CfgIF->iDiff_mean;
    }
    for (iS=2; iS < NWHEEL; iS++) {
    	IF->WheelOut[iS].Trq_Drive = 0;
    }

    IF->DriveOut[0].rotv_in =  (IF->WheelOut[0].rotv + IF->WheelOut[1].rotv)*0.5*CfgIF->iDiff_mean;

    for (iS=0; iS<NWHEEL; iS++)
	IF->WheelOut[iS].Trq_Supp2WC = -IF->WheelOut[iS].Trq_Drive;

    /** Update Trq **/
    for (iS=0; iS < NWHEEL; iS++) {
	if (IF->WheelIn[iS].Trq_Brake > 0.0)
	    WithBrake = 1;
    }

    if (WithBrake) {
	for (iS=0; iS < NWHEEL; iS++) {
	    struct tPTDriveLineIF_WheelIn  *wIn  = &IF->WheelIn[iS];
	    struct tPTDriveLineIF_WheelOut *wOut = &IF->WheelOut[iS];
	    double xabs, x, TrqP;

	    /* dynamic friction */
	    /* P element */
	    TrqP = M_MIN(100.0, 1000.0 * wOut->rotv);

	    x = wIn->Trq_T2W + wOut->Trq_Drive + TrqP;
	    xabs = fabs(x);
	    wOut->Trq_B2W = M_MIN(xabs, wIn->Trq_Brake);

	    /* against moving momentum */
	    wOut->Trq_B2W = -M_SGN(x) * wOut->Trq_B2W;
	}
    } else {
	for (iS=0; iS < NWHEEL; iS++)
	    IF->WheelOut[iS].Trq_B2W = 0.0;
    }

    if (SimCore.State == SCState_EndIdleGet || SimCore.State == SCState_EndIdleSet) {
	for (iS=0; iS < NWHEEL; iS++)
	    IF->WheelOut[iS].Trq_B2W = 0.0;
    }

    for (iS=0; iS < NWHEEL; iS++) {
	struct tPTDriveLineIF_WheelIn  *wIn  = &IF->WheelIn[iS];
	struct tPTDriveLineIF_WheelOut *wOut = &IF->WheelOut[iS];
	
	Trq_Ext2W[iS] = wOut->Trq_B2W + wIn->Trq_T2W + wIn->Trq_WhlBearing;
    }

    /* Wheel accelerations */
    for (iS=0; iS<NWHEEL; iS++) {
	mp->rota[iS] = (IF->WheelOut[iS].Trq_Drive + Trq_Ext2W[iS]) / (mp->Irot[iS] + IF->DriveIn[0].Inert_in*0.5);
    }

    /* Integration */
    if (SimCore.State != SCState_Simulate && SimCore.State != SCState_EndIdleGet) {
	for (iS=0; iS<NWHEEL; iS++)
	    mp->rota[iS] = 0.0;
    }
    if (SimCore.State == SCState_EndIdleSet) {
	for (iS=0; iS<NWHEEL; iS++)
	    IF->WheelOut[iS].rotv = 0.0;
    } else {
	for (iS=0; iS<NWHEEL; iS++) {
	    IF->WheelOut[iS].rotv += mp->rota[iS] * dt;
	    IF->WheelOut[iS].rot  += IF->WheelOut[iS].rotv * dt;
	}
    }

    return 0;
}


static void
MyModel_Delete (void *MP)
{
    struct tMyModel *mp = MP;

    free (mp);
}


int 
DriveLine_Register_MyModel (void)
{
    tModelClassDescr m;

    memset(&m, 0, sizeof(m));
    m.PTDriveLine.VersionId =	ThisVersionId;
    m.PTDriveLine.New =		MyModel_New;
    m.PTDriveLine.Calc =	MyModel_Calc;
    m.PTDriveLine.Delete =	MyModel_Delete;
    m.PTDriveLine.DeclQuants =	MyModel_DeclQuants;
    /* Should only be used if the model doesn't read params from extra files */
    m.PTDriveLine.ParamsChanged = ParamsChanged_IgnoreCheck;

    return Model_Register(ModelClass_PTDriveLine, ThisModelKind, &m);
}
