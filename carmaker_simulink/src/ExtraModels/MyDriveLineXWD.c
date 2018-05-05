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
** Simple driveline OpenXWD Model
**
** Add the declaration of the register function to one of your header files,
** for example to User.h and call it in User_Register()
**
**    DriveLineXWD_Register_MyModel ();
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

static const char ThisModelClass[] = "PowerTrain.DLXWD";
static const char ThisModelKind[]  = "MyModel";
static const int  ThisVersionId    = 1;


struct tMyModel {
    int nWheels;
};


static void
MyModel_DeclQuants (void *MP)
{
}



static void *
MyModel_New (struct tInfos *Inf, struct tPTDriveLineXWD_CfgIF *CfgIF, const char *KindKey)
{
    struct tMyModel *mp = NULL;
    char MsgPre[64];
    const char *ModelKind;
    int VersionId = 0;

    if ((ModelKind = SimCore_GetKindInfo(Inf, ModelClass_PTDriveLineXWD, KindKey,
	 				 0, ThisVersionId, &VersionId)) == NULL)
	return NULL;

    mp = (struct tMyModel*)calloc(1,sizeof(*mp));

    sprintf (MsgPre, "%s %s", ThisModelClass, ThisModelKind);

    CfgIF->iDiff_mean = iGetDblOpt(Inf, "PowerTrain.DL.iDiff_mean", 4);

    mp->nWheels = CfgIF->nWheels;

    if (mp->nWheels != NWHEEL) {
	LogErrF(EC_Init, "%s: model supports only a four wheel vehicle", MsgPre);
	goto ErrorReturn;
    }

    return mp;

    ErrorReturn:
	free (mp);
	return NULL;
}

static int
MyModel_Calc (void *MP, struct tPTDriveLineXWD_IF *IF, double dt)
{
    const tPTDriveLineXWD_CfgIF *CfgIF = IF->CfgIF;
    double Trq;
    int iS;

    for (iS=0; iS < 2; iS++) {
	Trq = IF->DriveIn.Trq_in;
	IF->WheelOut[iS].Trq_Drive = Trq*0.5*CfgIF->iDiff_mean;
    }
    for (iS=2; iS < NWHEEL; iS++) {
    	IF->WheelOut[iS].Trq_Drive = 0;
    }

    IF->DriveOut.rotv_in =  (IF->WheelIn[0].rotv + IF->WheelIn[1].rotv)*0.5*CfgIF->iDiff_mean;

    for (iS=0; iS<NWHEEL; iS++)
	IF->WheelOut[iS].Trq_Supp2WC = -IF->WheelOut[iS].Trq_Drive;

    return 0;
}


static void
MyModel_Delete (void *MP)
{
    struct tMyModel *mp = MP;

    free (mp);
}


int 
DriveLineXWD_Register_MyModel (void)
{
    tModelClassDescr m;

    memset(&m, 0, sizeof(m));
    m.PTDriveLineXWD.VersionId =	ThisVersionId;
    m.PTDriveLineXWD.New =		MyModel_New;
    m.PTDriveLineXWD.Calc =	MyModel_Calc;
    m.PTDriveLineXWD.Delete =	MyModel_Delete;
    m.PTDriveLineXWD.DeclQuants =	MyModel_DeclQuants;
    /* Should only be used if the model doesn't read params from extra files */
    m.PTDriveLineXWD.ParamsChanged = ParamsChanged_IgnoreCheck;

    return Model_Register(ModelClass_PTDriveLineXWD, ThisModelKind, &m);
}
