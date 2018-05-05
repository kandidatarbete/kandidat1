/* $Id: MyMotor.c,v 1.3 2016/03/18 14:32:58 fh Exp $ (c) IPG */
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
** Simple motor Model
**
** Add the declaration of the register function to one of your header files,
** for example to User.h and call it in User_Register()
**
**    Motor_Register_MyModel ();
**
******************************************************************************
*/

#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "CarMaker.h"
#include "Car/Vehicle_Car.h"
#include "MyModels.h"

static const char ThisModelClass[] = "Powertrain.Motor";
static const char ThisModelKind[]  = "MyModel";
static const int  ThisVersionId    = 1;


struct tMyModel {
    double Inert;
    double rotv;
    double rotv_max;
    double Ratio;
    double Trq;
    tLM    *Map;
};

static void MyModel_Delete (void *MP);


static void
MyModel_DeclQuants_dyn (struct tMyModel *mp, int park)
{
    static struct tMyModel MyModel_Dummy = {0};
    if (park)
	mp = &MyModel_Dummy;

    /* Define here dict entries for dynamically allocated variables. */
}


static void
MyModel_DeclQuants (void *MP)
{
    struct tMyModel *mp = MP;

    if (mp == NULL) {
	/* Define here dict entries for non-dynamically allocated (static) variables. */

    } else {
	MyModel_DeclQuants_dyn (mp, 0);
    }
}



static void *
MyModel_New (struct tInfos *Inf, struct tPTMotorCfgIF *CfgIF, const char *KindKey , const char *Ident)
{
    struct tMyModel *mp = NULL;
    const char *ModelKind;
    int VersionId = 0;

    if ((ModelKind = SimCore_GetKindInfo(Inf, ModelClass_PTMotor, KindKey,
	 				 0, ThisVersionId, &VersionId)) == NULL)
	return NULL;

    mp = (struct tMyModel*)calloc(1,sizeof(*mp));

    char MsgPre[64];
    int nRow, i;
    float *MapTable;
    sprintf (MsgPre, "%s %s", ThisModelClass, ThisModelKind);

    mp->Inert = fabs(iGetDbl (Inf, "PowerTrain.MotorISG.I"));
    mp->Ratio = fabs(iGetDbl (Inf, "PowerTrain.MotorISG.Ratio"));
    if (mp->Ratio==0.0) {
	LogErrF (EC_Init, "PowerTrain.MotorISG.Ratio must be positive and non zero");
	goto ErrorReturn;
    }
    mp->Inert *= mp->Ratio * mp->Ratio;

    char LevelString[4];
    sprintf(LevelString, iGetStr (Inf, "PowerTrain.MotorISG.VoltageLevel"));
    if (strncmp(LevelString, "LV", 4) == 0) {
        CfgIF->Level = PowerSupply_LV;
    } else if (strncmp(LevelString, "HV1", 4) == 0) {
        CfgIF->Level = PowerSupply_HV1;
    } else if (strncmp(LevelString, "HV2", 4) == 0) {
        CfgIF->Level = PowerSupply_HV2;
    } else {
        LogErrF(EC_Init, "Model '%s': %s is not a valid PowerSupply voltage level", ThisModelKind, LevelString);
    }

    nRow = 0;
    MapTable = iGetTableFlt2 (Inf, "PowerTrain.MotorISG.Mot.TrqMap", 2, &nRow);
    if (MapTable != NULL) {
	for (i=0; i<nRow; i++)
	    MapTable[i] *= rpm2radsec;
	mp->Map = LMInit(MapTable, MapTable+nRow, nRow, 0);
	mp->rotv_max = MapTable[nRow-1];
	free(MapTable);
    } else {
	goto ErrorReturn;
    }

    CfgIF->Ratio        = mp->Ratio;
    CfgIF->rotv_Mot_max = mp->rotv_max / mp->Ratio;
    CfgIF->TrqMot_max   = mp->Map;
    CfgIF->rotv_Gen_max = mp->rotv_max / mp->Ratio;
    CfgIF->TrqGen_max   = mp->Map;

    return mp;

    ErrorReturn:
	MyModel_Delete (mp);
	return NULL;
}

static int
MyModel_Calc (void *MP, struct tPTMotorIF *IF, double dt)
{
    struct tMyModel *mp = MP;
    const tPTMotorCfgIF *CfgIF = IF->CfgIF;

    mp->rotv = IF->rotv * mp->Ratio;
    mp->Trq  = LMEval (CfgIF->TrqMot_max, fabs(mp->rotv)) * IF->Load;

    IF->Trq     = mp->Trq * mp->Ratio;
    IF->PwrElec = IF->Trq * IF->rotv;
    IF->Inert   = mp->Inert;

    return 0;
}


static void
MyModel_Delete (void *MP)
{
    struct tMyModel *mp = MP;

    if (mp->Map!=NULL)
	LMDelete (mp->Map);

    /* Park the dict entries for dynamically allocated variables before deleting */
    MyModel_DeclQuants_dyn (mp, 1);
    free (mp);
}

int 
Motor_Register_MyModel (void)
{
    tModelClassDescr m;

    memset(&m, 0, sizeof(m));
    m.PTMotor.VersionId =	ThisVersionId;
    m.PTMotor.New =		MyModel_New;
    m.PTMotor.Calc =		MyModel_Calc;
    m.PTMotor.Delete =		MyModel_Delete;
    m.PTMotor.DeclQuants =	MyModel_DeclQuants;
    /* Should only be used if the model doesn't read params from extra files */
    m.PTMotor.ParamsChanged = 	ParamsChanged_IgnoreCheck;

    return Model_Register(ModelClass_PTMotor, ThisModelKind, &m);
}
