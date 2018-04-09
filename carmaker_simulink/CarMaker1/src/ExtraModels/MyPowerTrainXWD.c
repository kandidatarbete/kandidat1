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
** Simple powertrain Model
**
** Add the declaration of the register function to one of your header files,
** for example to User.h and call it in User_Register()
**
**    PowerTrainXWD_Register_MyModel ();
**
******************************************************************************
*/

#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "CarMaker.h"
#include "Car/Vehicle_Car.h"
#include "Car/PowerFlow.h"
#include "MyModels.h"

#define NWHEEL 4

static const char ThisModelClass[] = "PowerTrain.PTXWD";
static const char ThisModelKind[]  = "MyModel";
static const int  ThisVersionId    = 1;


struct tMyModel {
    /* Parameters */
    double Gas2Trq;	/* Coefficient Gas -> DriveTorque at Wheel */

    struct tMyWheel {
	double	Irot;
	double	Irot_act;
	double	rota;
    } Whl[NWHEEL];
};


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
MyModel_New (tInfos *Inf, struct tPowerTrainXWD_CfgIF *CfgIF, const char *KindKey)
{
    struct tMyModel *mp = NULL;
    int VersionId = 0;
    char MsgPre[64];
    const char *ModelKind;

    if ((ModelKind = SimCore_GetKindInfo(Inf, ModelClass_PowerTrainXWD, KindKey,
	 				 0, ThisVersionId, &VersionId)) == NULL)
	return NULL;

    mp = (struct tMyModel*)calloc(1,sizeof(*mp));

    sprintf (MsgPre, "%s %s", ThisModelClass, ThisModelKind);

    if (CfgIF->nWheels != NWHEEL) {
	LogErrF(EC_Init, "%s: model supports only a four wheel vehicle", MsgPre);
	goto ErrorReturn;
    }

    /* Gas -> Drive Torque */
    mp->Gas2Trq = iGetDbl(Inf, "MyPowerTrain.Gas2Trq");

    /* No shifting for IPGDriver required */
    CfgIF->PTKind             = PTKind_BEV;
    CfgIF->GearBox.GBKind     = GBKind_NoGearBox;
    CfgIF->DriveLine.DriveSourcePos[0] = Diff_Front;

    CfgIF->nMotor      = 1;
    CfgIF->nGearBoxM   = 1;
    CfgIF->Motor[0].Level = PowerSupply_HV1;
    CfgIF->GearBoxM[0].GBKind = GBKind_AutoWithManual;
    CfgIF->GearBoxM[0].ClKind = ClKind_Friction;

    if (iGetLongOpt (Inf,"PowerTrain.PowerSupply.BattLV.Active", 1)) {
	CfgIF->BattLV.SOC_min = iGetDblOpt (Inf,"PowerTrain.PowerSupply.BattLV.SOC_min", 10.0);
	CfgIF->BattLV.SOC_max = iGetDblOpt (Inf,"PowerTrain.PowerSupply.BattLV.SOC_max", 90.0);
	CfgIF->BattLV.Capacity = iGetDblOpt (Inf,"PowerTrain.PowerSupply.BattLV.Capacity", 10.0);
	CfgIF->BattLV.Voltage = iGetDblOpt (Inf,"PowerTrain.PowerSupply.BattLV.Voltage_oc", 12.0);
    }

    if (iGetLongOpt (Inf,"PowerTrain.PowerSupply.BattHV.Active", 1)) {
	CfgIF->BattHV.SOC_min = iGetDblOpt (Inf,"PowerTrain.PowerSupply.BattHV.SOC_min", 10.0);
	CfgIF->BattHV.SOC_max = iGetDblOpt (Inf,"PowerTrain.PowerSupply.BattHV.SOC_max", 90.0);
	CfgIF->BattHV.Capacity = iGetDblOpt (Inf,"PowerTrain.PowerSupply.BattHV.Capacity", 10.0);
	CfgIF->BattHV.Voltage = iGetDblOpt (Inf,"PowerTrain.PowerSupply.BattHV.Voltage_oc", 12.0);
    }

    return mp;

    ErrorReturn:
	free (mp);
	return NULL;
}


static int
MyModel_Calc (void *MP, struct tPowerTrainXWD_IF *IF, double dt)
{
    struct tMyModel *mp = MP;
    int i;

    /* Calculate the drive torque at wheels */
    for (i=0; i<NWHEEL; i++)
	IF->WheelOut[i].Trq_Drive = IF->Gas * mp->Gas2Trq;

    /* Support torque to the wheel carrier */
    for (i=0; i<NWHEEL; i++)
	IF->WheelOut[i].Trq_Supp2WC = -IF->WheelOut[i].Trq_Drive;

    /* Support torque to the vehicle body */
    IF->Trq_Supp2Bdy1[1]  = 0.0;
    IF->Trq_Supp2Bdy1B[1] = 0.0;

    return 0;
}


static void
MyModel_Delete (void *MP)
{
    struct tMyModel *mp = MP;

    /* Park the dict entries for dynamically allocated variables before deleting */
    MyModel_DeclQuants_dyn (mp, 1);
    free (mp);
}


int
PowerTrainXWD_Register_MyModel (void)
{
    tModelClassDescr m;

    memset(&m, 0, sizeof(m));
    m.PowerTrainXWD.VersionId =		ThisVersionId;
    m.PowerTrainXWD.New =		MyModel_New;
    m.PowerTrainXWD.Calc =		MyModel_Calc;
    m.PowerTrainXWD.Delete =		MyModel_Delete;
    m.PowerTrainXWD.DeclQuants =	MyModel_DeclQuants;
    /* Should only be used if the model doesn't read params from extra files */
    m.PowerTrainXWD.ParamsChanged = 	ParamsChanged_IgnoreCheck;

    return Model_Register(ModelClass_PowerTrainXWD, ThisModelKind, &m);
}
