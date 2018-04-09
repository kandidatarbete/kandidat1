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
** Simple tire Model
**
** Add the declaration of the register function to one of your header files,
** for example to User.h and call it in User_Register()
**
**    TireCPMod_Register_MyModel ();
**
******************************************************************************
*/

#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "CarMaker.h"
#include "Car/Vehicle_Car.h"
#include "MyModels.h"

static const char ThisModelClass[] = "TireCPMod";
static const char ThisModelKind[]  = "MyModel";
static const int  ThisVersionId    = 1;


struct tMyModel {
    /* Model specific Parameters */
    int TireNo;
    double HitTime;
    double HitDuration;
    double z_offset;
};


static void
MyModel_DeclQuants_dyn (struct tMyModel *mp, int park)
{
    tDDefault *df;
    static struct tMyModel MyModel_Dummy = {0};
    if (park)
	mp = &MyModel_Dummy;

    /* Define here dict entries for dynamically allocated variables. */

    df = DDefaultCreate ("MyTireCPMod.");
    DDefDouble4 (df, "HitTime",		"s",	&mp->HitTime,  DVA_VC);
    DDefDouble4 (df, "z_offset",	"m",	&mp->z_offset, DVA_VC);
    DDefaultDelete (df);
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
MyModel_New (
    struct tInfos	*inf,
    int			TireNo,
    int			Side,
    int			Twin)
{
    struct tMyModel *mp = (struct tMyModel*)calloc(1,sizeof(*mp));
    mp->TireNo = TireNo;

    mp->HitTime      = iGetDblOpt (inf, "MyTireCPMod.HitTime",     10.0);
    mp->HitDuration  = iGetDblOpt (inf, "MyTireCPMod.HitDuration", 0.01);
    mp->z_offset     = iGetDblOpt (inf, "MyTireCPMod.z_offset",    0.2);

    return mp;
}


static int
MyModel_Calc (void *MP, tTireCPModIF *IF, double dt)
{
    struct tMyModel *mp = MP;

    /* Hit on the both rear tires at time=TimeHit by an additional contact point z offset */
    if ((mp->TireNo==2 || mp->TireNo==3) && SimCore.Time>mp->HitTime
	 && SimCore.Time<mp->HitTime+mp->HitDuration) {
	IF->CP_0[2] += mp->z_offset;
	Log ("T=%g hit on tire=%d\n", SimCore.Time, mp->TireNo);
    }

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
TireCPMod_Register_MyModel (void)
{
    tModelClassDescr m;

    memset(&m, 0, sizeof(m));
    m.TireCPMod.VersionId =	ThisVersionId;
    m.TireCPMod.New =		MyModel_New;
    m.TireCPMod.Calc =		MyModel_Calc;
    m.TireCPMod.Delete =	MyModel_Delete;
    m.TireCPMod.DeclQuants =	MyModel_DeclQuants;
    /* Should only be used if the model doesn't read params from extra files */
    m.TireCPMod.ParamsChanged = ParamsChanged_IgnoreCheck;

    return Model_Register(ModelClass_TireCPMod, ThisModelKind, &m);
}
