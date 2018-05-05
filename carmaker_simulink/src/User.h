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

#ifndef _USER_H__
#define _USER_H__

#include <Global.h>
#include <Vehicle/MBSUtils.h>

#ifdef __cplusplus
extern "C" {
#endif



/* For debugging purposes */
#define N_USEROUTPUT	10
extern float UserOut[N_USEROUTPUT];


typedef struct tUser tUser;


struct tUser {
    /* example: powertrain */
    struct {
	struct {
	    double Dummy;
	} Engine;
    } PowerTrain;

#if defined (EXAMPLE_BDYSENSOR)
    tBdySensor  Sensor_One;
#endif /* #if defined (EXAMPLE_BDYSENSOR) */
};

extern tUser User;


int 	User_Init_First		(void);
int 	User_Init		(void);
void	User_PrintUsage		(const char *Pgm);
char  **User_ScanCmdLine	(int argc, char **argv);
int 	User_Start		(void);
int	User_Register		(void);
void	User_DeclQuants		(void);
int 	User_ShutDown		(int ShutDownForced);
int 	User_End		(void);
void 	User_Cleanup		(void);

int	User_TestRun_Start_atBegin		(void);
int	User_TestRun_Start_atEnd		(void);
int	User_TestRun_Start_StaticCond_Calc	(void);
int	User_TestRun_Start_Finalize		(void);
int	User_TestRun_RampUp			(double dt);
int	User_DrivMan_Calc			(double dt);
int	User_VehicleControl_Calc		(double dt);
int	User_Traffic_Calc			(double dt);
int	User_Calc				(double dt);
int	User_Check_IsIdle			(int IsIdle);
int	User_TestRun_End_First 			(void);
int	User_TestRun_End 			(void);

void 	User_In  (const unsigned CycleNo);
void	User_Out (const unsigned CycleNo);


/* User_<> functions,
** - called from SimCore and in CM_Main.c,
** - already defined in SimCore.h
*/
int 	User_Param_Get		(void);
int 	User_Param_Add		(void);
int 	User_ApoMsg_Eval (int channel, char *msg, int len, int who);
void 	User_ApoMsg_Send (double T, const unsigned CycleNo);


#define User_TestRun_Start   User_TestRun_Start__deprecated_function__Change_to__User_TestRun_Start_XYZ;


#ifdef __cplusplus
}
#endif

#endif	/* #ifndef _USER_H__ */
