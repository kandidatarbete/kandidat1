/*
******************************************************************************
**  CarMaker - Version 5.0.3
**  Vehicle Dynamic Simulation Toolkit
**
**  Copyright (C)   IPG Automotive GmbH
**                  Bannwaldallee 60             Phone  +49.721.98520.0
**                  76185 Karlsruhe              Fax    +49.721.98520.99
**                  Germany                      WWW    http://www.ipg.de
******************************************************************************
*/

#ifndef _IO_H__
#define _IO_H__

#if defined(XENO)
# include <PowerUTA.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

struct tInfos;


/*** Input Vector, signals from hardware, ... */
typedef struct {
    double	T;
    float	DeltaT;			/* DeltaT of the last time step */
    float	ax;			/* longitudinal acceleration 	*/
    float	ay;			/* lateral acceleration 	*/
    float	YawRate;		/* yaw rate  			*/
    float	StWhlAngle;		/* steering wheel angle 	*/
    float	Gas;			/* gas pedal          		*/
    char	BLS;			/* brake light switch 		*/

    float	WheelSpd[4];		/* wheel speeds                 */
    unsigned	HydValveMask;		/* hydraulic valves (hardware)	*/
    float	HydValve[12];		/* hydraulic valves 		*/
    char	PumpIsOn;		/* hydraulic pump active	*/
    char	Rel_SW;			/* brake/booster release switch */
    float	BooSignal;		/* booster signal		*/
    float	PuRetVolt;		/* hyd.pump return voltage	*/
    float	pMC;			/* pressure master cylinder 	*/
    float	pWB[4];			/* pressure wheel brake	       	*/

#if defined(XENO)
    /* Power UTA */
    PowerUTA	PwrUTA;
    struct {
	char	PWR;
	float	U_Read;			/* actual voltage, calibrated  */
	float	I_Read;			/* actual current, calibrated  */
	float	U_Set;			/* nominal voltage, calibrated */
	float	I_Set;			/* maximum current, calibrated */
    } PwrSupply;
#endif
} tIOVec;

extern tIOVec IO;


/*** Error classes */

enum {
    EC_App1 = LogECReserved,
    EC_App2
};


/*** I/O configuration */

/* extern int IO_None; DON'T - Variable is predefined by CarMaker! */
extern int IO_DemoApp;
#if defined(WITH_FLEXRAY)
extern int IO_DemoFR;
#endif
#if defined(WITH_SOMEIP)
extern int IO_DemoSIP;
#endif


/*** Slot numbers */

enum {
    Slot_AD  = 0,
    Slot_DA  = 1,
    Slot_CAN = 2,
    Slot_Din = 3,
    Slot_FG  = 4,
    Slot_Rel = 5
};


/*** I/O calibration */

typedef struct tCal {
    float	Min;
    float	Max;
    float	LimitLow;
    float	LimitHigh;
    float	Factor;
    float	Offset;
    char	Rezip;
} tCal;

typedef struct tCalVec {
    tCal	ax;
    tCal	ay;
    tCal	YawRate;
    tCal	StWhlAngle;
    tCal	Imon;
    tCal	Umon;
    tCal	Iout;
    tCal	Uout;
} tCalVec;

extern struct tCalVec Cal;


void	iGetCal	(struct tInfos *Inf, const char *key, tCal *cal, int optional);
float	CalIn   (tCal *cal, const int Value);
float	CalInF  (tCal *cal, const float Value);
int	CalOut	(tCal *cal, float Value);
float	CalOutF (tCal *cal, float Value);
int 	LimitInt (float fValue, int Min, int Max);


int	IO_Init_First (void);
int	IO_Init (void);
int	IO_Init_Finalize (void);

int	IO_Param_Get (struct tInfos *inf);
void	IO_In  (unsigned CycleNo);
void	IO_Out (unsigned CycleNo);

void	IO_Cleanup (void);


#ifdef __cplusplus
}
#endif

#endif	/* #ifndef _IO_H__ */

