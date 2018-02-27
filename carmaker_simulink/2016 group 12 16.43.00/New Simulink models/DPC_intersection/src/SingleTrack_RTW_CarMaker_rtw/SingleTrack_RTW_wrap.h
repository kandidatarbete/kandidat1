/***************************************************** target specific file ***/
/*  CarMaker wrapper module for Simulink models                               */
/*  ------------------------------------------------------------------------  */
/*  Copyright (c) IPG Automotive GmbH      www.ipg.de   Fon: +49.721.98520-0  */
/*  Bannwaldallee 60      D-76185 Karlsruhe   Germany   Fax: +49.721.98520-99 */
/******************************************************************************/

#ifndef __SINGLETRACK_RTW_WRAP_H__
#define __SINGLETRACK_RTW_WRAP_H__

#ifndef IS_CAR
# define IS_CAR
#endif

#ifdef __cplusplus
extern "C" {
#endif


struct tInfos;
struct tMdlBdyFrame;
struct tMatSuppDictDef;
struct tMatSuppTunables;


#if MATSUPP_NUMVER >= 80100
# define rtModel_SingleTrack_RTW          rtModel_SingleTrack_RTW_T
# define ExternalInputs_SingleTrack_RTW   ExtU_SingleTrack_RTW_T
# define ExternalOutputs_SingleTrack_RTW  ExtY_SingleTrack_RTW_T
#endif

#ifndef SingleTrack_RTW_rtModel
typedef struct rtModel_SingleTrack_RTW rtModel_SingleTrack_RTW;
#endif

/* Model registration function */
rtModel_SingleTrack_RTW *SingleTrack_RTW (struct tInfos *Inf);

#if !defined(CM4SLDS) && defined(MATSUPP_NUMVER) && MATSUPP_NUMVER >= 70900
void rt_ODECreateIntegrationData (RTWSolverInfo *si);
void rt_ODEUpdateContinuousStates(RTWSolverInfo *si);
void rt_ODEDestroyIntegrationData(RTWSolverInfo *si);
#endif


/* Dictionary variables and other items of the model */
extern struct tMatSuppDictDef *SingleTrack_RTW_DictDefines;
extern struct tMdlBdyFrame *SingleTrack_RTW_BdyFrameDefines;


/* Wrapper functions */
void SingleTrack_RTW_SetParams (rtModel_SingleTrack_RTW *rts,
			struct tMatSuppTunables *tuns,
			struct tInfos *Inf);
int SingleTrack_RTW_Register (void);


#ifdef __cplusplus
}
#endif

#endif /* __SINGLETRACK_RTW_WRAP_H__ */
