/***************************************************** target specific file ***/
/*  CarMaker wrapper module for Simulink models                               */
/*  ------------------------------------------------------------------------  */
/*  Copyright (c) IPG Automotive GmbH      www.ipg.de   Fon: +49.721.98520-0  */
/*  Bannwaldallee 60      D-76185 Karlsruhe   Germany   Fax: +49.721.98520-99 */
/******************************************************************************/

#ifndef __USERSTEER_RTW_WRAP_H__
#define __USERSTEER_RTW_WRAP_H__

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
# define rtModel_UserSteer_RTW          rtModel_UserSteer_RTW_T
# define ExternalInputs_UserSteer_RTW   ExtU_UserSteer_RTW_T
# define ExternalOutputs_UserSteer_RTW  ExtY_UserSteer_RTW_T
#endif

#ifndef UserSteer_RTW_rtModel
typedef struct rtModel_UserSteer_RTW rtModel_UserSteer_RTW;
#endif

/* Model registration function */
rtModel_UserSteer_RTW *UserSteer_RTW (struct tInfos *Inf);

#if !defined(CM4SLDS) && defined(MATSUPP_NUMVER) && MATSUPP_NUMVER >= 70900
void rt_ODECreateIntegrationData (RTWSolverInfo *si);
void rt_ODEUpdateContinuousStates(RTWSolverInfo *si);
void rt_ODEDestroyIntegrationData(RTWSolverInfo *si);
#endif


/* Dictionary variables and other items of the model */
extern struct tMatSuppDictDef *UserSteer_RTW_DictDefines;
extern struct tMdlBdyFrame *UserSteer_RTW_BdyFrameDefines;


/* Wrapper functions */
void UserSteer_RTW_SetParams (rtModel_UserSteer_RTW *rts,
			struct tMatSuppTunables *tuns,
			struct tInfos *Inf);
int UserSteer_RTW_Register (void);


#ifdef __cplusplus
}
#endif

#endif /* __USERSTEER_RTW_WRAP_H__ */
