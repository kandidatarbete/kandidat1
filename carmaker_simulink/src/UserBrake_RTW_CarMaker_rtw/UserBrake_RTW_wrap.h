/***************************************************** target specific file ***/
/*  Wrapper module for Simulink models                                        */
/*  ------------------------------------------------------------------------  */
/*  Copyright (c) IPG Automotive GmbH      www.ipg.de   Fon: +49.721.98520-0  */
/*  Bannwaldallee 60      D-76185 Karlsruhe   Germany   Fax: +49.721.98520-99 */
/******************************************************************************/

#ifndef __USERBRAKE_RTW_WRAP_H__
#define __USERBRAKE_RTW_WRAP_H__

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
#ifdef CLASSIC_INTERFACE
# define rtModel_UserBrake_RTW          rtModel_UserBrake_RTW_T
#else
# define rtModel_UserBrake_RTW          tag_rtM_UserBrake_RTW_T
#endif //CLASSIC_INTERFACE
# define ExternalInputs_UserBrake_RTW   ExtU_UserBrake_RTW_T
# define ExternalOutputs_UserBrake_RTW  ExtY_UserBrake_RTW_T
#endif

#ifndef UserBrake_RTW_rtModel
typedef struct rtModel_UserBrake_RTW rtModel_UserBrake_RTW;
#endif

/* Model registration function */
rtModel_UserBrake_RTW *UserBrake_RTW (struct tInfos *Inf);

#if !defined(CM4SLDS) && defined(MATSUPP_NUMVER) && MATSUPP_NUMVER >= 70900
void rt_ODECreateIntegrationData (RTWSolverInfo *si);
void rt_ODEUpdateContinuousStates(RTWSolverInfo *si);
void rt_ODEDestroyIntegrationData(RTWSolverInfo *si);
#endif


/* Dictionary variables and other items of the model */
extern struct tMatSuppDictDef *UserBrake_RTW_DictDefines;
extern struct tMdlBdyFrame *UserBrake_RTW_BdyFrameDefines;


/* Wrapper functions */
void UserBrake_RTW_SetParams (rtModel_UserBrake_RTW *rtm,
			struct tMatSuppTunables *tuns,
			struct tInfos *Inf);
int UserBrake_RTW_Register (void);


#ifdef __cplusplus
}
#endif

#endif /* __USERBRAKE_RTW_WRAP_H__ */

