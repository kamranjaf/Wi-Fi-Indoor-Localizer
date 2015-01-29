/*
 * File: LocalizationCore_private.h
 *
 * Code generated for Simulink model 'LocalizationCore'.
 *
 * Model version                  : 1.942
 * Simulink Coder version         : 8.4 (R2013a) 13-Feb-2013
 * TLC version                    : 8.4 (Jan 19 2013)
 * C/C++ source code generated on : Thu Jan 29 13:15:25 2015
 *
 * Target selection: ert_shrlib.tlc
 * Embedded hardware selection: ARM Compatible->ARM 10
 * Emulation hardware selection:
 *    Differs from embedded hardware (MATLAB Host)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_LocalizationCore_private_h_
#define RTW_HEADER_LocalizationCore_private_h_
#include "rtwtypes.h"
#ifndef __RTWTYPES_H__
#error This file requires rtwtypes.h to be included
#else
#ifdef TMWTYPES_PREVIOUSLY_INCLUDED
#error This file requires rtwtypes.h to be included before tmwtypes.h
#else

/* Check for inclusion of an incorrect version of rtwtypes.h */
#ifndef RTWTYPES_ID_C08S16I32L32N32F1
#error This code was generated with a different "rtwtypes.h" than the file included
#endif                                 /* RTWTYPES_ID_C08S16I32L32N32F1 */
#endif                                 /* TMWTYPES_PREVIOUSLY_INCLUDED */
#endif                                 /* __RTWTYPES_H__ */

extern real_T rt_roundd(real_T u);
extern void LocalizationCor_MATLABFunction4(const real32_T rtu_u[36],
  B_MATLABFunction4_Localizatio_T *localB);
extern void LocalizationCor_MATLABFunction5(const real32_T rtu_u[3],
  B_MATLABFunction5_Localizatio_T *localB);

#endif                                 /* RTW_HEADER_LocalizationCore_private_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
