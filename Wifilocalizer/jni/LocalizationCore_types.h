/*
 * File: LocalizationCore_types.h
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

#ifndef RTW_HEADER_LocalizationCore_types_h_
#define RTW_HEADER_LocalizationCore_types_h_
#include "rtwtypes.h"

/* Custom Type definition for MATLAB Function: '<S46>/CS Localization core' */
#ifndef struct_sKLpTDgVdadEcWmkZEqDlEH
#define struct_sKLpTDgVdadEcWmkZEqDlEH

struct sKLpTDgVdadEcWmkZEqDlEH
{
  uint32_T LT;
  uint32_T UT;
  uint32_T UHESS;
  uint32_T SYM;
  uint32_T POSDEF;
  uint32_T RECT;
  uint32_T TRANSA;
};

#endif                                 /*struct_sKLpTDgVdadEcWmkZEqDlEH*/

#ifndef typedef_sKLpTDgVdadEcWmkZEqDlEH_Local_T
#define typedef_sKLpTDgVdadEcWmkZEqDlEH_Local_T

typedef struct sKLpTDgVdadEcWmkZEqDlEH sKLpTDgVdadEcWmkZEqDlEH_Local_T;

#endif                                 /*typedef_sKLpTDgVdadEcWmkZEqDlEH_Local_T*/

#ifndef struct_s9s8BC13iTohZXRbLMSIDHE
#define struct_s9s8BC13iTohZXRbLMSIDHE

struct s9s8BC13iTohZXRbLMSIDHE
{
  boolean_T CaseSensitivity;
  boolean_T StructExpand;
  boolean_T PartialMatching;
};

#endif                                 /*struct_s9s8BC13iTohZXRbLMSIDHE*/

#ifndef typedef_s9s8BC13iTohZXRbLMSIDHE_Local_T
#define typedef_s9s8BC13iTohZXRbLMSIDHE_Local_T

typedef struct s9s8BC13iTohZXRbLMSIDHE s9s8BC13iTohZXRbLMSIDHE_Local_T;

#endif                                 /*typedef_s9s8BC13iTohZXRbLMSIDHE_Local_T*/

/* Parameters (auto storage) */
typedef struct P_LocalizationCore_T_ P_LocalizationCore_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_LocalizationCore_T RT_MODEL_LocalizationCore_T;

#endif                                 /* RTW_HEADER_LocalizationCore_types_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
