/*
 * File: LocalizationCore.h
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

#ifndef RTW_HEADER_LocalizationCore_h_
#define RTW_HEADER_LocalizationCore_h_
#ifndef LocalizationCore_COMMON_INCLUDES_
# define LocalizationCore_COMMON_INCLUDES_
#include <math.h>
#include <stddef.h>
#include <string.h>
#include "rtwtypes.h"
#endif                                 /* LocalizationCore_COMMON_INCLUDES_ */

#include "LocalizationCore_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

/* Block signals for system '<S38>/MATLAB Function4' */
typedef struct {
  real32_T y;                          /* '<S38>/MATLAB Function4' */
} B_MATLABFunction4_Localizatio_T;

/* Block signals for system '<S38>/MATLAB Function5' */
typedef struct {
  int32_T y[3];                        /* '<S38>/MATLAB Function5' */
} B_MATLABFunction5_Localizatio_T;

/* Block signals (auto storage) */
typedef struct {
  real_T wifi_int[2048];
  real_T indices[2048];
  real_T test_means[2048];
  int32_T iidx[2048];
  int32_T b_iidx[2048];
  real32_T rss_new[2000];
  real32_T DigitalFilter;              /* '<S14>/Digital Filter' */
  real32_T MultiportSwitch1;           /* '<S16>/Multiport Switch1' */
  real32_T accuracy;                   /* '<S50>/kNN Localization core' */
  real32_T MEAN_RSS[4194304];          /* '<S49>/MEAN_RSS' */
  real32_T N_AP;                       /* '<S49>/N_AP' */
  real32_T N_POINTS;                   /* '<S49>/N_POINTS' */
  real32_T POINT_LIST[6144];           /* '<S49>/POINT_LIST' */
  real32_T accuracy_f;                 /* '<S46>/CS Localization core' */
  real32_T MultiportSelector13_o2[3];  /* '<S39>/Multiport Selector13' */
  real32_T MultiportSelector13_o2_a[3];/* '<S38>/Multiport Selector13' */
  real32_T mean_rss[200];              /* '<S12>/MAC Filter and Merge' */
  real32_T mean_rss_e[100];            /* '<S11>/MAC Filter and Merge' */
  real32_T Coordinates[300];           /* '<S11>/MAC Filter and Merge' */
  real32_T TX_powers[100];             /* '<S11>/MAC Filter and Merge' */
  real32_T A_matrix[4];                /* '<S27>/A_matrix' */
  real32_T B_Vector[2];                /* '<S27>/B_Vector' */
  real32_T position[2];                /* '<S9>/MATLAB Function' */
  real32_T accuracy_n;                 /* '<S9>/MATLAB Function' */
  real32_T MultiportSwitch1_j;         /* '<S7>/Multiport Switch1' */
  real32_T accuracy_e;                 /* '<S22>/Trilateration core' */
  real32_T accuracy_g;                 /* '<S18>/Closest core' */
  real32_T accuracy_b;                 /* '<S17>/Avreage core' */
  real32_T Differentiate;              /* '<S8>/Differentiate' */
  real32_T SelectZAxis;                /* '<S14>/Select Z-Axis' */
  int32_T MultiportSwitch[3];          /* '<S16>/Multiport Switch' */
  int32_T position_out[3];             /* '<S50>/kNN Localization core' */
  int32_T position_out_i[3];           /* '<S46>/CS Localization core' */
  int32_T DataTypeConversion[3];       /* '<S39>/Data Type Conversion' */
  int32_T DataTypeConversion_n[3];     /* '<S38>/Data Type Conversion' */
  int32_T MultiportSwitch_c[3];        /* '<S7>/Multiport Switch' */
  int32_T position_out_n[3];           /* '<S22>/Trilateration core' */
  int32_T position_out_p[3];           /* '<S18>/Closest core' */
  int32_T position_out_m[3];           /* '<S17>/Avreage core' */
  uint8_T AP_LIST[34816];              /* '<S49>/AP_LIST' */
  uint8_T AP_list[3400];               /* '<S12>/MAC Filter and Merge' */
  B_MATLABFunction5_Localizatio_T sf_MATLABFunction5_n;/* '<S39>/MATLAB Function5' */
  B_MATLABFunction4_Localizatio_T sf_MATLABFunction4_i;/* '<S39>/MATLAB Function4' */
  B_MATLABFunction5_Localizatio_T sf_MATLABFunction5_h;/* '<S38>/MATLAB Function5' */
  B_MATLABFunction4_Localizatio_T sf_MATLABFunction4;/* '<S38>/MATLAB Function4' */
} B_LocalizationCore_T;

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  real32_T DigitalFilter_FILT_STATES[5];/* '<S13>/Digital Filter' */
  real32_T DiscreteTimeIntegrator_DSTATE;/* '<S14>/Discrete-Time Integrator' */
  real32_T LPF_FILT_STATES[50];        /* '<S8>/LPF' */
  real32_T Differentiate_FILT_STATES[2];/* '<S8>/Differentiate' */
  real32_T DigitalFilter_FILT_STATES_b[5];/* '<S14>/Digital Filter' */
  real32_T UnitDelay_DSTATE;           /* '<S14>/Unit Delay' */
  real32_T UnitDelay_DSTATE_i[6];      /* '<S39>/Unit Delay' */
  real32_T UnitDelay1_DSTATE[36];      /* '<S39>/Unit Delay1' */
  real32_T UnitDelay2_DSTATE;          /* '<S39>/Unit Delay2' */
  real32_T UnitDelay_DSTATE_f[6];      /* '<S38>/Unit Delay' */
  real32_T UnitDelay1_DSTATE_i[36];    /* '<S38>/Unit Delay1' */
  real32_T UnitDelay2_DSTATE_i;        /* '<S38>/Unit Delay2' */
  int32_T DigitalFilter_CIRCBUFFIDX;   /* '<S13>/Digital Filter' */
  int32_T LPF_CIRCBUFFIDX;             /* '<S8>/LPF' */
  int32_T Differentiate_CIRCBUFFIDX;   /* '<S8>/Differentiate' */
  int32_T DigitalFilter_CIRCBUFFIDX_n; /* '<S14>/Digital Filter' */
} DW_LocalizationCore_T;

/* Constant parameters (auto storage) */
typedef struct {
  /* Pooled Parameter (Expression: flatIdxVect)
   * Referenced by:
   *   '<S38>/Multiport Selector13'
   *   '<S39>/Multiport Selector13'
   */
  int32_T pooled2[6];

  /* Computed Parameter: MultiportSelector1_IN
   * Referenced by: '<S2>/Multiport Selector1'
   */
  int32_T MultiportSelector1_IN[4];

  /* Pooled Parameter (Expression: flatIdxVect)
   * Referenced by:
   *   '<S13>/Select Z-Axis'
   *   '<S14>/Select Z-Axis'
   */
  int32_T pooled4;
} ConstP_LocalizationCore_T;

/* External inputs (root inport signals with auto storage) */
typedef struct {
  real32_T Gyro[3];                    /* '<Root>/Gyro' */
  real32_T Accel[3];                   /* '<Root>/Accel' */
  real32_T Magnet[3];                  /* '<Root>/Magnet' */
  real32_T Gravity[3];                 /* '<Root>/Gravity' */
  real32_T LinearAccel[3];             /* '<Root>/LinearAccel' */
  real32_T RotationVector[3];          /* '<Root>/RotationVector' */
  real32_T GameRotation[3];            /* '<Root>/GameRotation' */
  real32_T Orientation[3];             /* '<Root>/Orientation' */
  real32_T RotationMatrix[9];          /* '<Root>/RotationMatrix' */
  real32_T Pressure;                   /* '<Root>/Pressure' */
  real32_T GPS[6];                     /* '<Root>/GPS' */
  real_T GPS_Status;                   /* '<Root>/GPS_Status' */
  real32_T Wifi_RSS[200];              /* '<Root>/Wifi_RSS' */
  uint8_T Wifi_MAC[3400];              /* '<Root>/Wifi_MAC' */
  real_T Wifi_Status;                  /* '<Root>/Wifi_Status' */
  real32_T Ble_RSS[100];               /* '<Root>/Ble_RSS' */
  uint8_T Ble_MAC[1700];               /* '<Root>/Ble_MAC' */
  real32_T Ble_Coordinates[300];       /* '<Root>/Ble_Coordinates' */
  real32_T Ble_TX_Powers[100];         /* '<Root>/Ble_TX_Powers' */
  real_T Ble_Status;                   /* '<Root>/Ble_Status' */
  real32_T Params[10];                 /* '<Root>/Params' */
} ExtU_LocalizationCore_T;

/* External outputs (root outports fed by signals with auto storage) */
typedef struct {
  int32_T Location[3];                 /* '<Root>/Location' */
  real32_T GeoLocation[3];             /* '<Root>/GeoLocation' */
  real32_T Accuracy;                   /* '<Root>/Accuracy' */
  real32_T Speed[3];                   /* '<Root>/Speed' */
  real32_T Steps;                      /* '<Root>/Steps' */
} ExtY_LocalizationCore_T;

/* Parameters (auto storage) */
struct P_LocalizationCore_T_ {
  real_T Constant_Value;               /* Expression: 0
                                        * Referenced by: '<S2>/Constant'
                                        */
  real_T Constant_Value_h;             /* Expression: 0
                                        * Referenced by: '<S10>/Constant'
                                        */
  real32_T Constant_Value_b;           /* Computed Parameter: Constant_Value_b
                                        * Referenced by: '<S20>/Constant'
                                        */
  real32_T Constant_Value_j;           /* Computed Parameter: Constant_Value_j
                                        * Referenced by: '<S19>/Constant'
                                        */
  real32_T Constant_Value_o;           /* Computed Parameter: Constant_Value_o
                                        * Referenced by: '<S21>/Constant'
                                        */
  real32_T A_matrix_Value[4];          /* Computed Parameter: A_matrix_Value
                                        * Referenced by: '<S27>/A_matrix'
                                        */
  real32_T B_Vector_Value[2];          /* Computed Parameter: B_Vector_Value
                                        * Referenced by: '<S27>/B_Vector'
                                        */
  real32_T UnitDelay_InitialCondition[6];/* Computed Parameter: UnitDelay_InitialCondition
                                          * Referenced by: '<S38>/Unit Delay'
                                          */
  real32_T UnitDelay1_InitialCondition[36];/* Computed Parameter: UnitDelay1_InitialCondition
                                            * Referenced by: '<S38>/Unit Delay1'
                                            */
  real32_T UnitDelay2_InitialCondition;/* Expression: single(1)
                                        * Referenced by: '<S38>/Unit Delay2'
                                        */
  real32_T _Value;                     /* Computed Parameter: _Value
                                        * Referenced by: '<S39>/ '
                                        */
  real32_T UnitDelay_InitialCondition_i[6];/* Computed Parameter: UnitDelay_InitialCondition_i
                                            * Referenced by: '<S39>/Unit Delay'
                                            */
  real32_T UnitDelay1_InitialCondition_h[36];/* Computed Parameter: UnitDelay1_InitialCondition_h
                                              * Referenced by: '<S39>/Unit Delay1'
                                              */
  real32_T UnitDelay2_InitialCondition_k;/* Expression: single(1)
                                          * Referenced by: '<S39>/Unit Delay2'
                                          */
  real32_T MEAN_RSS_Value[4194304];    /* Computed Parameter: MEAN_RSS_Value
                                        * Referenced by: '<S49>/MEAN_RSS'
                                        */
  real32_T N_AP_Value;                 /* Computed Parameter: N_AP_Value
                                        * Referenced by: '<S49>/N_AP'
                                        */
  real32_T N_POINTS_Value;             /* Computed Parameter: N_POINTS_Value
                                        * Referenced by: '<S49>/N_POINTS'
                                        */
  real32_T POINT_LIST_Value[6144];     /* Computed Parameter: POINT_LIST_Value
                                        * Referenced by: '<S49>/POINT_LIST'
                                        */
  real32_T DigitalFilter_RTP1COEFF[5]; /* Computed Parameter: DigitalFilter_RTP1COEFF
                                        * Referenced by: '<S13>/Digital Filter'
                                        */
  real32_T Constant_Value_ja;          /* Computed Parameter: Constant_Value_ja
                                        * Referenced by: '<S31>/Constant'
                                        */
  real32_T DiscreteTimeIntegrator_gainval;/* Computed Parameter: DiscreteTimeIntegrator_gainval
                                           * Referenced by: '<S14>/Discrete-Time Integrator'
                                           */
  real32_T DiscreteTimeIntegrator_IC;  /* Computed Parameter: DiscreteTimeIntegrator_IC
                                        * Referenced by: '<S14>/Discrete-Time Integrator'
                                        */
  real32_T Gain_Gain;                  /* Computed Parameter: Gain_Gain
                                        * Referenced by: '<S14>/Gain'
                                        */
  real32_T LPF_RTP1COEFF[50];          /* Computed Parameter: LPF_RTP1COEFF
                                        * Referenced by: '<S8>/LPF'
                                        */
  real32_T Differentiate_RTP1COEFF[2]; /* Computed Parameter: Differentiate_RTP1COEFF
                                        * Referenced by: '<S8>/Differentiate'
                                        */
  real32_T Gain_Gain_m;                /* Computed Parameter: Gain_Gain_m
                                        * Referenced by: '<S8>/Gain'
                                        */
  real32_T Constant_Value_jk[2];       /* Computed Parameter: Constant_Value_jk
                                        * Referenced by: '<S15>/Constant'
                                        */
  real32_T DigitalFilter_RTP1COEFF_f[5];/* Computed Parameter: DigitalFilter_RTP1COEFF_f
                                         * Referenced by: '<S14>/Digital Filter'
                                         */
  real32_T UnitDelay_InitialCondition_g;/* Expression: single(0)
                                         * Referenced by: '<S14>/Unit Delay'
                                         */
  int32_T Constant_Value_f;            /* Computed Parameter: Constant_Value_f
                                        * Referenced by: '<S48>/Constant'
                                        */
  int32_T Constant_Value_a;            /* Computed Parameter: Constant_Value_a
                                        * Referenced by: '<S47>/Constant'
                                        */
  int32_T Constant_Value_l;            /* Computed Parameter: Constant_Value_l
                                        * Referenced by: '<S36>/Constant'
                                        */
  int32_T Constant_Value_lr;           /* Computed Parameter: Constant_Value_lr
                                        * Referenced by: '<S37>/Constant'
                                        */
  uint8_T AP_LIST_Value[34816];        /* Expression: AP_list
                                        * Referenced by: '<S49>/AP_LIST'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_LocalizationCore_T {
  const char_T * volatile errorStatus;
};

/* Block parameters (auto storage) */
extern P_LocalizationCore_T LocalizationCore_P;

/* Block signals (auto storage) */
extern B_LocalizationCore_T LocalizationCore_B;

/* Block states (auto storage) */
extern DW_LocalizationCore_T LocalizationCore_DW;

/* External inputs (root inport signals with auto storage) */
extern ExtU_LocalizationCore_T LocalizationCore_U;

/* External outputs (root outports fed by signals with auto storage) */
extern ExtY_LocalizationCore_T LocalizationCore_Y;

/* Constant parameters (auto storage) */
extern const ConstP_LocalizationCore_T LocalizationCore_ConstP;

/* Model entry point functions */
extern void LocalizationCore_initialize(void);
extern void LocalizationCore_step(void);

/* Real-time Model object */
extern RT_MODEL_LocalizationCore_T *const LocalizationCore_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('Simulink_Android/LocalizationCore')    - opens subsystem Simulink_Android/LocalizationCore
 * hilite_system('Simulink_Android/LocalizationCore/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'Simulink_Android'
 * '<S1>'   : 'Simulink_Android/LocalizationCore'
 * '<S2>'   : 'Simulink_Android/LocalizationCore/Core'
 * '<S3>'   : 'Simulink_Android/LocalizationCore/MATLAB Function1'
 * '<S4>'   : 'Simulink_Android/LocalizationCore/MATLAB Function2'
 * '<S5>'   : 'Simulink_Android/LocalizationCore/MATLAB Function5'
 * '<S6>'   : 'Simulink_Android/LocalizationCore/Transpose'
 * '<S7>'   : 'Simulink_Android/LocalizationCore/Core/BLE Localizer'
 * '<S8>'   : 'Simulink_Android/LocalizationCore/Core/Barometer Processor'
 * '<S9>'   : 'Simulink_Android/LocalizationCore/Core/GPS to Pixel'
 * '<S10>'  : 'Simulink_Android/LocalizationCore/Core/Geolocation Calculator'
 * '<S11>'  : 'Simulink_Android/LocalizationCore/Core/MAC Merge and Filter BLE'
 * '<S12>'  : 'Simulink_Android/LocalizationCore/Core/MAC Merge and Filter WiFi'
 * '<S13>'  : 'Simulink_Android/LocalizationCore/Core/Reset Generator'
 * '<S14>'  : 'Simulink_Android/LocalizationCore/Core/Step Counter'
 * '<S15>'  : 'Simulink_Android/LocalizationCore/Core/Tracking Module'
 * '<S16>'  : 'Simulink_Android/LocalizationCore/Core/Wifi Localizer'
 * '<S17>'  : 'Simulink_Android/LocalizationCore/Core/BLE Localizer/ Average Localizer'
 * '<S18>'  : 'Simulink_Android/LocalizationCore/Core/BLE Localizer/Closest Localizer'
 * '<S19>'  : 'Simulink_Android/LocalizationCore/Core/BLE Localizer/Compare To Constant'
 * '<S20>'  : 'Simulink_Android/LocalizationCore/Core/BLE Localizer/Compare To Constant1'
 * '<S21>'  : 'Simulink_Android/LocalizationCore/Core/BLE Localizer/Compare To Constant2'
 * '<S22>'  : 'Simulink_Android/LocalizationCore/Core/BLE Localizer/Trilateration Localizer'
 * '<S23>'  : 'Simulink_Android/LocalizationCore/Core/BLE Localizer/ Average Localizer/Avreage core'
 * '<S24>'  : 'Simulink_Android/LocalizationCore/Core/BLE Localizer/Closest Localizer/Closest core'
 * '<S25>'  : 'Simulink_Android/LocalizationCore/Core/BLE Localizer/Trilateration Localizer/Trilateration core'
 * '<S26>'  : 'Simulink_Android/LocalizationCore/Core/GPS to Pixel/MATLAB Function'
 * '<S27>'  : 'Simulink_Android/LocalizationCore/Core/Geolocation Calculator/Geolocation Data'
 * '<S28>'  : 'Simulink_Android/LocalizationCore/Core/Geolocation Calculator/MATLAB Function1'
 * '<S29>'  : 'Simulink_Android/LocalizationCore/Core/MAC Merge and Filter BLE/MAC Filter and Merge'
 * '<S30>'  : 'Simulink_Android/LocalizationCore/Core/MAC Merge and Filter WiFi/MAC Filter and Merge'
 * '<S31>'  : 'Simulink_Android/LocalizationCore/Core/Reset Generator/Compare To Constant'
 * '<S32>'  : 'Simulink_Android/LocalizationCore/Core/Reset Generator/Transpose1'
 * '<S33>'  : 'Simulink_Android/LocalizationCore/Core/Step Counter/MATLAB Function2'
 * '<S34>'  : 'Simulink_Android/LocalizationCore/Core/Step Counter/MATLAB Function3'
 * '<S35>'  : 'Simulink_Android/LocalizationCore/Core/Step Counter/Transpose1'
 * '<S36>'  : 'Simulink_Android/LocalizationCore/Core/Tracking Module/Compare To Constant'
 * '<S37>'  : 'Simulink_Android/LocalizationCore/Core/Tracking Module/Compare To Constant1'
 * '<S38>'  : 'Simulink_Android/LocalizationCore/Core/Tracking Module/KALMAN Filter'
 * '<S39>'  : 'Simulink_Android/LocalizationCore/Core/Tracking Module/Particle Filter'
 * '<S40>'  : 'Simulink_Android/LocalizationCore/Core/Tracking Module/KALMAN Filter/KALMAN Filter'
 * '<S41>'  : 'Simulink_Android/LocalizationCore/Core/Tracking Module/KALMAN Filter/MATLAB Function4'
 * '<S42>'  : 'Simulink_Android/LocalizationCore/Core/Tracking Module/KALMAN Filter/MATLAB Function5'
 * '<S43>'  : 'Simulink_Android/LocalizationCore/Core/Tracking Module/Particle Filter/KALMAN Filter'
 * '<S44>'  : 'Simulink_Android/LocalizationCore/Core/Tracking Module/Particle Filter/MATLAB Function4'
 * '<S45>'  : 'Simulink_Android/LocalizationCore/Core/Tracking Module/Particle Filter/MATLAB Function5'
 * '<S46>'  : 'Simulink_Android/LocalizationCore/Core/Wifi Localizer/CS Wifi Localizer'
 * '<S47>'  : 'Simulink_Android/LocalizationCore/Core/Wifi Localizer/Compare To Constant'
 * '<S48>'  : 'Simulink_Android/LocalizationCore/Core/Wifi Localizer/Compare To Constant1'
 * '<S49>'  : 'Simulink_Android/LocalizationCore/Core/Wifi Localizer/RadioMap'
 * '<S50>'  : 'Simulink_Android/LocalizationCore/Core/Wifi Localizer/kNN Localizer'
 * '<S51>'  : 'Simulink_Android/LocalizationCore/Core/Wifi Localizer/CS Wifi Localizer/CS Localization core'
 * '<S52>'  : 'Simulink_Android/LocalizationCore/Core/Wifi Localizer/kNN Localizer/kNN Localization core'
 */
#endif                                 /* RTW_HEADER_LocalizationCore_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
