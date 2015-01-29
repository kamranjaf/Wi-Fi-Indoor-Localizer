/*
 * File: LocalizationCore.c
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

#include "LocalizationCore.h"
#include "LocalizationCore_private.h"

/* user code (top of source file) */
#include <jni.h>

void JNICALL Java_com_wifilocalizer_LocalizationCore_step(JNIEnv * env,
  jobject obj,jfloatArray gyro,jfloatArray accel,jfloatArray magnet,jfloatArray
  gravity,
  jfloatArray linearaccel,jfloatArray rotationvector,jfloatArray gamerotation,
  jfloatArray orientation,
  jfloatArray rotationmatrix,jfloat pressure,jfloatArray gps,jfloat gps_status,
  jfloatArray wifi_rss,jbyteArray wifi_mac,jfloat wifi_status,
  jfloatArray ble_rss,jbyteArray ble_mac,jfloatArray ble_coordinates,jfloatArray
  ble_tx_powers,jfloat ble_status,jfloatArray params)
{
  int i;
  jsize gyro_len = (*env)->GetArrayLength(env, gyro);
  jsize accel_len = (*env)->GetArrayLength(env, accel);
  jsize magnet_len = (*env)->GetArrayLength(env, magnet);
  jsize gravity_len = (*env)->GetArrayLength(env, gravity);
  jsize linearaccel_len = (*env)->GetArrayLength(env, linearaccel);
  jsize rotationvector_len = (*env)->GetArrayLength(env, rotationvector);
  jsize gamerotation_len = (*env)->GetArrayLength(env, gamerotation);
  jsize orientation_len = (*env)->GetArrayLength(env, orientation);
  jsize rotationmatrix_len = (*env)->GetArrayLength(env, rotationmatrix);
  jsize gps_len = (*env)->GetArrayLength(env, gps);
  jsize wifi_rss_len = (*env)->GetArrayLength(env, wifi_rss);
  jsize wifi_mac_len = (*env)->GetArrayLength(env, wifi_mac);
  jsize ble_rss_len = (*env)->GetArrayLength(env, ble_rss);
  jsize ble_mac_len = (*env)->GetArrayLength(env, ble_mac);
  jsize ble_coordinates_len = (*env)->GetArrayLength(env, ble_coordinates);
  jsize ble_tx_powers_len = (*env)->GetArrayLength(env, ble_tx_powers);
  jsize params_len = (*env)->GetArrayLength(env, params);
  jfloat *gyro_ptr = (*env)->GetFloatArrayElements(env, gyro, 0);
  jfloat *accel_ptr = (*env)->GetFloatArrayElements(env, accel, 0);
  jfloat *magnet_ptr = (*env)->GetFloatArrayElements(env, magnet, 0);
  jfloat *gravity_ptr = (*env)->GetFloatArrayElements(env, gravity, 0);
  jfloat *linearaccel_ptr = (*env)->GetFloatArrayElements(env, linearaccel, 0);
  jfloat *rotationvector_ptr = (*env)->GetFloatArrayElements(env, rotationvector,
    0);
  jfloat *gamerotation_ptr = (*env)->GetFloatArrayElements(env, gamerotation, 0);
  jfloat *orientation_ptr = (*env)->GetFloatArrayElements(env, orientation, 0);
  jfloat *rotationmatrix_ptr = (*env)->GetFloatArrayElements(env, rotationmatrix,
    0);
  jfloat *gps_ptr = (*env)->GetFloatArrayElements(env, gps, 0);
  jfloat *wifi_rss_ptr = (*env)->GetFloatArrayElements(env, wifi_rss, 0);
  jbyte *wifi_mac_ptr = (*env)->GetByteArrayElements(env, wifi_mac, 0);
  jfloat *ble_rss_ptr = (*env)->GetFloatArrayElements(env, ble_rss, 0);
  jbyte *ble_mac_ptr = (*env)->GetByteArrayElements(env, ble_mac, 0);
  jfloat *ble_coordinates_ptr = (*env)->GetFloatArrayElements(env,
    ble_coordinates, 0);
  jfloat *ble_tx_powers_ptr = (*env)->GetFloatArrayElements(env, ble_tx_powers,
    0);
  jfloat *params_ptr = (*env)->GetFloatArrayElements(env, params, 0);
  for (i=0;i<gyro_len;i++) {
    LocalizationCore_U.Gyro[i]= *(gyro_ptr+i);
  }

  for (i=0;i<accel_len;i++) {
    LocalizationCore_U.Accel[i]= *(accel_ptr+i);
  }

  for (i=0;i<magnet_len;i++) {
    LocalizationCore_U.Magnet[i]= *(magnet_ptr+i);
  }

  for (i=0;i<gravity_len;i++) {
    LocalizationCore_U.Gravity[i]= *(gravity_ptr+i);
  }

  for (i=0;i<linearaccel_len;i++) {
    LocalizationCore_U.LinearAccel[i]= *(linearaccel_ptr+i);
  }

  for (i=0;i<gamerotation_len;i++) {
    LocalizationCore_U.GameRotation[i]= *(gamerotation_ptr+i);
  }

  for (i=0;i<orientation_len;i++) {
    LocalizationCore_U.Orientation[i]= *(orientation_ptr+i);
  }

  for (i=0;i<rotationmatrix_len;i++) {
    LocalizationCore_U.RotationMatrix[i]= *(rotationmatrix_ptr+i);
  }

  for (i=0;i<gps_len;i++) {
    LocalizationCore_U.GPS[i]= *(gps_ptr+i);
  }

  for (i=0;i<wifi_rss_len;i++) {
    LocalizationCore_U.Wifi_RSS[i]= *(wifi_rss_ptr+i);
  }

  for (i=0;i<wifi_mac_len;i++) {
    LocalizationCore_U.Wifi_MAC[i]= *(wifi_mac_ptr+i);
  }

  for (i=0;i<ble_rss_len;i++) {
    LocalizationCore_U.Ble_RSS[i]= *(ble_rss_ptr+i);
  }

  for (i=0;i<ble_mac_len;i++) {
    LocalizationCore_U.Ble_MAC[i]= *(ble_mac_ptr+i);
  }

  for (i=0;i<ble_coordinates_len;i++) {
    LocalizationCore_U.Ble_Coordinates[i]= *(ble_coordinates_ptr+i);
  }

  for (i=0;i<ble_tx_powers_len;i++) {
    LocalizationCore_U.Ble_TX_Powers[i]= *(ble_tx_powers_ptr+i);
  }

  for (i=0;i<params_len;i++) {
    LocalizationCore_U.Params[i]= *(params_ptr+i);
  }

  LocalizationCore_U.Pressure= pressure;
  LocalizationCore_U.GPS_Status= gps_status;
  LocalizationCore_U.Wifi_Status= wifi_status;
  LocalizationCore_U.Ble_Status= ble_status;
  LocalizationCore_step();
  (*env)->ReleaseFloatArrayElements(env, gyro, gyro_ptr, 0);
  (*env)->ReleaseFloatArrayElements(env, accel, accel_ptr, 0);
  (*env)->ReleaseFloatArrayElements(env, magnet, magnet_ptr, 0);
  (*env)->ReleaseFloatArrayElements(env, gravity, gravity_ptr, 0);
  (*env)->ReleaseFloatArrayElements(env, linearaccel, linearaccel_ptr, 0);
  (*env)->ReleaseFloatArrayElements(env, rotationvector, rotationvector_ptr, 0);
  (*env)->ReleaseFloatArrayElements(env, gamerotation, gamerotation_ptr, 0);
  (*env)->ReleaseFloatArrayElements(env, orientation, orientation_ptr, 0);
  (*env)->ReleaseFloatArrayElements(env, rotationmatrix, rotationmatrix_ptr, 0);
  (*env)->ReleaseFloatArrayElements(env, gps, gps_ptr, 0);
  (*env)->ReleaseFloatArrayElements(env, wifi_rss, wifi_rss_ptr, 0);
  (*env)->ReleaseByteArrayElements(env, wifi_mac, wifi_mac_ptr, 0);
  (*env)->ReleaseFloatArrayElements(env, ble_rss, ble_rss_ptr, 0);
  (*env)->ReleaseByteArrayElements(env, ble_mac, ble_mac_ptr, 0);
  (*env)->ReleaseFloatArrayElements(env, ble_coordinates, ble_coordinates_ptr, 0);
  (*env)->ReleaseFloatArrayElements(env, ble_tx_powers, ble_tx_powers_ptr, 0);
  (*env)->ReleaseFloatArrayElements(env, params, params_ptr, 0);
}

jintArray JNICALL Java_com_wifilocalizer_LocalizationCore_getlocation(JNIEnv
  * env,jobject obj)
{
  jintArray result = (*env) -> NewIntArray (env, 3);
  (*env)->SetIntArrayRegion(env, result, 0, 3, LocalizationCore_Y.Location);
  return result;
}

jfloatArray JNICALL Java_com_wifilocalizer_LocalizationCore_getgeolocation
  (JNIEnv
   * env,jobject obj)
{
  jfloatArray result = (*env) -> NewFloatArray (env, 3);
  (*env)->SetFloatArrayRegion(env, result, 0, 3, LocalizationCore_Y.GeoLocation);
  return result;
}

jfloat JNICALL Java_com_wifilocalizer_LocalizationCore_getaccuracy(JNIEnv
  * env,jobject obj)
{
  return LocalizationCore_Y.Accuracy;
}

jfloatArray JNICALL Java_com_wifilocalizer_LocalizationCore_getspeed(JNIEnv
  * env,jobject obj)
{
  jfloatArray result = (*env) -> NewFloatArray (env, 3);
  (*env)->SetFloatArrayRegion(env, result, 0, 3, LocalizationCore_Y.Speed);
  return result;
}

jfloat JNICALL Java_com_wifilocalizer_LocalizationCore_getsteps(JNIEnv
  * env,jobject obj)
{
  return LocalizationCore_Y.Steps;
}

void JNICALL Java_com_wifilocalizer_LocalizationCore_initialize(JNIEnv * env,
  jobject obj)
{
  LocalizationCore_initialize();
}

void JNICALL Java_com_wifilocalizer_LocalizationCore_InjectRadioMap(JNIEnv * env,
  jobject obj, jbyteArray AP_List, jfloatArray mean_rss, jfloatArray pointList,
  jfloat n_points, jfloat n_ap,
  jfloatArray A, jfloatArray B)
{
  int i;
  jsize AP_List_len = (*env)->GetArrayLength(env, AP_List);
  jsize mean_rss_len = (*env)->GetArrayLength(env, mean_rss);
  jsize pointList_len = (*env)->GetArrayLength(env, pointList);
  jsize A_len = (*env)->GetArrayLength(env, A);
  jsize B_len = (*env)->GetArrayLength(env, B);
  jbyte *AP_List_ptr = (*env)->GetByteArrayElements(env, AP_List, 0);
  jfloat *mean_rss_ptr = (*env)->GetFloatArrayElements(env, mean_rss, 0);
  jfloat *pointList_ptr = (*env)->GetFloatArrayElements(env, pointList, 0);
  jfloat *A_ptr = (*env)->GetFloatArrayElements(env, A, 0);
  jfloat *B_ptr = (*env)->GetFloatArrayElements(env, B, 0);
  for (i=0;i<17;i++) {
    memcpy(&LocalizationCore_B.AP_LIST[i*2048], AP_List_ptr+i*((int)n_ap), ((int)
            n_ap) * sizeof(uint8_T));
  }

  for (i=0;i<((int)n_points);i++) {
    memcpy(&LocalizationCore_B.MEAN_RSS[i*2048], mean_rss_ptr+i*((int)n_ap),
           ((int)n_ap) * sizeof(real32_T));
  }

  for (i=0;i<3;i++) {
    memcpy(&LocalizationCore_B.POINT_LIST[i*2048], pointList_ptr+i*((int)
            n_points), ((int)n_points) * sizeof(real32_T));
  }

  LocalizationCore_B.A_matrix[0] = *(A_ptr);
  LocalizationCore_B.A_matrix[1] = *(A_ptr+1);
  LocalizationCore_B.A_matrix[2] = *(A_ptr+2);
  LocalizationCore_B.A_matrix[3] = *(A_ptr+3);
  LocalizationCore_B.B_Vector[0] = *(B_ptr);
  LocalizationCore_B.B_Vector[1] = *(B_ptr+1);
  LocalizationCore_B.N_AP = n_ap;
  LocalizationCore_B.N_POINTS = n_points;
  (*env)->ReleaseByteArrayElements(env, AP_List, AP_List_ptr, 0);
  (*env)->ReleaseFloatArrayElements(env, mean_rss, mean_rss_ptr, 0);
  (*env)->ReleaseFloatArrayElements(env, pointList, pointList_ptr, 0);
  (*env)->ReleaseFloatArrayElements(env, A, A_ptr, 0);
  (*env)->ReleaseFloatArrayElements(env, B, B_ptr, 0);
}

/* Block signals (auto storage) */
B_LocalizationCore_T LocalizationCore_B;

/* Block states (auto storage) */
DW_LocalizationCore_T LocalizationCore_DW;

/* External inputs (root inport signals with auto storage) */
ExtU_LocalizationCore_T LocalizationCore_U;

/* External outputs (root outports fed by signals with auto storage) */
ExtY_LocalizationCore_T LocalizationCore_Y;

/* Real-time model */
RT_MODEL_LocalizationCore_T LocalizationCore_M_;
RT_MODEL_LocalizationCore_T *const LocalizationCore_M = &LocalizationCore_M_;

/* Forward declaration for local functions */
static void LocalizationCore_eml_sort_l(const real_T x[2048], real_T y[2048],
  int32_T idx[2048]);
static void LocalizationCore_eml_sort_lvl(const real_T x[2048], real_T y[2048],
  int32_T idx[2048]);
static void LocalizationCore_eml_sort_lv(const real_T x[2048], real_T y[2048],
  int32_T idx[2048]);
static real32_T LocalizationCore_eml_xnrm2(int32_T n, const real32_T x[150],
  int32_T ix0);
static real32_T LocalizationCore_eml_div(real_T x, real32_T y);
static void LocalizationCore_eml_xscal(int32_T n, real32_T a, real32_T x[150],
  int32_T ix0);
static real32_T LocalizationCore_eml_xnrm2_i(int32_T n, const real32_T x[50],
  int32_T ix0);
static void LocalizationCore_eml_xscal_m(int32_T n, real32_T a, real32_T x[50],
  int32_T ix0);
static real32_T LocalizationCore_eml_div_n(real32_T x, real32_T y);
static void LocalizationCore_eml_xscal_mg(real32_T a, real32_T x[9], int32_T ix0);
static void LocalizationCore_eml_xswap(real32_T x[9], int32_T ix0, int32_T iy0);
static void LocalizationCore_eml_xrotg(real32_T *a, real32_T *b, real32_T *c,
  real32_T *s);
static void LocalizationCore_eml_xrot(real32_T x[9], int32_T ix0, int32_T iy0,
  real32_T c, real32_T s);
static real32_T LocalizationCore_eml_xdotc_a(int32_T n, const real32_T x[9],
  int32_T ix0, const real32_T y[9], int32_T iy0);
static void LocalizationCore_eml_xaxpy_eck(int32_T n, real32_T a, int32_T ix0,
  real32_T y[9], int32_T iy0);
static void LocalizationCore_eml_xaxpy_ec(int32_T n, real32_T a, const real32_T
  x[3], int32_T ix0, real32_T y[150], int32_T iy0);
static void LocalizationCore_eml_xaxpy_e(int32_T n, real32_T a, const real32_T
  x[150], int32_T ix0, real32_T y[3], int32_T iy0);
static real32_T LocalizationCore_eml_xdotc(int32_T n, const real32_T x[150],
  int32_T ix0, const real32_T y[150], int32_T iy0);
static void LocalizationCore_eml_xaxpy(int32_T n, real32_T a, int32_T ix0,
  real32_T y[150], int32_T iy0);
static void LocalizationCore_eml_xgesvd(const real32_T A[150], real32_T U[9],
  real32_T S[3]);
static void LocalizationCore_orth(const real32_T A[150], real32_T Q_data[9],
  int32_T Q_sizes[2]);
static real32_T LocalizationCore_eml_xnrm2_in(int32_T n, const real32_T x[150],
  int32_T ix0);
static void LocalizationCore_eml_xscal_mg3(int32_T n, real32_T a, real32_T x[150],
  int32_T ix0);
static real32_T LocalizationCore_eml_xnrm2_ina(const real32_T x[3], int32_T ix0);
static void LocalizationCore_eml_xscal_mg3o(real32_T a, real32_T x[3], int32_T
  ix0);
static void LocalizationCor_eml_xscal_mg3or(real32_T a, real32_T x[150], int32_T
  ix0);
static void LocalizationCore_eml_xswap_b(real32_T x[150], int32_T ix0, int32_T
  iy0);
static void LocalizationCore_eml_xrot_d(real32_T x[150], int32_T ix0, int32_T
  iy0, real32_T c, real32_T s);
static real32_T LocalizationCore_eml_xdotc_a5(int32_T n, const real32_T x[150],
  int32_T ix0, const real32_T y[150], int32_T iy0);
static void LocalizationCore_eml_xaxpy_eckn(int32_T n, real32_T a, int32_T ix0,
  real32_T y[150], int32_T iy0);
static void LocalizationCo_eml_xaxpy_eckne4(int32_T n, real32_T a, const
  real32_T x[50], int32_T ix0, real32_T y[150], int32_T iy0);
static void LocalizationCor_eml_xaxpy_eckne(int32_T n, real32_T a, const
  real32_T x[150], int32_T ix0, real32_T y[50], int32_T iy0);
static void LocalizationCore_eml_xgesvd_l(const real32_T A[150], real32_T U[150],
  real32_T S[3], real32_T V[9]);
static void LocalizationCore_pinv(const real32_T A[150], real32_T X[150]);
static void LocalizationCore_abs(const real_T x[3], real_T y[3]);
static void LocalizationCore_rdivide(real_T x, const real_T y[3], real_T z[3]);
static real_T LocalizationCore_norm_c(const real_T x_data[15], const int32_T
  *x_sizes);
static void LocalizationCore_rdivide_b(const real_T x[3], const real_T y[3],
  real_T z[3]);
static void LocalizationCore_power(const real_T a[3], real_T y[3]);
static void LocalizationCore_diag(const real_T v[3], real_T d[9]);
static void Locali_eml_signed_integer_colon(int32_T b, int32_T y_data[3],
  int32_T y_sizes[2]);
static void LocalizationCore_eml_xtrsv(int32_T n, const real_T A_data[9], const
  int32_T A_sizes[2], int32_T lda, real_T x_data[3]);
static real_T LocalizationCore_eml_rcond(const real_T A_data[9], const int32_T
  A_sizes[2], real_T normA);
static void LocalizationCore_eml_lusolve(const real_T A_data[9], const int32_T
  A_sizes[2], const real_T B_data[3], const int32_T *B_sizes, real_T X_data[3],
  int32_T *X_sizes, real_T *rc);
static real_T LocalizationCore_eml_xnrm2_inai(int32_T n, const real_T x_data[9],
  int32_T ix0);
static real_T LocalizationC_eml_matlab_zlarfg(int32_T n, real_T *alpha1, real_T
  x_data[9], int32_T ix0);
static void LocalizationCore_eml_qrsolve(const real_T A_data[9], const int32_T
  A_sizes[2], real_T B_data[3], real_T Y_data[3], int32_T *Y_sizes, real_T
  *rankR);
static void LocalizationCore_linsolve(const real_T A_data[9], const int32_T
  A_sizes[2], const real_T B_data[3], const int32_T *B_sizes, real_T C_data[3],
  int32_T *C_sizes, real_T *r);
static void LocalizationCore_l1eq_pd(real_T x0[3], const real_T A_data[9], const
  int32_T A_sizes[2], const real_T b_data[3]);
static void LocalizationCore_eml_sort_lvl1f(const real32_T x[3], real32_T y[3],
  int32_T idx[3]);
static void LocalizationCore_eml_sort_lvl1(const real32_T x[3], real32_T y[3],
  int32_T idx[3]);
static real32_T LocalizationCore_sum(const real32_T x[3]);
static void LocalizationCore_eml_sort_d(const real_T x[2048], real_T y[2048],
  int32_T idx[2048]);
static void LocalizationCore_eml_sort_h(const real32_T x[100], real32_T y[100],
  int32_T idx[100]);
static void LocalizationCore_eml_sort(const real32_T x[100], real32_T y[100],
  int32_T idx[100]);
static real32_T LocalizationCore_norm(const real32_T x[3]);
static boolean_T LocalizationCore_isequaln_g(const uint8_T varargin_1[17], const
  uint8_T varargin_2[17]);
static boolean_T LocalizationCore_isequaln(const uint8_T varargin_1[8], const
  uint8_T varargin_2[8]);
static boolean_T LocalizationCore_isequaln_k(const uint8_T varargin_1[17], const
  uint8_T varargin_2[17]);
static boolean_T LocalizationCore_isequaln_gd(const uint8_T varargin_1[17],
  const uint8_T varargin_2[17]);
static boolean_T LocalizationCore_isequaln_ig(const uint8_T varargin_1[17],
  const uint8_T varargin_2[17]);
static boolean_T LocalizationCore_isequaln_i(const uint8_T varargin_1[8], const
  uint8_T varargin_2[8]);

/*
 * Output and update for atomic system:
 *    '<S38>/MATLAB Function4'
 *    '<S39>/MATLAB Function4'
 */
void LocalizationCor_MATLABFunction4(const real32_T rtu_u[36],
  B_MATLABFunction4_Localizatio_T *localB)
{
  /* MATLAB Function 'LocalizationCore/Core/Tracking Module/KALMAN Filter/MATLAB Function4': '<S41>:1' */
  /* '<S41>:1:3' */
  localB->y = rtu_u[0];
}

real_T rt_roundd(real_T u)
{
  real_T y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

/*
 * Output and update for atomic system:
 *    '<S38>/MATLAB Function5'
 *    '<S39>/MATLAB Function5'
 */
void LocalizationCor_MATLABFunction5(const real32_T rtu_u[3],
  B_MATLABFunction5_Localizatio_T *localB)
{
  real_T tmp;

  /* MATLAB Function 'LocalizationCore/Core/Tracking Module/KALMAN Filter/MATLAB Function5': '<S42>:1' */
  /* '<S42>:1:2' */
  /* '<S42>:1:3' */
  /* '<S42>:1:4' */
  /* '<S42>:1:5' */
  tmp = rt_roundd(rt_roundd((real_T)(rtu_u[0] * 13.5F)));
  if (tmp < 2.147483648E+9) {
    if (tmp >= -2.147483648E+9) {
      localB->y[0] = (int32_T)tmp;
    } else {
      localB->y[0] = MIN_int32_T;
    }
  } else {
    localB->y[0] = MAX_int32_T;
  }

  tmp = rt_roundd(rt_roundd((real_T)(rtu_u[1] * 13.5F)));
  if (tmp < 2.147483648E+9) {
    if (tmp >= -2.147483648E+9) {
      localB->y[1] = (int32_T)tmp;
    } else {
      localB->y[1] = MIN_int32_T;
    }
  } else {
    localB->y[1] = MAX_int32_T;
  }

  tmp = rt_roundd(rt_roundd((real_T)(rtu_u[2] * 8.0F / 37.0F + 1.0F)));
  if (tmp < 2.147483648E+9) {
    if (tmp >= -2.147483648E+9) {
      localB->y[2] = (int32_T)tmp;
    } else {
      localB->y[2] = MIN_int32_T;
    }
  } else {
    localB->y[2] = MAX_int32_T;
  }
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static void LocalizationCore_eml_sort_l(const real_T x[2048], real_T y[2048],
  int32_T idx[2048])
{
  int32_T k;
  int16_T idx_0[2048];
  int32_T i;
  int32_T j;
  int32_T pEnd;
  int32_T p;
  int32_T q;
  int32_T qEnd;
  int32_T kEnd;
  int32_T i_0;
  for (pEnd = 0; pEnd < 2048; pEnd++) {
    idx[pEnd] = pEnd + 1;
  }

  for (pEnd = 0; pEnd <= 2047; pEnd += 2) {
    if (!(x[pEnd] <= x[pEnd + 1])) {
      idx[pEnd] = pEnd + 2;
      idx[pEnd + 1] = pEnd + 1;
    }
  }

  for (i_0 = 0; i_0 < 2048; i_0++) {
    idx_0[i_0] = 1;
  }

  i_0 = 2;
  while (i_0 < 2048) {
    i = i_0 << 1;
    j = 1;
    pEnd = 1 + i_0;
    while (pEnd < 2049) {
      p = j;
      q = pEnd;
      qEnd = j + i;
      k = 0;
      kEnd = qEnd - j;
      while (k + 1 <= kEnd) {
        if (x[idx[p - 1] - 1] <= x[idx[q - 1] - 1]) {
          idx_0[k] = (int16_T)idx[p - 1];
          p++;
          if (p == pEnd) {
            while (q < qEnd) {
              k++;
              idx_0[k] = (int16_T)idx[q - 1];
              q++;
            }
          }
        } else {
          idx_0[k] = (int16_T)idx[q - 1];
          q++;
          if (q == qEnd) {
            while (p < pEnd) {
              k++;
              idx_0[k] = (int16_T)idx[p - 1];
              p++;
            }
          }
        }

        k++;
      }

      for (pEnd = 0; pEnd + 1 <= kEnd; pEnd++) {
        idx[(j + pEnd) - 1] = idx_0[pEnd];
      }

      j = qEnd;
      pEnd = qEnd + i_0;
    }

    i_0 = i;
  }

  for (pEnd = 0; pEnd < 2048; pEnd++) {
    y[pEnd] = x[idx[pEnd] - 1];
  }
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static void LocalizationCore_eml_sort_lvl(const real_T x[2048], real_T y[2048],
  int32_T idx[2048])
{
  int32_T k;
  int16_T idx_0[2048];
  int32_T i;
  int32_T j;
  int32_T pEnd;
  int32_T p;
  int32_T q;
  int32_T qEnd;
  int32_T kEnd;
  int32_T i_0;
  for (pEnd = 0; pEnd < 2048; pEnd++) {
    idx[pEnd] = pEnd + 1;
  }

  for (pEnd = 0; pEnd <= 2047; pEnd += 2) {
    if (!(x[pEnd] >= x[pEnd + 1])) {
      idx[pEnd] = pEnd + 2;
      idx[pEnd + 1] = pEnd + 1;
    }
  }

  for (i_0 = 0; i_0 < 2048; i_0++) {
    idx_0[i_0] = 1;
  }

  i_0 = 2;
  while (i_0 < 2048) {
    i = i_0 << 1;
    j = 1;
    pEnd = 1 + i_0;
    while (pEnd < 2049) {
      p = j;
      q = pEnd;
      qEnd = j + i;
      k = 0;
      kEnd = qEnd - j;
      while (k + 1 <= kEnd) {
        if (x[idx[p - 1] - 1] >= x[idx[q - 1] - 1]) {
          idx_0[k] = (int16_T)idx[p - 1];
          p++;
          if (p == pEnd) {
            while (q < qEnd) {
              k++;
              idx_0[k] = (int16_T)idx[q - 1];
              q++;
            }
          }
        } else {
          idx_0[k] = (int16_T)idx[q - 1];
          q++;
          if (q == qEnd) {
            while (p < pEnd) {
              k++;
              idx_0[k] = (int16_T)idx[p - 1];
              p++;
            }
          }
        }

        k++;
      }

      for (pEnd = 0; pEnd + 1 <= kEnd; pEnd++) {
        idx[(j + pEnd) - 1] = idx_0[pEnd];
      }

      j = qEnd;
      pEnd = qEnd + i_0;
    }

    i_0 = i;
  }

  for (pEnd = 0; pEnd < 2048; pEnd++) {
    y[pEnd] = x[idx[pEnd] - 1];
  }
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static void LocalizationCore_eml_sort_lv(const real_T x[2048], real_T y[2048],
  int32_T idx[2048])
{
  LocalizationCore_eml_sort_lvl(x, y, idx);
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static real32_T LocalizationCore_eml_xnrm2(int32_T n, const real32_T x[150],
  int32_T ix0)
{
  real32_T y;
  real32_T scale;
  int32_T kend;
  real32_T absxk;
  real32_T t;
  int32_T k;
  y = 0.0F;
  if (n == 1) {
    y = fabsf(x[ix0 - 1]);
  } else {
    scale = 1.17549435E-38F;
    kend = (ix0 + n) - 1;
    for (k = ix0; k <= kend; k++) {
      absxk = fabsf(x[k - 1]);
      if (absxk > scale) {
        t = scale / absxk;
        y = y * t * t + 1.0F;
        scale = absxk;
      } else {
        t = absxk / scale;
        y += t * t;
      }
    }

    y = scale * sqrtf(y);
  }

  return y;
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static real32_T LocalizationCore_eml_div(real_T x, real32_T y)
{
  return (real32_T)x / y;
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static void LocalizationCore_eml_xscal(int32_T n, real32_T a, real32_T x[150],
  int32_T ix0)
{
  int32_T b;
  int32_T k;
  b = (ix0 + n) - 1;
  for (k = ix0; k <= b; k++) {
    x[k - 1] *= a;
  }
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static real32_T LocalizationCore_eml_xnrm2_i(int32_T n, const real32_T x[50],
  int32_T ix0)
{
  real32_T y;
  real32_T scale;
  int32_T kend;
  real32_T absxk;
  real32_T t;
  int32_T k;
  y = 0.0F;
  scale = 1.17549435E-38F;
  kend = (ix0 + n) - 1;
  for (k = ix0; k <= kend; k++) {
    absxk = fabsf(x[k - 1]);
    if (absxk > scale) {
      t = scale / absxk;
      y = y * t * t + 1.0F;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * sqrtf(y);
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static void LocalizationCore_eml_xscal_m(int32_T n, real32_T a, real32_T x[50],
  int32_T ix0)
{
  int32_T b;
  int32_T k;
  b = (ix0 + n) - 1;
  for (k = ix0; k <= b; k++) {
    x[k - 1] *= a;
  }
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static real32_T LocalizationCore_eml_div_n(real32_T x, real32_T y)
{
  return x / y;
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static void LocalizationCore_eml_xscal_mg(real32_T a, real32_T x[9], int32_T ix0)
{
  int32_T k;
  for (k = ix0; k <= ix0 + 2; k++) {
    x[k - 1] *= a;
  }
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static void LocalizationCore_eml_xswap(real32_T x[9], int32_T ix0, int32_T iy0)
{
  int32_T ix;
  int32_T iy;
  real32_T temp;
  ix = ix0 - 1;
  iy = iy0 - 1;
  temp = x[ix];
  x[ix] = x[iy];
  x[iy] = temp;
  ix++;
  iy++;
  temp = x[ix];
  x[ix] = x[iy];
  x[iy] = temp;
  ix++;
  iy++;
  temp = x[ix];
  x[ix] = x[iy];
  x[iy] = temp;
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static void LocalizationCore_eml_xrotg(real32_T *a, real32_T *b, real32_T *c,
  real32_T *s)
{
  real32_T roe;
  real32_T absa;
  real32_T absb;
  real32_T scale;
  real32_T ads;
  real32_T bds;
  real32_T r;
  roe = *b;
  absa = fabsf(*a);
  absb = fabsf(*b);
  if (absa > absb) {
    roe = *a;
  }

  scale = absa + absb;
  if (scale == 0.0F) {
    bds = 0.0F;
    ads = 1.0F;
    r = 0.0F;
    scale = 0.0F;
  } else {
    ads = absa / scale;
    bds = absb / scale;
    r = sqrtf(ads * ads + bds * bds) * scale;
    if (roe < 0.0F) {
      r = -r;
    }

    ads = *a / r;
    bds = *b / r;
    if (absa > absb) {
      scale = bds;
    } else if (ads != 0.0F) {
      scale = 1.0F / ads;
    } else {
      scale = 1.0F;
    }
  }

  *a = r;
  *b = scale;
  *c = ads;
  *s = bds;
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static void LocalizationCore_eml_xrot(real32_T x[9], int32_T ix0, int32_T iy0,
  real32_T c, real32_T s)
{
  int32_T ix;
  int32_T iy;
  real32_T y;
  real32_T b_y;
  ix = ix0 - 1;
  iy = iy0 - 1;
  y = c * x[ix];
  b_y = s * x[iy];
  x[iy] = c * x[iy] - s * x[ix];
  x[ix] = y + b_y;
  iy++;
  ix++;
  y = c * x[ix];
  b_y = s * x[iy];
  x[iy] = c * x[iy] - s * x[ix];
  x[ix] = y + b_y;
  iy++;
  ix++;
  y = c * x[ix];
  b_y = s * x[iy];
  x[iy] = c * x[iy] - s * x[ix];
  x[ix] = y + b_y;
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static real32_T LocalizationCore_eml_xdotc_a(int32_T n, const real32_T x[9],
  int32_T ix0, const real32_T y[9], int32_T iy0)
{
  real32_T d;
  int32_T ix;
  int32_T iy;
  int32_T k;
  d = 0.0F;
  if (!(n < 1)) {
    ix = ix0;
    iy = iy0;
    for (k = 1; k <= n; k++) {
      d += x[ix - 1] * y[iy - 1];
      ix++;
      iy++;
    }
  }

  return d;
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static void LocalizationCore_eml_xaxpy_eck(int32_T n, real32_T a, int32_T ix0,
  real32_T y[9], int32_T iy0)
{
  int32_T ix;
  int32_T iy;
  int32_T k;
  if (!((n < 1) || (a == 0.0F))) {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * y[ix];
      ix++;
      iy++;
    }
  }
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static void LocalizationCore_eml_xaxpy_ec(int32_T n, real32_T a, const real32_T
  x[3], int32_T ix0, real32_T y[150], int32_T iy0)
{
  int32_T ix;
  int32_T iy;
  int32_T k;
  if (!((n < 1) || (a == 0.0F))) {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * x[ix];
      ix++;
      iy++;
    }
  }
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static void LocalizationCore_eml_xaxpy_e(int32_T n, real32_T a, const real32_T
  x[150], int32_T ix0, real32_T y[3], int32_T iy0)
{
  int32_T ix;
  int32_T iy;
  int32_T k;
  if (!((n < 1) || (a == 0.0F))) {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * x[ix];
      ix++;
      iy++;
    }
  }
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static real32_T LocalizationCore_eml_xdotc(int32_T n, const real32_T x[150],
  int32_T ix0, const real32_T y[150], int32_T iy0)
{
  real32_T d;
  int32_T ix;
  int32_T iy;
  int32_T k;
  d = 0.0F;
  ix = ix0;
  iy = iy0;
  for (k = 1; k <= n; k++) {
    d += x[ix - 1] * y[iy - 1];
    ix++;
    iy++;
  }

  return d;
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static void LocalizationCore_eml_xaxpy(int32_T n, real32_T a, int32_T ix0,
  real32_T y[150], int32_T iy0)
{
  int32_T ix;
  int32_T iy;
  int32_T k;
  if (!(a == 0.0F)) {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * y[ix];
      ix++;
      iy++;
    }
  }
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static void LocalizationCore_eml_xgesvd(const real32_T A[150], real32_T U[9],
  real32_T S[3])
{
  real32_T b_A[150];
  real32_T s[4];
  real32_T e[50];
  real32_T work[3];
  int32_T q;
  int32_T qjj;
  int32_T m;
  int32_T iter;
  real32_T snorm;
  real32_T ztest;
  int32_T kase;
  int32_T qs;
  real32_T ztest_0;
  real32_T f;
  real32_T sm;
  real32_T sqds;
  real32_T b;
  boolean_T exitg;
  real32_T varargin_1_idx;
  memcpy(&b_A[0], &A[0], 150U * sizeof(real32_T));
  memset(&e[0], (int32_T)0.0F, 50U * sizeof(real32_T));
  work[0] = 0.0F;
  work[1] = 0.0F;
  work[2] = 0.0F;
  for (kase = 0; kase < 9; kase++) {
    U[kase] = 0.0F;
  }

  ztest = LocalizationCore_eml_xnrm2(3, b_A, 1);
  if (ztest > 0.0F) {
    if (b_A[0] < 0.0F) {
      s[0] = -ztest;
    } else {
      s[0] = ztest;
    }

    LocalizationCore_eml_xscal(3, LocalizationCore_eml_div(1.0, s[0]), b_A, 1);
    b_A[0]++;
    s[0] = -s[0];
  } else {
    s[0] = 0.0F;
  }

  for (kase = 1; kase + 1 < 51; kase++) {
    qjj = 3 * kase;
    if (s[0] != 0.0F) {
      LocalizationCore_eml_xaxpy(3, -LocalizationCore_eml_div_n
        (LocalizationCore_eml_xdotc(3, b_A, 1, b_A, qjj + 1), b_A[0]), 1, b_A,
        qjj + 1);
    }

    e[kase] = b_A[qjj];
  }

  for (kase = 0; kase + 1 < 4; kase++) {
    U[kase] = b_A[kase];
  }

  ztest = LocalizationCore_eml_xnrm2_i(49, e, 2);
  if (ztest == 0.0F) {
    e[0] = 0.0F;
  } else {
    if (e[1] < 0.0F) {
      e[0] = -ztest;
    } else {
      e[0] = ztest;
    }

    LocalizationCore_eml_xscal_m(49, LocalizationCore_eml_div(1.0, e[0]), e, 2);
    e[1]++;
  }

  e[0] = -e[0];
  if (e[0] != 0.0F) {
    for (kase = 1; kase + 1 < 4; kase++) {
      work[kase] = 0.0F;
    }

    for (kase = 1; kase + 1 < 51; kase++) {
      LocalizationCore_eml_xaxpy_e(2, e[kase], b_A, 3 * kase + 2, work, 2);
    }

    for (kase = 1; kase + 1 < 51; kase++) {
      LocalizationCore_eml_xaxpy_ec(2, LocalizationCore_eml_div_n(-e[kase], e[1]),
        work, 2, b_A, 3 * kase + 2);
    }
  }

  ztest = LocalizationCore_eml_xnrm2(2, b_A, 5);
  if (ztest > 0.0F) {
    if (b_A[4] < 0.0F) {
      s[1] = -ztest;
    } else {
      s[1] = ztest;
    }

    LocalizationCore_eml_xscal(2, LocalizationCore_eml_div(1.0, s[1]), b_A, 5);
    b_A[4]++;
    s[1] = -s[1];
  } else {
    s[1] = 0.0F;
  }

  for (kase = 2; kase + 1 < 51; kase++) {
    qjj = 3 * kase + 1;
    if (s[1] != 0.0F) {
      LocalizationCore_eml_xaxpy(2, -LocalizationCore_eml_div_n
        (LocalizationCore_eml_xdotc(2, b_A, 5, b_A, qjj + 1), b_A[4]), 5, b_A,
        qjj + 1);
    }

    e[kase] = b_A[qjj];
  }

  for (kase = 1; kase + 1 < 4; kase++) {
    U[kase + 3] = b_A[kase + 3];
  }

  ztest = LocalizationCore_eml_xnrm2_i(48, e, 3);
  if (ztest == 0.0F) {
    e[1] = 0.0F;
  } else {
    if (e[2] < 0.0F) {
      e[1] = -ztest;
    } else {
      e[1] = ztest;
    }

    LocalizationCore_eml_xscal_m(48, LocalizationCore_eml_div(1.0, e[1]), e, 3);
    e[2]++;
  }

  e[1] = -e[1];
  if (e[1] != 0.0F) {
    for (kase = 2; kase + 1 < 4; kase++) {
      work[kase] = 0.0F;
    }

    for (kase = 2; kase + 1 < 51; kase++) {
      LocalizationCore_eml_xaxpy_e(1, e[kase], b_A, 3 * kase + 3, work, 3);
    }

    for (kase = 2; kase + 1 < 51; kase++) {
      LocalizationCore_eml_xaxpy_ec(1, LocalizationCore_eml_div_n(-e[kase], e[2]),
        work, 3, b_A, 3 * kase + 3);
    }
  }

  for (kase = 3; kase + 1 < 51; kase++) {
    e[kase] = b_A[3 * kase + 2];
  }

  ztest = LocalizationCore_eml_xnrm2_i(47, e, 4);
  if (ztest == 0.0F) {
    e[2] = 0.0F;
  } else {
    if (e[3] < 0.0F) {
      e[2] = -ztest;
    } else {
      e[2] = ztest;
    }

    LocalizationCore_eml_xscal_m(47, LocalizationCore_eml_div(1.0, e[2]), e, 4);
  }

  e[2] = -e[2];
  m = 2;
  s[2] = b_A[8];
  s[3] = 0.0F;
  U[6] = 0.0F;
  U[7] = 0.0F;
  U[8] = 1.0F;
  for (qs = 1; qs >= 0; qs += -1) {
    iter = 3 * qs + qs;
    if (s[qs] != 0.0F) {
      for (kase = qs + 1; kase + 1 < 4; kase++) {
        qjj = (3 * kase + qs) + 1;
        LocalizationCore_eml_xaxpy_eck(3 - qs, -LocalizationCore_eml_div_n
          (LocalizationCore_eml_xdotc_a(3 - qs, U, iter + 1, U, qjj), U[iter]),
          iter + 1, U, qjj);
      }

      for (kase = qs; kase + 1 < 4; kase++) {
        U[kase + 3 * qs] = -U[3 * qs + kase];
      }

      U[iter]++;
      kase = 1;
      while (kase <= qs) {
        U[3] = 0.0F;
        kase = 2;
      }
    } else {
      U[3 * qs] = 0.0F;
      U[1 + 3 * qs] = 0.0F;
      U[2 + 3 * qs] = 0.0F;
      U[iter] = 1.0F;
    }
  }

  f = e[0];
  if (s[0] != 0.0F) {
    ztest_0 = fabsf(s[0]);
    ztest = LocalizationCore_eml_div_n(s[0], ztest_0);
    s[0] = ztest_0;
    f = LocalizationCore_eml_div_n(e[0], ztest);
    LocalizationCore_eml_xscal_mg(ztest, U, 1);
  }

  if (f != 0.0F) {
    ztest_0 = fabsf(f);
    ztest = LocalizationCore_eml_div_n(ztest_0, f);
    f = ztest_0;
    s[1] *= ztest;
  }

  e[0] = f;
  f = e[1];
  if (s[1] != 0.0F) {
    ztest_0 = fabsf(s[1]);
    ztest = LocalizationCore_eml_div_n(s[1], ztest_0);
    s[1] = ztest_0;
    f = LocalizationCore_eml_div_n(e[1], ztest);
    LocalizationCore_eml_xscal_mg(ztest, U, 4);
  }

  if (f != 0.0F) {
    ztest_0 = fabsf(f);
    ztest = LocalizationCore_eml_div_n(ztest_0, f);
    f = ztest_0;
    s[2] = b_A[8] * ztest;
  }

  e[1] = f;
  f = e[2];
  if (s[2] != 0.0F) {
    ztest_0 = fabsf(s[2]);
    ztest = LocalizationCore_eml_div_n(s[2], ztest_0);
    s[2] = ztest_0;
    f = LocalizationCore_eml_div_n(e[2], ztest);
    LocalizationCore_eml_xscal_mg(ztest, U, 7);
  }

  if (f != 0.0F) {
    f = fabsf(f);
    s[3] = 0.0F;
  }

  e[2] = f;
  e[3] = 0.0F;
  iter = 0;
  snorm = fmaxf(fmaxf(fmaxf(fmaxf(0.0F, fmaxf(s[0], fabsf(e[0]))), fmaxf(s[1],
    fabsf(e[1]))), fmaxf(s[2], fabsf(f))), fmaxf(s[3], 0.0F));
  while ((m + 2 > 0) && (!(iter >= 75))) {
    qjj = m + 1;
    do {
      kase = 0;
      q = qjj;
      if (qjj == 0) {
        kase = 1;
      } else {
        ztest = fabsf(e[qjj - 1]);
        if ((ztest <= (fabsf(s[qjj - 1]) + fabsf(s[qjj])) * 1.1920929E-7F) ||
            (ztest <= 9.86076132E-32F) || ((iter > 20) && (ztest <=
              1.1920929E-7F * snorm))) {
          e[qjj - 1] = 0.0F;
          kase = 1;
        } else {
          qjj--;
        }
      }
    } while (kase == 0);

    if (m + 1 == qjj) {
      kase = 4;
    } else {
      qs = m + 2;
      kase = m + 2;
      exitg = FALSE;
      while ((!exitg) && (kase >= qjj)) {
        qs = kase;
        if (kase == qjj) {
          exitg = TRUE;
        } else {
          ztest = 0.0F;
          if (kase < m + 2) {
            ztest = fabsf(e[kase - 1]);
          }

          if (kase > qjj + 1) {
            ztest += fabsf(e[kase - 2]);
          }

          ztest_0 = fabsf(s[kase - 1]);
          if ((ztest_0 <= 1.1920929E-7F * ztest) || (ztest_0 <= 9.86076132E-32F))
          {
            s[kase - 1] = 0.0F;
            exitg = TRUE;
          } else {
            kase--;
          }
        }
      }

      if (qs == qjj) {
        kase = 3;
      } else if (m + 2 == qs) {
        kase = 1;
      } else {
        kase = 2;
        q = qs;
      }
    }

    switch (kase) {
     case 1:
      f = e[m];
      e[m] = 0.0F;
      for (kase = m; kase + 1 >= q + 1; kase--) {
        ztest = s[kase];
        LocalizationCore_eml_xrotg(&ztest, &f, &ztest_0, &b);
        s[kase] = ztest;
        if (kase + 1 > q + 1) {
          f = e[kase - 1] * -b;
          e[kase - 1] *= ztest_0;
        }
      }
      break;

     case 2:
      f = e[q - 1];
      e[q - 1] = 0.0F;
      for (kase = q; kase + 1 <= m + 2; kase++) {
        ztest = s[kase];
        LocalizationCore_eml_xrotg(&ztest, &f, &ztest_0, &b);
        s[kase] = ztest;
        f = -b * e[kase];
        e[kase] *= ztest_0;
        LocalizationCore_eml_xrot(U, 3 * kase + 1, 3 * (q - 1) + 1, ztest_0, b);
      }
      break;

     case 3:
      varargin_1_idx = fabsf(s[m + 1]);
      f = fabsf(s[m]);
      b = fabsf(e[m]);
      sm = fabsf(s[q]);
      sqds = fabsf(e[q]);
      if (f > varargin_1_idx) {
        varargin_1_idx = f;
      }

      if (b > varargin_1_idx) {
        varargin_1_idx = b;
      }

      if (sm > varargin_1_idx) {
        varargin_1_idx = sm;
      }

      if (sqds > varargin_1_idx) {
        varargin_1_idx = sqds;
      }

      sm = LocalizationCore_eml_div_n(s[m + 1], varargin_1_idx);
      ztest = LocalizationCore_eml_div_n(s[m], varargin_1_idx);
      ztest_0 = LocalizationCore_eml_div_n(e[m], varargin_1_idx);
      sqds = LocalizationCore_eml_div_n(s[q], varargin_1_idx);
      b = ((ztest + sm) * (ztest - sm) + ztest_0 * ztest_0) / 2.0F;
      ztest = sm * ztest_0;
      ztest *= ztest;
      ztest_0 = 0.0F;
      if ((b != 0.0F) || (ztest != 0.0F)) {
        ztest_0 = sqrtf(b * b + ztest);
        if (b < 0.0F) {
          ztest_0 = -ztest_0;
        }

        ztest_0 = LocalizationCore_eml_div_n(ztest, b + ztest_0);
      }

      f = (sqds + sm) * (sqds - sm) + ztest_0;
      ztest = sqds * LocalizationCore_eml_div_n(e[q], varargin_1_idx);
      for (kase = q + 1; kase <= m + 1; kase++) {
        LocalizationCore_eml_xrotg(&f, &ztest, &sm, &varargin_1_idx);
        if (kase > q + 1) {
          e[kase - 2] = f;
        }

        ztest = s[kase - 1] * sm;
        ztest_0 = e[kase - 1] * varargin_1_idx;
        e[kase - 1] = e[kase - 1] * sm - s[kase - 1] * varargin_1_idx;
        b = s[kase];
        s[kase] *= sm;
        ztest_0 += ztest;
        ztest = varargin_1_idx * b;
        LocalizationCore_eml_xrotg(&ztest_0, &ztest, &b, &sm);
        s[kase - 1] = ztest_0;
        f = e[kase - 1] * b + sm * s[kase];
        s[kase] = e[kase - 1] * -sm + b * s[kase];
        ztest = sm * e[kase];
        e[kase] *= b;
        if (kase < 3) {
          LocalizationCore_eml_xrot(U, 3 * (kase - 1) + 1, 3 * kase + 1, b, sm);
        }
      }

      e[m] = f;
      iter++;
      break;

     default:
      if (s[q] < 0.0F) {
        s[q] = -s[q];
      }

      kase = q + 1;
      while ((q + 1 < 4) && (s[q] < s[kase])) {
        ztest_0 = s[q];
        s[q] = s[kase];
        s[kase] = ztest_0;
        if (q + 1 < 3) {
          LocalizationCore_eml_xswap(U, 3 * q + 1, 3 * (q + 1) + 1);
        }

        q = kase;
        kase++;
      }

      iter = 0;
      m--;
      break;
    }
  }

  S[0] = s[0];
  S[1] = s[1];
  S[2] = s[2];
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static void LocalizationCore_orth(const real32_T A[150], real32_T Q_data[9],
  int32_T Q_sizes[2])
{
  int32_T r;
  real32_T tol;
  real32_T U[9];
  real32_T s[3];
  int32_T i;
  LocalizationCore_eml_xgesvd(A, U, s);
  tol = 50.0F * s[0] * 1.1920929E-7F;
  r = ((s[0] > tol) + (s[1] > tol)) + (s[2] > tol);
  if (1 > r) {
    r = 0;
  }

  Q_sizes[0] = 3;
  Q_sizes[1] = r;
  for (i = 0; i < r; i++) {
    Q_data[3 * i] = U[3 * i];
    Q_data[1 + 3 * i] = U[3 * i + 1];
    Q_data[2 + 3 * i] = U[3 * i + 2];
  }
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static real32_T LocalizationCore_eml_xnrm2_in(int32_T n, const real32_T x[150],
  int32_T ix0)
{
  real32_T y;
  real32_T scale;
  int32_T kend;
  real32_T absxk;
  real32_T t;
  int32_T k;
  y = 0.0F;
  scale = 1.17549435E-38F;
  kend = (ix0 + n) - 1;
  for (k = ix0; k <= kend; k++) {
    absxk = fabsf(x[k - 1]);
    if (absxk > scale) {
      t = scale / absxk;
      y = y * t * t + 1.0F;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * sqrtf(y);
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static void LocalizationCore_eml_xscal_mg3(int32_T n, real32_T a, real32_T x[150],
  int32_T ix0)
{
  int32_T b;
  int32_T k;
  b = (ix0 + n) - 1;
  for (k = ix0; k <= b; k++) {
    x[k - 1] *= a;
  }
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static real32_T LocalizationCore_eml_xnrm2_ina(const real32_T x[3], int32_T ix0)
{
  real32_T y;
  real32_T scale;
  real32_T absxk;
  real32_T t;
  int32_T k;
  y = 0.0F;
  scale = 1.17549435E-38F;
  for (k = ix0; k <= ix0 + 1; k++) {
    absxk = fabsf(x[k - 1]);
    if (absxk > scale) {
      t = scale / absxk;
      y = y * t * t + 1.0F;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * sqrtf(y);
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static void LocalizationCore_eml_xscal_mg3o(real32_T a, real32_T x[3], int32_T
  ix0)
{
  int32_T k;
  for (k = ix0; k <= ix0 + 1; k++) {
    x[k - 1] *= a;
  }
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static void LocalizationCor_eml_xscal_mg3or(real32_T a, real32_T x[150], int32_T
  ix0)
{
  int32_T k;
  for (k = ix0; k <= ix0 + 49; k++) {
    x[k - 1] *= a;
  }
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static void LocalizationCore_eml_xswap_b(real32_T x[150], int32_T ix0, int32_T
  iy0)
{
  int32_T ix;
  int32_T iy;
  real32_T temp;
  int32_T k;
  ix = ix0 - 1;
  iy = iy0 - 1;
  for (k = 0; k < 50; k++) {
    temp = x[ix];
    x[ix] = x[iy];
    x[iy] = temp;
    ix++;
    iy++;
  }
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static void LocalizationCore_eml_xrot_d(real32_T x[150], int32_T ix0, int32_T
  iy0, real32_T c, real32_T s)
{
  int32_T ix;
  int32_T iy;
  int32_T k;
  real32_T y;
  real32_T b_y;
  ix = ix0 - 1;
  iy = iy0 - 1;
  for (k = 0; k < 50; k++) {
    y = c * x[ix];
    b_y = s * x[iy];
    x[iy] = c * x[iy] - s * x[ix];
    x[ix] = y + b_y;
    iy++;
    ix++;
  }
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static real32_T LocalizationCore_eml_xdotc_a5(int32_T n, const real32_T x[150],
  int32_T ix0, const real32_T y[150], int32_T iy0)
{
  real32_T d;
  int32_T ix;
  int32_T iy;
  int32_T k;
  d = 0.0F;
  ix = ix0;
  iy = iy0;
  for (k = 1; k <= n; k++) {
    d += x[ix - 1] * y[iy - 1];
    ix++;
    iy++;
  }

  return d;
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static void LocalizationCore_eml_xaxpy_eckn(int32_T n, real32_T a, int32_T ix0,
  real32_T y[150], int32_T iy0)
{
  int32_T ix;
  int32_T iy;
  int32_T k;
  if (!(a == 0.0F)) {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * y[ix];
      ix++;
      iy++;
    }
  }
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static void LocalizationCo_eml_xaxpy_eckne4(int32_T n, real32_T a, const
  real32_T x[50], int32_T ix0, real32_T y[150], int32_T iy0)
{
  int32_T ix;
  int32_T iy;
  int32_T k;
  if (!(a == 0.0F)) {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * x[ix];
      ix++;
      iy++;
    }
  }
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static void LocalizationCor_eml_xaxpy_eckne(int32_T n, real32_T a, const
  real32_T x[150], int32_T ix0, real32_T y[50], int32_T iy0)
{
  int32_T ix;
  int32_T iy;
  int32_T k;
  if (!(a == 0.0F)) {
    ix = ix0 - 1;
    iy = iy0 - 1;
    for (k = 0; k < n; k++) {
      y[iy] += a * x[ix];
      ix++;
      iy++;
    }
  }
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static void LocalizationCore_eml_xgesvd_l(const real32_T A[150], real32_T U[150],
  real32_T S[3], real32_T V[9])
{
  real32_T b_A[150];
  real32_T s[3];
  real32_T e[3];
  real32_T work[50];
  real32_T Vf[9];
  int32_T q;
  int32_T qp;
  int32_T qjj;
  int32_T m;
  int32_T iter;
  real32_T snorm;
  real32_T ztest;
  int32_T kase;
  int32_T qs;
  real32_T ztest_0;
  real32_T f;
  real32_T sm;
  real32_T sqds;
  real32_T b;
  boolean_T exitg;
  real32_T varargin_1_idx;
  memcpy(&b_A[0], &A[0], 150U * sizeof(real32_T));
  e[0] = 0.0F;
  e[1] = 0.0F;
  e[2] = 0.0F;
  memset(&work[0], (int32_T)0.0F, 50U * sizeof(real32_T));
  memset(&U[0], (int32_T)0.0F, 150U * sizeof(real32_T));
  for (kase = 0; kase < 9; kase++) {
    Vf[kase] = 0.0F;
  }

  ztest = LocalizationCore_eml_xnrm2_in(50, b_A, 1);
  if (ztest > 0.0F) {
    if (b_A[0] < 0.0F) {
      s[0] = -ztest;
    } else {
      s[0] = ztest;
    }

    LocalizationCore_eml_xscal_mg3(50, LocalizationCore_eml_div(1.0, s[0]), b_A,
      1);
    b_A[0]++;
    s[0] = -s[0];
  } else {
    s[0] = 0.0F;
  }

  for (kase = 1; kase + 1 < 4; kase++) {
    qjj = 50 * kase;
    if (s[0] != 0.0F) {
      LocalizationCore_eml_xaxpy_eckn(50, -LocalizationCore_eml_div_n
        (LocalizationCore_eml_xdotc_a5(50, b_A, 1, b_A, qjj + 1), b_A[0]), 1,
        b_A, qjj + 1);
    }

    e[kase] = b_A[qjj];
  }

  for (qp = 0; qp + 1 < 51; qp++) {
    U[qp] = b_A[qp];
  }

  ztest = LocalizationCore_eml_xnrm2_ina(e, 2);
  if (ztest == 0.0F) {
    e[0] = 0.0F;
  } else {
    if (e[1] < 0.0F) {
      ztest = -ztest;
    }

    e[0] = ztest;
    LocalizationCore_eml_xscal_mg3o(LocalizationCore_eml_div(1.0, ztest), e, 2);
    e[1]++;
  }

  e[0] = -e[0];
  if (e[0] != 0.0F) {
    for (qp = 2; qp < 51; qp++) {
      work[qp - 1] = 0.0F;
    }

    for (qp = 1; qp + 1 < 4; qp++) {
      LocalizationCor_eml_xaxpy_eckne(49, e[qp], b_A, 2 + 50 * qp, work, 2);
    }

    for (qp = 1; qp + 1 < 4; qp++) {
      LocalizationCo_eml_xaxpy_eckne4(49, LocalizationCore_eml_div_n(-e[qp], e[1]),
        work, 2, b_A, 2 + 50 * qp);
    }
  }

  for (qp = 1; qp + 1 < 4; qp++) {
    Vf[qp] = e[qp];
  }

  ztest = LocalizationCore_eml_xnrm2_in(49, b_A, 52);
  if (ztest > 0.0F) {
    if (b_A[51] < 0.0F) {
      s[1] = -ztest;
    } else {
      s[1] = ztest;
    }

    LocalizationCore_eml_xscal_mg3(49, LocalizationCore_eml_div(1.0, s[1]), b_A,
      52);
    b_A[51]++;
    s[1] = -s[1];
  } else {
    s[1] = 0.0F;
  }

  for (kase = 2; kase + 1 < 4; kase++) {
    qjj = 50 * kase + 1;
    if (s[1] != 0.0F) {
      LocalizationCore_eml_xaxpy_eckn(49, -LocalizationCore_eml_div_n
        (LocalizationCore_eml_xdotc_a5(49, b_A, 52, b_A, qjj + 1), b_A[51]), 52,
        b_A, qjj + 1);
    }

    e[kase] = b_A[qjj];
  }

  for (qp = 1; qp + 1 < 51; qp++) {
    U[qp + 50] = b_A[qp + 50];
  }

  ztest = LocalizationCore_eml_xnrm2_in(48, b_A, 103);
  if (ztest > 0.0F) {
    if (b_A[102] < 0.0F) {
      s[2] = -ztest;
    } else {
      s[2] = ztest;
    }

    LocalizationCore_eml_xscal_mg3(48, LocalizationCore_eml_div(1.0, s[2]), b_A,
      103);
    b_A[102]++;
    s[2] = -s[2];
  } else {
    s[2] = 0.0F;
  }

  for (qp = 2; qp + 1 < 51; qp++) {
    U[qp + 100] = b_A[qp + 100];
  }

  m = 1;
  for (qs = 2; qs >= 0; qs += -1) {
    iter = 50 * qs + qs;
    if (s[qs] != 0.0F) {
      for (qp = qs + 1; qp + 1 < 4; qp++) {
        qjj = (50 * qp + qs) + 1;
        LocalizationCore_eml_xaxpy_eckn(50 - qs, -LocalizationCore_eml_div_n
          (LocalizationCore_eml_xdotc_a5(50 - qs, U, iter + 1, U, qjj), U[iter]),
          iter + 1, U, qjj);
      }

      for (kase = qs; kase + 1 < 51; kase++) {
        U[kase + 50 * qs] = -U[50 * qs + kase];
      }

      U[iter]++;
      for (kase = 1; kase <= qs; kase++) {
        U[(kase + 50 * qs) - 1] = 0.0F;
      }
    } else {
      memset(&U[50 * qs], (int32_T)0.0F, 50U * sizeof(real32_T));
      U[iter] = 1.0F;
    }
  }

  for (qjj = 2; qjj >= 0; qjj += -1) {
    if ((qjj + 1 <= 1) && (e[0] != 0.0F)) {
      LocalizationCore_eml_xaxpy_eck(2, -LocalizationCore_eml_div_n
        (LocalizationCore_eml_xdotc_a(2, Vf, 2, Vf, 5), Vf[1]), 2, Vf, 5);
      LocalizationCore_eml_xaxpy_eck(2, -LocalizationCore_eml_div_n
        (LocalizationCore_eml_xdotc_a(2, Vf, 2, Vf, 8), Vf[1]), 2, Vf, 8);
    }

    Vf[3 * qjj] = 0.0F;
    Vf[1 + 3 * qjj] = 0.0F;
    Vf[2 + 3 * qjj] = 0.0F;
    Vf[qjj + 3 * qjj] = 1.0F;
  }

  f = e[0];
  if (s[0] != 0.0F) {
    ztest_0 = fabsf(s[0]);
    ztest = LocalizationCore_eml_div_n(s[0], ztest_0);
    s[0] = ztest_0;
    f = LocalizationCore_eml_div_n(e[0], ztest);
    LocalizationCor_eml_xscal_mg3or(ztest, U, 1);
  }

  if (f != 0.0F) {
    ztest_0 = fabsf(f);
    ztest = LocalizationCore_eml_div_n(ztest_0, f);
    f = ztest_0;
    s[1] *= ztest;
    LocalizationCore_eml_xscal_mg(ztest, Vf, 4);
  }

  e[0] = f;
  f = b_A[101];
  if (s[1] != 0.0F) {
    ztest_0 = fabsf(s[1]);
    ztest = LocalizationCore_eml_div_n(s[1], ztest_0);
    s[1] = ztest_0;
    f = LocalizationCore_eml_div_n(b_A[101], ztest);
    LocalizationCor_eml_xscal_mg3or(ztest, U, 51);
  }

  if (f != 0.0F) {
    ztest_0 = fabsf(f);
    ztest = LocalizationCore_eml_div_n(ztest_0, f);
    f = ztest_0;
    s[2] *= ztest;
    LocalizationCore_eml_xscal_mg(ztest, Vf, 7);
  }

  e[1] = f;
  if (s[2] != 0.0F) {
    ztest_0 = fabsf(s[2]);
    ztest = LocalizationCore_eml_div_n(s[2], ztest_0);
    s[2] = ztest_0;
    LocalizationCor_eml_xscal_mg3or(ztest, U, 101);
  }

  e[2] = 0.0F;
  iter = 0;
  snorm = fmaxf(fmaxf(fmaxf(0.0F, fmaxf(s[0], e[0])), fmaxf(s[1], f)), s[2]);
  while ((m + 2 > 0) && (!(iter >= 75))) {
    qjj = m + 1;
    do {
      kase = 0;
      q = qjj;
      if (qjj == 0) {
        kase = 1;
      } else {
        ztest = fabsf(e[qjj - 1]);
        if ((ztest <= (fabsf(s[qjj - 1]) + fabsf(s[qjj])) * 1.1920929E-7F) ||
            (ztest <= 9.86076132E-32F) || ((iter > 20) && (ztest <=
              1.1920929E-7F * snorm))) {
          e[qjj - 1] = 0.0F;
          kase = 1;
        } else {
          qjj--;
        }
      }
    } while (kase == 0);

    if (m + 1 == qjj) {
      kase = 4;
    } else {
      qs = m + 2;
      qp = m + 2;
      exitg = FALSE;
      while ((!exitg) && (qp >= qjj)) {
        qs = qp;
        if (qp == qjj) {
          exitg = TRUE;
        } else {
          ztest = 0.0F;
          if (qp < m + 2) {
            ztest = fabsf(e[qp - 1]);
          }

          if (qp > qjj + 1) {
            ztest += fabsf(e[qp - 2]);
          }

          ztest_0 = fabsf(s[qp - 1]);
          if ((ztest_0 <= 1.1920929E-7F * ztest) || (ztest_0 <= 9.86076132E-32F))
          {
            s[qp - 1] = 0.0F;
            exitg = TRUE;
          } else {
            qp--;
          }
        }
      }

      if (qs == qjj) {
        kase = 3;
      } else {
        if (m + 2 != qs) {
          kase = 2;
          q = qs;
        }
      }
    }

    switch (kase) {
     case 1:
      f = e[m];
      e[m] = 0.0F;
      for (kase = m; kase + 1 >= q + 1; kase--) {
        ztest = s[kase];
        LocalizationCore_eml_xrotg(&ztest, &f, &ztest_0, &b);
        s[kase] = ztest;
        if (kase + 1 > q + 1) {
          f = -b * e[0];
          e[0] *= ztest_0;
        }

        LocalizationCore_eml_xrot(Vf, 3 * kase + 1, 3 * (m + 1) + 1, ztest_0, b);
      }
      break;

     case 2:
      f = e[q - 1];
      e[q - 1] = 0.0F;
      for (kase = q; kase + 1 <= m + 2; kase++) {
        ztest = s[kase];
        LocalizationCore_eml_xrotg(&ztest, &f, &ztest_0, &b);
        s[kase] = ztest;
        f = -b * e[kase];
        e[kase] *= ztest_0;
        LocalizationCore_eml_xrot_d(U, 50 * kase + 1, 50 * (q - 1) + 1, ztest_0,
          b);
      }
      break;

     case 3:
      varargin_1_idx = fabsf(s[m + 1]);
      f = fabsf(s[m]);
      b = fabsf(e[m]);
      sm = fabsf(s[q]);
      sqds = fabsf(e[q]);
      if (f > varargin_1_idx) {
        varargin_1_idx = f;
      }

      if (b > varargin_1_idx) {
        varargin_1_idx = b;
      }

      if (sm > varargin_1_idx) {
        varargin_1_idx = sm;
      }

      if (sqds > varargin_1_idx) {
        varargin_1_idx = sqds;
      }

      sm = LocalizationCore_eml_div_n(s[m + 1], varargin_1_idx);
      ztest = LocalizationCore_eml_div_n(s[m], varargin_1_idx);
      ztest_0 = LocalizationCore_eml_div_n(e[m], varargin_1_idx);
      sqds = LocalizationCore_eml_div_n(s[q], varargin_1_idx);
      b = ((ztest + sm) * (ztest - sm) + ztest_0 * ztest_0) / 2.0F;
      ztest = sm * ztest_0;
      ztest *= ztest;
      ztest_0 = 0.0F;
      if ((b != 0.0F) || (ztest != 0.0F)) {
        ztest_0 = sqrtf(b * b + ztest);
        if (b < 0.0F) {
          ztest_0 = -ztest_0;
        }

        ztest_0 = LocalizationCore_eml_div_n(ztest, b + ztest_0);
      }

      f = (sqds + sm) * (sqds - sm) + ztest_0;
      ztest = sqds * LocalizationCore_eml_div_n(e[q], varargin_1_idx);
      for (kase = q + 1; kase <= m + 1; kase++) {
        LocalizationCore_eml_xrotg(&f, &ztest, &sm, &varargin_1_idx);
        if (kase > q + 1) {
          e[0] = f;
        }

        ztest = s[kase - 1] * sm;
        ztest_0 = e[kase - 1] * varargin_1_idx;
        e[kase - 1] = e[kase - 1] * sm - s[kase - 1] * varargin_1_idx;
        b = s[kase];
        s[kase] *= sm;
        LocalizationCore_eml_xrot(Vf, 3 * (kase - 1) + 1, 3 * kase + 1, sm,
          varargin_1_idx);
        ztest_0 += ztest;
        ztest = varargin_1_idx * b;
        LocalizationCore_eml_xrotg(&ztest_0, &ztest, &b, &sm);
        s[kase - 1] = ztest_0;
        f = e[kase - 1] * b + sm * s[kase];
        s[kase] = e[kase - 1] * -sm + b * s[kase];
        ztest = sm * e[kase];
        e[kase] *= b;
        LocalizationCore_eml_xrot_d(U, 50 * (kase - 1) + 1, 50 * kase + 1, b, sm);
      }

      e[m] = f;
      iter++;
      break;

     default:
      if (s[q] < 0.0F) {
        s[q] = -s[q];
        LocalizationCore_eml_xscal_mg(-1.0F, Vf, 3 * q + 1);
      }

      qp = q + 1;
      while ((q + 1 < 3) && (s[q] < s[qp])) {
        ztest_0 = s[q];
        s[q] = s[qp];
        s[qp] = ztest_0;
        LocalizationCore_eml_xswap(Vf, 3 * q + 1, 3 * (q + 1) + 1);
        LocalizationCore_eml_xswap_b(U, 50 * q + 1, 50 * (q + 1) + 1);
        q = qp;
        qp++;
      }

      iter = 0;
      m--;
      break;
    }
  }

  S[0] = s[0];
  S[1] = s[1];
  S[2] = s[2];
  for (qp = 0; qp < 3; qp++) {
    V[3 * qp] = Vf[3 * qp];
    V[1 + 3 * qp] = Vf[3 * qp + 1];
    V[2 + 3 * qp] = Vf[3 * qp + 2];
  }
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static void LocalizationCore_pinv(const real32_T A[150], real32_T X[150])
{
  real32_T V[9];
  int32_T r;
  int32_T vcol;
  real32_T S[9];
  int32_T j;
  real32_T z;
  real32_T U[150];
  real32_T s[3];
  int32_T ar;
  int32_T ia;
  int32_T b;
  int32_T ib;
  int32_T b_ic;
  memset(&X[0], (int32_T)0.0F, 150U * sizeof(real32_T));
  LocalizationCore_eml_xgesvd_l(A, U, s, V);
  for (r = 0; r < 9; r++) {
    S[r] = 0.0F;
  }

  S[0] = s[0];
  S[4] = s[1];
  S[8] = s[2];
  r = 0;
  vcol = 0;
  while ((vcol + 1 < 4) && (S[3 * vcol + vcol] > 50.0F * s[0] * 1.1920929E-7F))
  {
    r++;
    vcol++;
  }

  if (r > 0) {
    vcol = 0;
    for (j = 0; j + 1 <= r; j++) {
      z = 1.0F / S[3 * j + j];
      for (ar = vcol; ar + 1 <= vcol + 3; ar++) {
        V[ar] *= z;
      }

      vcol += 3;
    }

    for (vcol = 0; vcol <= 148; vcol += 3) {
      for (j = vcol; j + 1 <= vcol + 3; j++) {
        X[j] = 0.0F;
      }
    }

    vcol = -1;
    for (j = 0; j <= 148; j += 3) {
      ar = 0;
      vcol++;
      b = ((r - 1) * 50 + vcol) + 1;
      for (ib = vcol; ib + 1 <= b; ib += 50) {
        if (U[ib] != 0.0F) {
          ia = ar;
          for (b_ic = j; b_ic + 1 <= j + 3; b_ic++) {
            ia++;
            X[b_ic] += V[ia - 1] * U[ib];
          }
        }

        ar += 3;
      }
    }
  }
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static void LocalizationCore_abs(const real_T x[3], real_T y[3])
{
  y[0] = fabs(x[0]);
  y[1] = fabs(x[1]);
  y[2] = fabs(x[2]);
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static void LocalizationCore_rdivide(real_T x, const real_T y[3], real_T z[3])
{
  z[0] = x / y[0];
  z[1] = x / y[1];
  z[2] = x / y[2];
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static real_T LocalizationCore_norm_c(const real_T x_data[15], const int32_T
  *x_sizes)
{
  real_T y;
  real_T scale;
  real_T absxk;
  real_T t;
  int32_T k;
  y = 0.0;
  scale = 2.2250738585072014E-308;
  for (k = 1; k <= *x_sizes; k++) {
    absxk = fabs(x_data[k - 1]);
    if (absxk > scale) {
      t = scale / absxk;
      y = y * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * sqrt(y);
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static void LocalizationCore_rdivide_b(const real_T x[3], const real_T y[3],
  real_T z[3])
{
  z[0] = x[0] / y[0];
  z[1] = x[1] / y[1];
  z[2] = x[2] / y[2];
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static void LocalizationCore_power(const real_T a[3], real_T y[3])
{
  y[0] = a[0] * a[0];
  y[1] = a[1] * a[1];
  y[2] = a[2] * a[2];
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static void LocalizationCore_diag(const real_T v[3], real_T d[9])
{
  memset(&d[0], 0, 9U * sizeof(real_T));
  d[0] = v[0];
  d[4] = v[1];
  d[8] = v[2];
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static void Locali_eml_signed_integer_colon(int32_T b, int32_T y_data[3],
  int32_T y_sizes[2])
{
  int32_T n;
  int32_T yk;
  int32_T k;
  if (b < 1) {
    n = 0;
  } else {
    n = b;
  }

  y_sizes[0] = 1;
  y_sizes[1] = n;
  if (n > 0) {
    y_data[0] = 1;
    yk = 1;
    for (k = 2; k <= n; k++) {
      yk++;
      y_data[k - 1] = yk;
    }
  }
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static void LocalizationCore_eml_xtrsv(int32_T n, const real_T A_data[9], const
  int32_T A_sizes[2], int32_T lda, real_T x_data[3])
{
  int32_T jjA;
  int32_T ix;
  int32_T j;
  int32_T b;
  int32_T i;
  if (!((A_sizes[0] == 0) || (A_sizes[1] == 0))) {
    for (j = 0; j + 1 <= n; j++) {
      jjA = j * lda + j;
      b = (n - j) - 1;
      for (i = 1; i <= b; i++) {
        ix = j + i;
        x_data[ix] -= A_data[jjA + i] * x_data[j];
      }
    }
  }
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static real_T LocalizationCore_eml_rcond(const real_T A_data[9], const int32_T
  A_sizes[2], real_T normA)
{
  real_T result;
  real_T ainvnm;
  int32_T iter;
  int32_T kase;
  int32_T jump;
  int32_T j;
  int32_T jlast;
  int32_T k;
  real_T b_temp;
  int32_T i;
  int32_T jjA;
  int32_T ix;
  real_T absrexk;
  real_T x_data[3];
  int32_T x_sizes;
  int8_T b_idx;
  result = 0.0;
  if (A_sizes[0] == 0) {
    result = 1.7976931348623157E+308;
  } else if (A_sizes[0] == 1) {
    if (fabs(A_data[0]) != 0.0) {
      result = 1.0;
    }
  } else {
    if (!(normA == 0.0)) {
      k = A_sizes[0];
      do {
        jlast = 0;
        if (k > 0) {
          if (A_data[((k - 1) * A_sizes[0] + k) - 1] == 0.0) {
            jlast = 1;
          } else {
            k--;
          }
        } else {
          ainvnm = 0.0;
          iter = 2;
          kase = 1;
          jump = 1;
          j = 0;
          b_idx = (int8_T)A_sizes[0];
          x_sizes = b_idx;
          jlast = b_idx;
          for (jjA = 0; jjA < jlast; jjA++) {
            x_data[jjA] = 1.0 / (real_T)A_sizes[0];
          }

          while (kase != 0) {
            if (kase == 1) {
              LocalizationCore_eml_xtrsv(A_sizes[0], A_data, A_sizes, A_sizes[0],
                x_data);
              if (!((A_sizes[0] == 0) || (A_sizes[1] == 0))) {
                for (jlast = A_sizes[0] - 1; jlast + 1 > 0; jlast--) {
                  jjA = jlast * A_sizes[0] + jlast;
                  x_data[jlast] /= A_data[jjA];
                  for (i = 1; i <= jlast; i++) {
                    ix = jlast - i;
                    x_data[ix] -= A_data[jjA - i] * x_data[jlast];
                  }
                }
              }
            } else {
              if (!((A_sizes[0] == 0) || (A_sizes[1] == 0))) {
                for (jlast = 0; jlast + 1 <= A_sizes[0]; jlast++) {
                  jjA = jlast * A_sizes[0];
                  b_temp = x_data[jlast];
                  for (i = 1; i <= jlast; i++) {
                    b_temp -= A_data[(jjA + i) - 1] * x_data[i - 1];
                  }

                  b_temp /= A_data[jjA + jlast];
                  x_data[jlast] = b_temp;
                }

                for (jlast = A_sizes[0] - 1; jlast + 1 > 0; jlast--) {
                  jjA = jlast * A_sizes[0];
                  b_temp = x_data[jlast];
                  for (i = A_sizes[0]; i >= jlast + 2; i--) {
                    b_temp -= A_data[(jjA + i) - 1] * x_data[i - 1];
                  }

                  x_data[jlast] = b_temp;
                }
              }
            }

            if (jump == 1) {
              ainvnm = 0.0;
              for (jlast = 0; jlast < x_sizes; jlast++) {
                ainvnm += fabs(x_data[jlast]);
              }

              for (jjA = 0; jjA < x_sizes; jjA++) {
                b_temp = fabs(x_data[jjA]);
                if (b_temp > 2.2250738585072014E-308) {
                  x_data[jjA] /= b_temp;
                } else {
                  x_data[jjA] = 1.0;
                }
              }

              kase = 2;
              jump = 2;
            } else if (jump == 2) {
              j = 0;
              b_temp = fabs(x_data[0]);
              for (jlast = 1; jlast - 1 <= x_sizes - 2; jlast++) {
                absrexk = fabs(x_data[jlast]);
                if (!(absrexk <= b_temp)) {
                  j = jlast;
                  b_temp = absrexk;
                }
              }

              iter = 2;
              for (jjA = 0; jjA < x_sizes; jjA++) {
                x_data[jjA] = 0.0;
              }

              x_data[j] = 1.0;
              kase = 1;
              jump = 3;
            } else if (jump == 3) {
              ainvnm = 0.0;
              for (jlast = 0; jlast < x_sizes; jlast++) {
                ainvnm += fabs(x_data[jlast]);
              }

              if (ainvnm <= x_data[0]) {
                b_temp = 1.0;
                for (jjA = 0; jjA < x_sizes; jjA++) {
                  x_data[jjA] = (((1.0 + (real_T)jjA) - 1.0) / ((real_T)x_sizes
                    - 1.0) + 1.0) * b_temp;
                  b_temp = -b_temp;
                }

                kase = 1;
                jump = 5;
              } else {
                for (jjA = 0; jjA < x_sizes; jjA++) {
                  b_temp = fabs(x_data[jjA]);
                  if (b_temp > 2.2250738585072014E-308) {
                    x_data[jjA] /= b_temp;
                  } else {
                    x_data[jjA] = 1.0;
                  }
                }

                kase = 2;
                jump = 4;
              }
            } else if (jump == 4) {
              jlast = j;
              j = 0;
              b_temp = fabs(x_data[0]);
              for (jjA = 1; jjA - 1 <= x_sizes - 2; jjA++) {
                absrexk = fabs(x_data[jjA]);
                if (!(absrexk <= b_temp)) {
                  j = jjA;
                  b_temp = absrexk;
                }
              }

              if ((fabs(x_data[jlast]) != fabs(x_data[j])) && (iter <= 5)) {
                iter++;
                for (jjA = 0; jjA < x_sizes; jjA++) {
                  x_data[jjA] = 0.0;
                }

                x_data[j] = 1.0;
                kase = 1;
                jump = 3;
              } else {
                b_temp = 1.0;
                for (jjA = 0; jjA < x_sizes; jjA++) {
                  x_data[jjA] = (((1.0 + (real_T)jjA) - 1.0) / ((real_T)x_sizes
                    - 1.0) + 1.0) * b_temp;
                  b_temp = -b_temp;
                }

                kase = 1;
                jump = 5;
              }
            } else {
              if (jump == 5) {
                b_temp = 0.0;
                for (jlast = 0; jlast < x_sizes; jlast++) {
                  b_temp += fabs(x_data[jlast]);
                }

                b_temp = 2.0 * b_temp / 3.0 / (real_T)A_sizes[0];
                if (b_temp > ainvnm) {
                  ainvnm = b_temp;
                }

                kase = 0;
              }
            }
          }

          if (ainvnm != 0.0) {
            result = 1.0 / ainvnm / normA;
          }

          jlast = 1;
        }
      } while (jlast == 0);
    }
  }

  return result;
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static void LocalizationCore_eml_lusolve(const real_T A_data[9], const int32_T
  A_sizes[2], const real_T B_data[3], const int32_T *B_sizes, real_T X_data[3],
  int32_T *X_sizes, real_T *rc)
{
  real_T oneNormA;
  real_T s;
  int32_T mmj;
  int32_T b_j;
  int32_T b_c;
  int32_T ix;
  real_T b_s;
  int32_T iy;
  int32_T jy;
  int32_T c_ix;
  int32_T c_j;
  int32_T e;
  int32_T ijA;
  real_T b_A_data[9];
  int32_T b_A_sizes[2];
  int32_T ipiv_data[3];
  int32_T ipiv_sizes[2];
  int32_T u;
  if ((A_sizes[0] == 1) || (A_sizes[1] == 1)) {
    oneNormA = 0.0;
    ix = A_sizes[0] * A_sizes[1];
    for (iy = 0; iy < ix; iy++) {
      oneNormA += fabs(A_data[iy]);
    }
  } else {
    oneNormA = 0.0;
    for (ix = 0; ix < A_sizes[1]; ix++) {
      s = 0.0;
      for (iy = 0; iy < A_sizes[0]; iy++) {
        s += fabs(A_data[A_sizes[0] * ix + iy]);
      }

      if (s > oneNormA) {
        oneNormA = s;
      }
    }
  }

  b_A_sizes[0] = A_sizes[0];
  b_A_sizes[1] = A_sizes[1];
  ix = A_sizes[0] * A_sizes[1];
  for (iy = 0; iy < ix; iy++) {
    b_A_data[iy] = A_data[iy];
  }

  ix = A_sizes[1];
  Locali_eml_signed_integer_colon(ix, ipiv_data, ipiv_sizes);
  if (!(A_sizes[1] < 1)) {
    u = A_sizes[1] - 1;
    if (!(u <= A_sizes[1])) {
      u = A_sizes[1];
    }

    for (b_j = 0; b_j + 1 <= u; b_j++) {
      mmj = A_sizes[1] - b_j;
      b_c = (A_sizes[1] + 1) * b_j;
      if (mmj < 1) {
        jy = -1;
      } else {
        jy = 0;
        if (mmj > 1) {
          ix = b_c;
          s = fabs(b_A_data[b_c]);
          for (iy = 1; iy + 1 <= mmj; iy++) {
            ix++;
            b_s = fabs(b_A_data[ix]);
            if (b_s > s) {
              jy = iy;
              s = b_s;
            }
          }
        }
      }

      if (b_A_data[b_c + jy] != 0.0) {
        if (jy != 0) {
          ipiv_data[b_j] = (b_j + jy) + 1;
          ix = b_j;
          iy = b_j + jy;
          for (jy = 1; jy <= A_sizes[1]; jy++) {
            s = b_A_data[ix];
            b_A_data[ix] = b_A_data[iy];
            b_A_data[iy] = s;
            ix += A_sizes[1];
            iy += A_sizes[1];
          }
        }

        ix = b_c + mmj;
        for (iy = b_c + 1; iy + 1 <= ix; iy++) {
          b_A_data[iy] /= b_A_data[b_c];
        }
      }

      ix = (A_sizes[1] - b_j) - 1;
      iy = b_c + A_sizes[1];
      jy = b_c + A_sizes[1];
      for (c_j = 1; c_j <= ix; c_j++) {
        s = b_A_data[jy];
        if (b_A_data[jy] != 0.0) {
          c_ix = b_c + 1;
          e = mmj + iy;
          for (ijA = 1 + iy; ijA + 1 <= e; ijA++) {
            b_A_data[ijA] += b_A_data[c_ix] * -s;
            c_ix++;
          }
        }

        jy += A_sizes[1];
        iy += A_sizes[1];
      }
    }
  }

  *X_sizes = *B_sizes;
  ix = *B_sizes;
  for (iy = 0; iy < ix; iy++) {
    X_data[iy] = B_data[iy];
  }

  for (ix = 0; ix + 1 <= A_sizes[1]; ix++) {
    if (ix + 1 != ipiv_data[ix]) {
      s = X_data[ix];
      X_data[ix] = X_data[ipiv_data[ix] - 1];
      X_data[ipiv_data[ix] - 1] = s;
    }
  }

  if (*B_sizes != 0) {
    for (ix = 0; ix + 1 <= A_sizes[1]; ix++) {
      iy = A_sizes[1] * ix;
      if (X_data[ix] != 0.0) {
        for (jy = ix + 1; jy + 1 <= A_sizes[1]; jy++) {
          X_data[jy] -= b_A_data[jy + iy] * X_data[ix];
        }
      }
    }
  }

  if (*B_sizes != 0) {
    for (ix = A_sizes[1] - 1; ix + 1 > 0; ix--) {
      iy = A_sizes[1] * ix;
      if (X_data[ix] != 0.0) {
        X_data[ix] /= b_A_data[ix + iy];
        for (jy = 0; jy + 1 <= ix; jy++) {
          X_data[jy] -= b_A_data[jy + iy] * X_data[ix];
        }
      }
    }
  }

  *rc = LocalizationCore_eml_rcond(b_A_data, b_A_sizes, oneNormA);
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static real_T LocalizationCore_eml_xnrm2_inai(int32_T n, const real_T x_data[9],
  int32_T ix0)
{
  real_T y;
  real_T scale;
  int32_T kend;
  real_T absxk;
  real_T t;
  int32_T k;
  y = 0.0;
  if (!(n < 1)) {
    if (n == 1) {
      y = fabs(x_data[ix0 - 1]);
    } else {
      scale = 2.2250738585072014E-308;
      kend = (ix0 + n) - 1;
      for (k = ix0; k <= kend; k++) {
        absxk = fabs(x_data[k - 1]);
        if (absxk > scale) {
          t = scale / absxk;
          y = y * t * t + 1.0;
          scale = absxk;
        } else {
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * sqrt(y);
    }
  }

  return y;
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static real_T LocalizationC_eml_matlab_zlarfg(int32_T n, real_T *alpha1, real_T
  x_data[9], int32_T ix0)
{
  real_T tau;
  real_T xnorm;
  int32_T knt;
  int32_T b;
  int32_T b_k;
  tau = 0.0;
  if (!(n <= 0)) {
    xnorm = LocalizationCore_eml_xnrm2_inai(n - 1, x_data, ix0);
    if (xnorm != 0.0) {
      xnorm = hypot(*alpha1, xnorm);
      if (*alpha1 >= 0.0) {
        xnorm = -xnorm;
      }

      if (fabs(xnorm) < 1.0020841800044864E-292) {
        knt = 0;
        do {
          knt++;
          b = (ix0 + n) - 2;
          for (b_k = ix0; b_k <= b; b_k++) {
            x_data[b_k - 1] *= 9.9792015476736E+291;
          }

          xnorm *= 9.9792015476736E+291;
          *alpha1 *= 9.9792015476736E+291;
        } while (!(fabs(xnorm) >= 1.0020841800044864E-292));

        xnorm = LocalizationCore_eml_xnrm2_inai(n - 1, x_data, ix0);
        xnorm = hypot(*alpha1, xnorm);
        if (*alpha1 >= 0.0) {
          xnorm = -xnorm;
        }

        tau = (xnorm - *alpha1) / xnorm;
        *alpha1 = 1.0 / (*alpha1 - xnorm);
        b = (ix0 + n) - 2;
        for (b_k = ix0; b_k <= b; b_k++) {
          x_data[b_k - 1] *= *alpha1;
        }

        for (b = 1; b <= knt; b++) {
          xnorm *= 1.0020841800044864E-292;
        }

        *alpha1 = xnorm;
      } else {
        tau = (xnorm - *alpha1) / xnorm;
        *alpha1 = 1.0 / (*alpha1 - xnorm);
        b = (ix0 + n) - 2;
        for (b_k = ix0; b_k <= b; b_k++) {
          x_data[b_k - 1] *= *alpha1;
        }

        *alpha1 = xnorm;
      }
    }
  }

  return tau;
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static void LocalizationCore_eml_qrsolve(const real_T A_data[9], const int32_T
  A_sizes[2], real_T B_data[3], real_T Y_data[3], int32_T *Y_sizes, real_T
  *rankR)
{
  int32_T mn;
  int32_T b_mn;
  int32_T b_k;
  int32_T i_i;
  int32_T nmi;
  int32_T mmi;
  int32_T pvt;
  real_T atmp;
  int32_T i_ip;
  int32_T f_i;
  int32_T ix;
  real_T smax;
  real_T s;
  int32_T iy;
  int32_T lastv;
  int32_T lastc;
  int32_T b_ia;
  real_T t;
  boolean_T exitg;
  real_T b_A_data[9];
  int32_T b_A_sizes[2];
  real_T tau_data[3];
  int32_T jpvt_data[3];
  int32_T jpvt_sizes[2];
  real_T work_data[3];
  real_T vn1_data[3];
  real_T vn2_data[3];
  int8_T g_idx;
  mn = (int32_T)fmin(A_sizes[0], A_sizes[1]) - 1;
  b_A_sizes[0] = A_sizes[0];
  b_A_sizes[1] = A_sizes[1];
  b_k = A_sizes[0] * A_sizes[1];
  for (ix = 0; ix < b_k; ix++) {
    b_A_data[ix] = A_data[ix];
  }

  if (A_sizes[0] <= A_sizes[1]) {
    b_mn = A_sizes[0];
  } else {
    b_mn = A_sizes[1];
  }

  Locali_eml_signed_integer_colon(A_sizes[1], jpvt_data, jpvt_sizes);
  if (!((A_sizes[0] == 0) || (A_sizes[1] == 0))) {
    b_k = (int8_T)A_sizes[1];
    for (ix = 0; ix < b_k; ix++) {
      work_data[ix] = 0.0;
    }

    b_k = 1;
    for (ix = 0; ix + 1 <= A_sizes[1]; ix++) {
      vn1_data[ix] = LocalizationCore_eml_xnrm2_inai(A_sizes[0], A_data, b_k);
      vn2_data[ix] = vn1_data[ix];
      b_k += A_sizes[0];
    }

    for (f_i = 0; f_i + 1 <= b_mn; f_i++) {
      i_i = f_i * A_sizes[0] + f_i;
      nmi = A_sizes[1] - f_i;
      mmi = (A_sizes[0] - f_i) - 1;
      if (nmi < 1) {
        b_k = -1;
      } else {
        b_k = 0;
        if (nmi > 1) {
          ix = f_i;
          smax = fabs(vn1_data[f_i]);
          for (pvt = 0; pvt + 2 <= nmi; pvt++) {
            ix++;
            s = fabs(vn1_data[ix]);
            if (s > smax) {
              b_k = pvt + 1;
              smax = s;
            }
          }
        }
      }

      pvt = f_i + b_k;
      if (pvt + 1 != f_i + 1) {
        b_k = A_sizes[0] * pvt;
        iy = A_sizes[0] * f_i;
        for (ix = 1; ix <= A_sizes[0]; ix++) {
          smax = b_A_data[b_k];
          b_A_data[b_k] = b_A_data[iy];
          b_A_data[iy] = smax;
          b_k++;
          iy++;
        }

        b_k = jpvt_data[pvt];
        jpvt_data[pvt] = jpvt_data[f_i];
        jpvt_data[f_i] = b_k;
        vn1_data[pvt] = vn1_data[f_i];
        vn2_data[pvt] = vn2_data[f_i];
      }

      if (f_i + 1 < A_sizes[0]) {
        atmp = b_A_data[i_i];
        smax = LocalizationC_eml_matlab_zlarfg(mmi + 1, &atmp, b_A_data, i_i + 2);
        tau_data[f_i] = smax;
      } else {
        atmp = b_A_data[i_i];
        tau_data[f_i] = 0.0;
      }

      b_A_data[i_i] = atmp;
      if (f_i + 1 < A_sizes[1]) {
        atmp = b_A_data[i_i];
        b_A_data[i_i] = 1.0;
        i_ip = ((f_i + 1) * A_sizes[0] + f_i) + 1;
        if (tau_data[f_i] != 0.0) {
          lastv = mmi;
          b_k = i_i + mmi;
          while ((lastv + 1 > 0) && (b_A_data[b_k] == 0.0)) {
            lastv--;
            b_k--;
          }

          lastc = nmi - 1;
          exitg = FALSE;
          while ((!exitg) && (lastc > 0)) {
            b_k = (lastc - 1) * A_sizes[0] + i_ip;
            ix = b_k;
            do {
              iy = 0;
              if (ix <= b_k + lastv) {
                if (b_A_data[ix - 1] != 0.0) {
                  iy = 1;
                } else {
                  ix++;
                }
              } else {
                lastc--;
                iy = 2;
              }
            } while (iy == 0);

            if (iy == 1) {
              exitg = TRUE;
            }
          }
        } else {
          lastv = -1;
          lastc = 0;
        }

        if (lastv + 1 > 0) {
          if (lastc != 0) {
            for (b_k = 1; b_k <= lastc; b_k++) {
              work_data[b_k - 1] = 0.0;
            }

            b_k = 0;
            ix = (lastc - 1) * A_sizes[0] + i_ip;
            for (iy = i_ip; iy <= ix; iy += A_sizes[0]) {
              pvt = i_i;
              smax = 0.0;
              nmi = iy + lastv;
              for (b_ia = iy; b_ia <= nmi; b_ia++) {
                smax += b_A_data[b_ia - 1] * b_A_data[pvt];
                pvt++;
              }

              work_data[b_k] += smax;
              b_k++;
            }
          }

          if (!(-tau_data[f_i] == 0.0)) {
            b_k = 0;
            for (ix = 1; ix <= lastc; ix++) {
              if (work_data[b_k] != 0.0) {
                smax = work_data[b_k] * -tau_data[f_i];
                iy = i_i;
                pvt = lastv + i_ip;
                for (nmi = i_ip; nmi <= pvt; nmi++) {
                  b_A_data[nmi - 1] += b_A_data[iy] * smax;
                  iy++;
                }
              }

              b_k++;
              i_ip += A_sizes[0];
            }
          }
        }

        b_A_data[i_i] = atmp;
      }

      for (b_k = f_i + 1; b_k + 1 <= A_sizes[1]; b_k++) {
        ix = (A_sizes[0] * b_k + f_i) + 1;
        if (vn1_data[b_k] != 0.0) {
          s = fabs(b_A_data[b_A_sizes[0] * b_k + f_i]) / vn1_data[b_k];
          smax = s * s;
          s = 1.0 - s * s;
          if (1.0 - smax < 0.0) {
            s = 0.0;
          }

          smax = vn1_data[b_k] / vn2_data[b_k];
          if (smax * smax * s <= 1.4901161193847656E-8) {
            if (f_i + 1 < A_sizes[0]) {
              smax = 0.0;
              if (!(mmi < 1)) {
                if (mmi == 1) {
                  smax = fabs(b_A_data[ix]);
                } else {
                  s = 2.2250738585072014E-308;
                  for (pvt = ix; pvt + 1 <= ix + 2; pvt++) {
                    atmp = fabs(b_A_data[pvt]);
                    if (atmp > s) {
                      t = s / atmp;
                      smax = smax * t * t + 1.0;
                      s = atmp;
                    } else {
                      t = atmp / s;
                      smax += t * t;
                    }
                  }

                  smax = s * sqrt(smax);
                }
              }

              vn1_data[b_k] = smax;
              vn2_data[b_k] = vn1_data[b_k];
            } else {
              vn1_data[b_k] = 0.0;
              vn2_data[b_k] = 0.0;
            }
          } else {
            vn1_data[b_k] *= sqrt(s);
          }
        }
      }
    }
  }

  *rankR = 0.0;
  if (mn + 1 > 0) {
    b_k = 0;
    while ((b_k <= mn) && (!(fabs(b_A_data[b_A_sizes[0] * b_k + b_k]) <= fmax
             (A_sizes[0], A_sizes[1]) * fabs(b_A_data[0]) *
             2.2204460492503131E-16))) {
      (*rankR)++;
      b_k++;
    }
  }

  g_idx = (int8_T)A_sizes[1];
  *Y_sizes = g_idx;
  b_k = g_idx;
  for (ix = 0; ix < b_k; ix++) {
    Y_data[ix] = 0.0;
  }

  for (pvt = 0; pvt <= mn; pvt++) {
    if (tau_data[pvt] != 0.0) {
      smax = B_data[pvt];
      b_k = A_sizes[0] - pvt;
      for (ix = 0; ix <= b_k - 2; ix++) {
        iy = (pvt + ix) + 1;
        smax += b_A_data[b_A_sizes[0] * pvt + iy] * B_data[iy];
      }

      smax *= tau_data[pvt];
      if (smax != 0.0) {
        B_data[pvt] -= smax;
        b_k = A_sizes[0] - pvt;
        for (ix = 0; ix <= b_k - 2; ix++) {
          iy = (pvt + ix) + 1;
          B_data[iy] -= b_A_data[b_A_sizes[0] * pvt + iy] * smax;
        }
      }
    }
  }

  for (b_k = 0; b_k <= mn; b_k++) {
    Y_data[jpvt_data[b_k] - 1] = B_data[b_k];
  }

  for (b_k = 0; b_k <= mn; b_k++) {
    ix = mn - b_k;
    Y_data[jpvt_data[ix] - 1] /= b_A_data[b_A_sizes[0] * ix + ix];
    for (pvt = 0; pvt < ix; pvt++) {
      Y_data[jpvt_data[pvt] - 1] -= b_A_data[b_A_sizes[0] * ix + pvt] *
        Y_data[jpvt_data[ix] - 1];
    }
  }
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static void LocalizationCore_linsolve(const real_T A_data[9], const int32_T
  A_sizes[2], const real_T B_data[3], const int32_T *B_sizes, real_T C_data[3],
  int32_T *C_sizes, real_T *r)
{
  real_T b_r;
  int32_T i;
  int32_T loop_ub;
  real_T B_data_0[3];
  int32_T B_sizes_0;
  if (A_sizes[0] != A_sizes[1]) {
    B_sizes_0 = *B_sizes;
    loop_ub = *B_sizes;
    for (i = 0; i < loop_ub; i++) {
      B_data_0[i] = B_data[i];
    }

    LocalizationCore_eml_qrsolve(A_data, A_sizes, B_data_0, C_data, C_sizes,
      &b_r);
    *r = b_r;
  } else {
    LocalizationCore_eml_lusolve(A_data, A_sizes, B_data, B_sizes, C_data,
      C_sizes, &b_r);
    *r = b_r;
  }
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static void LocalizationCore_l1eq_pd(real_T x0[3], const real_T A_data[9], const
  int32_T A_sizes[2], const real_T b_data[3])
{
  real_T u[3];
  real_T lamu[3];
  real_T lamu_0[3];
  real_T sdg;
  real_T tau;
  real_T resnorm;
  real_T pditer;
  boolean_T done;
  real_T w[3];
  real_T sig[3];
  real_T sig_0[3];
  real_T sigx[3];
  real_T dx[3];
  real_T dlamu[3];
  real_T s;
  int32_T backiter;
  real_T up[3];
  real_T fu1p[3];
  real_T fu2p[3];
  real_T rdp[6];
  real_T rcp[6];
  real_T b_b[9];
  real_T mtmp;
  int32_T br;
  int32_T ar;
  int32_T ia;
  int32_T b_ia;
  int32_T h_ic;
  static const int8_T gradf[6] = { 0, 0, 0, 1, 1, 1 };

  boolean_T exitg;
  boolean_T exitg_0;
  boolean_T guard;
  real_T w_0[3];
  real_T sig_1[3];
  real_T w_1[3];
  real_T w_2[3];
  real_T w_3[3];
  real_T sigx_0[3];
  real_T lamu_1[3];
  real_T tmp[3];
  int32_T loop_ub;
  real_T lamu_2[6];
  real_T Atv[6];
  real_T lamu_3[6];
  real_T sigx_1[6];
  real_T w_4[6];
  real_T sigx_2[6];
  real_T v_data[3];
  int32_T v_sizes;
  real_T rpri_data[3];
  int32_T rpri_sizes;
  real_T H11p_data[9];
  int32_T H11p_sizes[2];
  real_T Adx_data[3];
  int8_T indn_data[3];
  real_T a_data[9];
  real_T b_a_data[9];
  real_T g_y_data[9];
  real_T varargin_1_data[7];
  int8_T ii_data[3];
  real_T gradf0_data[15];
  real_T rpri_data_0[3];
  int8_T ii_data_0[3];
  int32_T b_a_sizes_idx;
  int8_T c_idx;
  int8_T c_idx_0;
  real_T x_idx;
  real_T x_idx_0;
  real_T x_idx_1;
  real_T Atv_idx;
  real_T Atv_idx_0;
  real_T Atv_idx_1;
  real_T w2_idx;
  real_T w2_idx_0;
  real_T w2_idx_1;
  real_T Atdv_idx;
  real_T Atdv_idx_0;
  real_T Atdv_idx_1;

  /* '<S51>:1:101' */
  /* '<S51>:1:111' */
  /* '<S51>:1:113' */
  x_idx = x0[0];
  x_idx_0 = x0[1];
  x_idx_1 = x0[2];

  /* '<S51>:1:114' */
  LocalizationCore_abs(x0, up);
  mtmp = up[0];
  if (up[1] > up[0]) {
    mtmp = up[1];
  }

  if (up[2] > mtmp) {
    mtmp = up[2];
  }

  mtmp *= 0.1;
  LocalizationCore_abs(x0, u);
  u[0] = 0.95 * u[0] + mtmp;
  u[1] = 0.95 * u[1] + mtmp;
  mtmp += 0.95 * u[2];
  u[2] = mtmp;

  /* '<S51>:1:116' */
  fu1p[0] = x0[0] - u[0];
  fu1p[1] = x0[1] - u[1];
  fu1p[2] = x0[2] - mtmp;

  /* '<S51>:1:117' */
  fu2p[0] = -x0[0] - u[0];
  fu2p[1] = -x0[1] - u[1];
  fu2p[2] = -x0[2] - mtmp;

  /* '<S51>:1:119' */
  LocalizationCore_rdivide(-1.0, fu1p, lamu);

  /* '<S51>:1:120' */
  LocalizationCore_rdivide(-1.0, fu2p, lamu_0);

  /* '<S51>:1:122' */
  loop_ub = A_sizes[0] * A_sizes[1];
  for (h_ic = 0; h_ic < loop_ub; h_ic++) {
    a_data[h_ic] = -A_data[h_ic];
  }

  up[0] = lamu[0] - lamu_0[0];
  up[1] = lamu[1] - lamu_0[1];
  up[2] = lamu[2] - lamu_0[2];
  loop_ub = (int8_T)A_sizes[0];
  for (h_ic = 0; h_ic < loop_ub; h_ic++) {
    v_data[h_ic] = 0.0;
  }

  if (A_sizes[0] != 0) {
    ar = 0;
    while ((A_sizes[0] > 0) && (ar <= 0)) {
      for (ar = 1; ar <= A_sizes[0]; ar++) {
        v_data[ar - 1] = 0.0;
      }

      ar = A_sizes[0];
    }

    br = 0;
    ar = 0;
    while ((A_sizes[0] > 0) && (ar <= 0)) {
      for (b_ia = br; b_ia + 1 <= br + 3; b_ia++) {
        if (up[b_ia] != 0.0) {
          ia = ar;
          for (backiter = 0; backiter + 1 <= A_sizes[0]; backiter++) {
            ia++;
            v_data[backiter] += a_data[ia - 1] * up[b_ia];
          }
        }

        ar += A_sizes[0];
      }

      br += 3;
      ar = A_sizes[0];
    }
  }

  /* '<S51>:1:123' */
  b_a_sizes_idx = A_sizes[0];
  loop_ub = A_sizes[0];
  for (h_ic = 0; h_ic < loop_ub; h_ic++) {
    b_a_data[3 * h_ic] = A_data[h_ic];
    b_a_data[1 + 3 * h_ic] = A_data[h_ic + A_sizes[0]];
    b_a_data[2 + 3 * h_ic] = A_data[(A_sizes[0] << 1) + h_ic];
  }

  if ((A_sizes[0] == 1) || ((int8_T)A_sizes[0] == 1)) {
    Atv_idx = 0.0;
    for (h_ic = 0; h_ic < b_a_sizes_idx; h_ic++) {
      Atv_idx += b_a_data[3 * h_ic] * v_data[h_ic];
    }

    Atv_idx_0 = 0.0;
    for (h_ic = 0; h_ic < b_a_sizes_idx; h_ic++) {
      Atv_idx_0 += b_a_data[3 * h_ic + 1] * v_data[h_ic];
    }

    Atv_idx_1 = 0.0;
    for (h_ic = 0; h_ic < b_a_sizes_idx; h_ic++) {
      Atv_idx_1 += b_a_data[3 * h_ic + 2] * v_data[h_ic];
    }
  } else {
    Atv_idx = 0.0;
    Atv_idx_0 = 0.0;
    Atv_idx_1 = 0.0;
    ar = 0;
    for (backiter = 0; backiter + 1 <= A_sizes[0]; backiter++) {
      if (v_data[backiter] != 0.0) {
        b_ia = ar + 1;
        Atv_idx += b_a_data[b_ia - 1] * v_data[backiter];
        b_ia++;
        Atv_idx_0 += b_a_data[b_ia - 1] * v_data[backiter];
        b_ia++;
        Atv_idx_1 += b_a_data[b_ia - 1] * v_data[backiter];
      }

      ar += 3;
    }
  }

  c_idx = (int8_T)A_sizes[0];
  loop_ub = (int8_T)A_sizes[0];
  rpri_sizes = c_idx;
  for (h_ic = 0; h_ic < loop_ub; h_ic++) {
    rpri_data[h_ic] = 0.0;
  }

  if (A_sizes[0] != 0) {
    ar = 0;
    while ((A_sizes[0] > 0) && (ar <= 0)) {
      for (ar = 1; ar <= A_sizes[0]; ar++) {
        rpri_data[ar - 1] = 0.0;
      }

      ar = A_sizes[0];
    }

    br = 0;
    ar = 0;
    while ((A_sizes[0] > 0) && (ar <= 0)) {
      b_ia = 0;
      for (backiter = br; backiter + 1 <= br + 3; backiter++) {
        if (x0[backiter] != 0.0) {
          ar = b_ia;
          for (ia = 0; ia + 1 <= A_sizes[0]; ia++) {
            ar++;
            rpri_data[ia] += A_data[ar - 1] * x0[backiter];
          }
        }

        b_ia += A_sizes[0];
      }

      br += 3;
      ar = A_sizes[0];
    }
  }

  /* '<S51>:1:124' */
  for (h_ic = 0; h_ic < loop_ub; h_ic++) {
    rpri_data[h_ic] -= b_data[h_ic];
  }

  /* '<S51>:1:126' */
  sdg = -(((fu1p[0] * lamu[0] + fu1p[1] * lamu[1]) + fu1p[2] * lamu[2]) +
          ((fu2p[0] * lamu_0[0] + fu2p[1] * lamu_0[1]) + fu2p[2] * lamu_0[2]));

  /* '<S51>:1:127' */
  tau = 60.0 / sdg;

  /* '<S51>:1:129' */
  mtmp = 1.0 / tau;

  /* '<S51>:1:130' */
  /* '<S51>:1:131' */
  lamu_2[0] = lamu[0] - lamu_0[0];
  lamu_2[1] = lamu[1] - lamu_0[1];
  lamu_2[2] = lamu[2] - lamu_0[2];
  lamu_2[3] = -lamu[0] - lamu_0[0];
  lamu_2[4] = -lamu[1] - lamu_0[1];
  lamu_2[5] = -lamu[2] - lamu_0[2];
  Atv[0] = Atv_idx;
  Atv[1] = Atv_idx_0;
  Atv[2] = Atv_idx_1;
  Atv[3] = 0.0;
  Atv[4] = 0.0;
  Atv[5] = 0.0;
  lamu_3[0] = -lamu[0] * fu1p[0];
  lamu_3[1] = -lamu[1] * fu1p[1];
  lamu_3[2] = -lamu[2] * fu1p[2];
  lamu_3[3] = -lamu_0[0] * fu2p[0];
  lamu_3[4] = -lamu_0[1] * fu2p[1];
  lamu_3[5] = -lamu_0[2] * fu2p[2];
  b_a_sizes_idx = 12 + c_idx;
  for (h_ic = 0; h_ic < 6; h_ic++) {
    gradf0_data[h_ic] = ((real_T)gradf[h_ic] + lamu_2[h_ic]) + Atv[h_ic];
  }

  for (h_ic = 0; h_ic < 6; h_ic++) {
    gradf0_data[h_ic + 6] = lamu_3[h_ic] - mtmp;
  }

  for (h_ic = 0; h_ic < loop_ub; h_ic++) {
    gradf0_data[h_ic + 12] = rpri_data[h_ic];
  }

  resnorm = LocalizationCore_norm_c(gradf0_data, &b_a_sizes_idx);

  /* '<S51>:1:133' */
  pditer = 0.0;

  /* '<S51>:1:134' */
  done = (sdg < 0.001);
  exitg = FALSE;
  while ((!exitg) && (!done)) {
    /* '<S51>:1:135' */
    /* '<S51>:1:137' */
    pditer++;

    /* '<S51>:1:139' */
    mtmp = -1.0 / tau;
    LocalizationCore_rdivide(-1.0, fu1p, rpri_data_0);
    LocalizationCore_rdivide(1.0, fu2p, tmp);
    w[0] = (rpri_data_0[0] + tmp[0]) * mtmp - Atv_idx;
    w[1] = (rpri_data_0[1] + tmp[1]) * mtmp - Atv_idx_0;
    w[2] = (rpri_data_0[2] + tmp[2]) * mtmp - Atv_idx_1;

    /* '<S51>:1:140' */
    mtmp = 1.0 / tau;
    LocalizationCore_rdivide(1.0, fu1p, rpri_data_0);
    LocalizationCore_rdivide(1.0, fu2p, tmp);
    w2_idx = -1.0 - (rpri_data_0[0] + tmp[0]) * mtmp;
    w2_idx_0 = -1.0 - (rpri_data_0[1] + tmp[1]) * mtmp;
    w2_idx_1 = -1.0 - (rpri_data_0[2] + tmp[2]) * mtmp;

    /* '<S51>:1:141' */
    /* '<S51>:1:143' */
    lamu_1[0] = -lamu[0];
    lamu_1[1] = -lamu[1];
    lamu_1[2] = -lamu[2];
    LocalizationCore_rdivide_b(lamu_1, fu1p, rpri_data_0);
    LocalizationCore_rdivide_b(lamu_0, fu2p, tmp);
    sig[0] = rpri_data_0[0] - tmp[0];
    sig[1] = rpri_data_0[1] - tmp[1];
    sig[2] = rpri_data_0[2] - tmp[2];

    /* '<S51>:1:144' */
    LocalizationCore_rdivide_b(lamu, fu1p, rpri_data_0);
    LocalizationCore_rdivide_b(lamu_0, fu2p, tmp);
    sig_0[0] = rpri_data_0[0] - tmp[0];
    sig_0[1] = rpri_data_0[1] - tmp[1];
    sig_0[2] = rpri_data_0[2] - tmp[2];

    /* '<S51>:1:145' */
    LocalizationCore_power(sig_0, rpri_data_0);
    LocalizationCore_rdivide_b(rpri_data_0, sig, tmp);
    sigx[0] = sig[0] - tmp[0];
    sigx[1] = sig[1] - tmp[1];
    sigx[2] = sig[2] - tmp[2];

    /* '<S51>:1:147' */
    loop_ub = A_sizes[0] * A_sizes[1];
    for (h_ic = 0; h_ic < loop_ub; h_ic++) {
      a_data[h_ic] = -A_data[h_ic];
    }

    LocalizationCore_rdivide(1.0, sigx, rpri_data_0);
    LocalizationCore_diag(rpri_data_0, b_b);
    c_idx = (int8_T)A_sizes[0];
    loop_ub = c_idx * 3;
    for (h_ic = 0; h_ic < loop_ub; h_ic++) {
      g_y_data[h_ic] = 0.0;
    }

    if (A_sizes[0] != 0) {
      b_a_sizes_idx = A_sizes[0] << 1;
      ar = 0;
      while ((A_sizes[0] > 0) && (ar <= b_a_sizes_idx)) {
        backiter = ar + A_sizes[0];
        for (b_ia = ar; b_ia + 1 <= backiter; b_ia++) {
          g_y_data[b_ia] = 0.0;
        }

        ar += A_sizes[0];
      }

      b_ia = 0;
      backiter = 0;
      while ((A_sizes[0] > 0) && (backiter <= b_a_sizes_idx)) {
        ar = 0;
        for (ia = b_ia; ia + 1 <= b_ia + 3; ia++) {
          if (b_b[ia] != 0.0) {
            br = ar;
            loop_ub = backiter + A_sizes[0];
            for (h_ic = backiter; h_ic + 1 <= loop_ub; h_ic++) {
              br++;
              g_y_data[h_ic] += a_data[br - 1] * b_b[ia];
            }
          }

          ar += A_sizes[0];
        }

        b_ia += 3;
        backiter += A_sizes[0];
      }
    }

    loop_ub = A_sizes[0];
    for (h_ic = 0; h_ic < loop_ub; h_ic++) {
      b_a_data[3 * h_ic] = A_data[h_ic];
      b_a_data[1 + 3 * h_ic] = A_data[h_ic + A_sizes[0]];
      b_a_data[2 + 3 * h_ic] = A_data[(A_sizes[0] << 1) + h_ic];
    }

    c_idx_0 = (int8_T)A_sizes[0];
    H11p_sizes[0] = c_idx;
    H11p_sizes[1] = c_idx_0;
    loop_ub = c_idx * c_idx_0;
    for (h_ic = 0; h_ic < loop_ub; h_ic++) {
      H11p_data[h_ic] = 0.0;
    }

    if (!((c_idx == 0) || (A_sizes[0] == 0))) {
      b_a_sizes_idx = (A_sizes[0] - 1) * c_idx;
      ar = 0;
      while ((c_idx > 0) && (ar <= b_a_sizes_idx)) {
        backiter = ar + c_idx;
        for (b_ia = ar; b_ia + 1 <= backiter; b_ia++) {
          H11p_data[b_ia] = 0.0;
        }

        ar += c_idx;
      }

      b_ia = 0;
      backiter = 0;
      while ((c_idx > 0) && (backiter <= b_a_sizes_idx)) {
        ar = 0;
        for (ia = b_ia; ia + 1 <= b_ia + 3; ia++) {
          if (b_a_data[ia] != 0.0) {
            br = ar;
            loop_ub = backiter + c_idx;
            for (h_ic = backiter; h_ic + 1 <= loop_ub; h_ic++) {
              br++;
              H11p_data[h_ic] += g_y_data[br - 1] * b_a_data[ia];
            }
          }

          ar += c_idx;
        }

        b_ia += 3;
        backiter += c_idx;
      }
    }

    LocalizationCore_rdivide_b(w, sigx, rpri_data_0);
    w_3[0] = w2_idx * sig_0[0];
    w_3[1] = w2_idx_0 * sig_0[1];
    w_3[2] = w2_idx_1 * sig_0[2];
    sigx_0[0] = sigx[0] * sig[0];
    sigx_0[1] = sigx[1] * sig[1];
    sigx_0[2] = sigx[2] * sig[2];
    LocalizationCore_rdivide_b(w_3, sigx_0, tmp);
    up[0] = rpri_data_0[0] - tmp[0];
    up[1] = rpri_data_0[1] - tmp[1];
    up[2] = rpri_data_0[2] - tmp[2];
    loop_ub = (int8_T)A_sizes[0];
    v_sizes = (int8_T)A_sizes[0];
    for (h_ic = 0; h_ic < loop_ub; h_ic++) {
      v_data[h_ic] = 0.0;
    }

    if (A_sizes[0] != 0) {
      ar = 0;
      while ((A_sizes[0] > 0) && (ar <= 0)) {
        for (ar = 1; ar <= A_sizes[0]; ar++) {
          v_data[ar - 1] = 0.0;
        }

        ar = A_sizes[0];
      }

      br = 0;
      ar = 0;
      while ((A_sizes[0] > 0) && (ar <= 0)) {
        b_ia = 0;
        for (backiter = br; backiter + 1 <= br + 3; backiter++) {
          if (up[backiter] != 0.0) {
            ar = b_ia;
            for (ia = 0; ia + 1 <= A_sizes[0]; ia++) {
              ar++;
              v_data[ia] += A_data[ar - 1] * up[backiter];
            }
          }

          b_ia += A_sizes[0];
        }

        br += 3;
        ar = A_sizes[0];
      }
    }

    /* '<S51>:1:148' */
    for (h_ic = 0; h_ic < rpri_sizes; h_ic++) {
      rpri_data_0[h_ic] = -rpri_data[h_ic] - v_data[h_ic];
    }

    LocalizationCore_linsolve(H11p_data, H11p_sizes, rpri_data_0, &rpri_sizes,
      v_data, &v_sizes, &mtmp);
    if (mtmp < 1.0E-14) {
      /* '<S51>:1:150' */
      /* '<S51>:1:152' */
      exitg = TRUE;
    } else {
      /* '<S51>:1:155' */
      b_a_sizes_idx = A_sizes[0];
      loop_ub = A_sizes[0];
      for (h_ic = 0; h_ic < loop_ub; h_ic++) {
        b_a_data[3 * h_ic] = A_data[h_ic];
        b_a_data[1 + 3 * h_ic] = A_data[h_ic + A_sizes[0]];
        b_a_data[2 + 3 * h_ic] = A_data[(A_sizes[0] << 1) + h_ic];
      }

      if ((A_sizes[0] == 1) || (v_sizes == 1)) {
        up[0] = 0.0;
        mtmp = 0.0;
        for (h_ic = 0; h_ic < b_a_sizes_idx; h_ic++) {
          mtmp += b_a_data[3 * h_ic] * v_data[h_ic];
          up[0] = mtmp;
        }

        up[1] = 0.0;
        mtmp = 0.0;
        for (h_ic = 0; h_ic < b_a_sizes_idx; h_ic++) {
          mtmp += b_a_data[3 * h_ic + 1] * v_data[h_ic];
          up[1] = mtmp;
        }

        up[2] = 0.0;
        mtmp = 0.0;
        for (h_ic = 0; h_ic < b_a_sizes_idx; h_ic++) {
          mtmp += b_a_data[3 * h_ic + 2] * v_data[h_ic];
          up[2] = mtmp;
        }
      } else {
        up[0] = 0.0;
        up[1] = 0.0;
        up[2] = 0.0;
        ar = 0;
        for (backiter = 0; backiter + 1 <= A_sizes[0]; backiter++) {
          if (v_data[backiter] != 0.0) {
            b_ia = ar + 1;
            up[0] += b_a_data[b_ia - 1] * v_data[backiter];
            b_ia++;
            up[1] += b_a_data[b_ia - 1] * v_data[backiter];
            b_ia++;
            up[2] += b_a_data[b_ia - 1] * v_data[backiter];
          }

          ar += 3;
        }
      }

      w_1[0] = w2_idx * sig_0[0];
      w_1[1] = w2_idx_0 * sig_0[1];
      w_1[2] = w2_idx_1 * sig_0[2];
      LocalizationCore_rdivide_b(w_1, sig, rpri_data_0);
      w_2[0] = (w[0] - rpri_data_0[0]) - up[0];
      w_2[1] = (w[1] - rpri_data_0[1]) - up[1];
      w_2[2] = (w[2] - rpri_data_0[2]) - up[2];
      LocalizationCore_rdivide_b(w_2, sigx, dx);

      /* '<S51>:1:156' */
      loop_ub = (int8_T)A_sizes[0];
      for (h_ic = 0; h_ic < loop_ub; h_ic++) {
        Adx_data[h_ic] = 0.0;
      }

      if (A_sizes[0] != 0) {
        ar = 0;
        while ((A_sizes[0] > 0) && (ar <= 0)) {
          for (ar = 1; ar <= A_sizes[0]; ar++) {
            Adx_data[ar - 1] = 0.0;
          }

          ar = A_sizes[0];
        }

        br = 0;
        ar = 0;
        while ((A_sizes[0] > 0) && (ar <= 0)) {
          b_ia = 0;
          for (backiter = br; backiter + 1 <= br + 3; backiter++) {
            if (dx[backiter] != 0.0) {
              ar = b_ia;
              for (ia = 0; ia + 1 <= A_sizes[0]; ia++) {
                ar++;
                Adx_data[ia] += A_data[ar - 1] * dx[backiter];
              }
            }

            b_ia += A_sizes[0];
          }

          br += 3;
          ar = A_sizes[0];
        }
      }

      /* '<S51>:1:157' */
      b_a_sizes_idx = A_sizes[0];
      loop_ub = A_sizes[0];
      for (h_ic = 0; h_ic < loop_ub; h_ic++) {
        b_a_data[3 * h_ic] = A_data[h_ic];
        b_a_data[1 + 3 * h_ic] = A_data[h_ic + A_sizes[0]];
        b_a_data[2 + 3 * h_ic] = A_data[(A_sizes[0] << 1) + h_ic];
      }

      if ((A_sizes[0] == 1) || (v_sizes == 1)) {
        Atdv_idx = 0.0;
        for (h_ic = 0; h_ic < b_a_sizes_idx; h_ic++) {
          Atdv_idx += b_a_data[3 * h_ic] * v_data[h_ic];
        }

        Atdv_idx_0 = 0.0;
        for (h_ic = 0; h_ic < b_a_sizes_idx; h_ic++) {
          Atdv_idx_0 += b_a_data[3 * h_ic + 1] * v_data[h_ic];
        }

        Atdv_idx_1 = 0.0;
        for (h_ic = 0; h_ic < b_a_sizes_idx; h_ic++) {
          Atdv_idx_1 += b_a_data[3 * h_ic + 2] * v_data[h_ic];
        }
      } else {
        Atdv_idx = 0.0;
        Atdv_idx_0 = 0.0;
        Atdv_idx_1 = 0.0;
        ar = 0;
        for (backiter = 0; backiter + 1 <= A_sizes[0]; backiter++) {
          if (v_data[backiter] != 0.0) {
            b_ia = ar + 1;
            Atdv_idx += b_a_data[b_ia - 1] * v_data[backiter];
            b_ia++;
            Atdv_idx_0 += b_a_data[b_ia - 1] * v_data[backiter];
            b_ia++;
            Atdv_idx_1 += b_a_data[b_ia - 1] * v_data[backiter];
          }

          ar += 3;
        }
      }

      /* '<S51>:1:159' */
      w_0[0] = w2_idx - sig_0[0] * dx[0];
      w_0[1] = w2_idx_0 - sig_0[1] * dx[1];
      w_0[2] = w2_idx_1 - sig_0[2] * dx[2];
      sig_1[0] = sig[0];
      sig_1[1] = sig[1];
      sig_1[2] = sig[2];
      LocalizationCore_rdivide_b(w_0, sig_1, sig);

      /* '<S51>:1:161' */
      LocalizationCore_rdivide_b(lamu, fu1p, rpri_data_0);
      LocalizationCore_rdivide(1.0 / tau, fu1p, tmp);
      sig_0[0] = ((-dx[0] + sig[0]) * rpri_data_0[0] - lamu[0]) - tmp[0];
      sig_0[1] = ((-dx[1] + sig[1]) * rpri_data_0[1] - lamu[1]) - tmp[1];
      sig_0[2] = ((-dx[2] + sig[2]) * rpri_data_0[2] - lamu[2]) - tmp[2];

      /* '<S51>:1:162' */
      LocalizationCore_rdivide_b(lamu_0, fu2p, rpri_data_0);
      LocalizationCore_rdivide(1.0 / tau, fu2p, tmp);
      dlamu[0] = ((dx[0] + sig[0]) * rpri_data_0[0] - lamu_0[0]) - tmp[0];
      dlamu[1] = ((dx[1] + sig[1]) * rpri_data_0[1] - lamu_0[1]) - tmp[1];
      dlamu[2] = ((dx[2] + sig[2]) * rpri_data_0[2] - lamu_0[2]) - tmp[2];

      /*  make sure that the step is feasible: keeps lamu1,lamu2 > 0, fu1,fu2 < 0 */
      /* '<S51>:1:165' */
      ar = 0;
      backiter = 1;
      exitg_0 = FALSE;
      while ((!exitg_0) && (backiter < 4)) {
        guard = FALSE;
        if (sig_0[backiter - 1] < 0.0) {
          ar++;
          ii_data[ar - 1] = (int8_T)backiter;
          if (ar >= 3) {
            exitg_0 = TRUE;
          } else {
            guard = TRUE;
          }
        } else {
          guard = TRUE;
        }

        if (guard) {
          backiter++;
        }
      }

      if (1 > ar) {
        ar = 0;
      }

      for (h_ic = 0; h_ic < ar; h_ic++) {
        ii_data_0[h_ic] = ii_data[h_ic];
      }

      for (h_ic = 0; h_ic < ar; h_ic++) {
        ii_data[h_ic] = ii_data_0[h_ic];
      }

      v_sizes = ar + 1;
      for (h_ic = 0; h_ic < ar; h_ic++) {
        v_data[h_ic] = ii_data[h_ic];
      }

      /* '<S51>:1:165' */
      ar = 0;
      backiter = 1;
      exitg_0 = FALSE;
      while ((!exitg_0) && (backiter < 4)) {
        guard = FALSE;
        if (dlamu[backiter - 1] < 0.0) {
          ar++;
          ii_data[ar - 1] = (int8_T)backiter;
          if (ar >= 3) {
            exitg_0 = TRUE;
          } else {
            guard = TRUE;
          }
        } else {
          guard = TRUE;
        }

        if (guard) {
          backiter++;
        }
      }

      if (1 > ar) {
        ar = 0;
      }

      for (h_ic = 0; h_ic < ar; h_ic++) {
        ii_data_0[h_ic] = ii_data[h_ic];
      }

      for (h_ic = 0; h_ic < ar; h_ic++) {
        ii_data[h_ic] = ii_data_0[h_ic];
      }

      for (h_ic = 0; h_ic < ar; h_ic++) {
        indn_data[h_ic] = ii_data[h_ic];
      }

      /* '<S51>:1:166' */
      b_a_sizes_idx = v_sizes + ar;
      varargin_1_data[0] = 1.0;
      loop_ub = v_sizes - 1;
      for (h_ic = 0; h_ic < loop_ub; h_ic++) {
        varargin_1_data[h_ic + 1] = -lamu[(int32_T)v_data[h_ic] - 1] / sig_0
          [(int32_T)v_data[h_ic] - 1];
      }

      for (h_ic = 0; h_ic < ar; h_ic++) {
        varargin_1_data[h_ic + v_sizes] = -lamu_0[indn_data[h_ic] - 1] /
          dlamu[indn_data[h_ic] - 1];
      }

      mtmp = varargin_1_data[0];
      if ((b_a_sizes_idx > 1) && (1 < b_a_sizes_idx)) {
        for (ar = 1; ar + 1 <= b_a_sizes_idx; ar++) {
          if (varargin_1_data[ar] < mtmp) {
            mtmp = varargin_1_data[ar];
          }
        }
      }

      /* '<S51>:1:167' */
      ar = 0;
      backiter = 1;
      exitg_0 = FALSE;
      while ((!exitg_0) && (backiter < 4)) {
        guard = FALSE;
        b_a_sizes_idx = backiter - 1;
        if (dx[b_a_sizes_idx] - sig[b_a_sizes_idx] > 0.0) {
          ar++;
          ii_data[ar - 1] = (int8_T)backiter;
          if (ar >= 3) {
            exitg_0 = TRUE;
          } else {
            guard = TRUE;
          }
        } else {
          guard = TRUE;
        }

        if (guard) {
          backiter++;
        }
      }

      if (1 > ar) {
        ar = 0;
      }

      for (h_ic = 0; h_ic < ar; h_ic++) {
        ii_data_0[h_ic] = ii_data[h_ic];
      }

      for (h_ic = 0; h_ic < ar; h_ic++) {
        ii_data[h_ic] = ii_data_0[h_ic];
      }

      v_sizes = ar + 1;
      for (h_ic = 0; h_ic < ar; h_ic++) {
        v_data[h_ic] = ii_data[h_ic];
      }

      /* '<S51>:1:167' */
      ar = 0;
      backiter = 1;
      exitg_0 = FALSE;
      while ((!exitg_0) && (backiter < 4)) {
        guard = FALSE;
        b_a_sizes_idx = backiter - 1;
        if (-dx[b_a_sizes_idx] - sig[b_a_sizes_idx] > 0.0) {
          ar++;
          ii_data[ar - 1] = (int8_T)backiter;
          if (ar >= 3) {
            exitg_0 = TRUE;
          } else {
            guard = TRUE;
          }
        } else {
          guard = TRUE;
        }

        if (guard) {
          backiter++;
        }
      }

      if (1 > ar) {
        ar = 0;
      }

      for (h_ic = 0; h_ic < ar; h_ic++) {
        ii_data_0[h_ic] = ii_data[h_ic];
      }

      for (h_ic = 0; h_ic < ar; h_ic++) {
        ii_data[h_ic] = ii_data_0[h_ic];
      }

      for (h_ic = 0; h_ic < ar; h_ic++) {
        indn_data[h_ic] = ii_data[h_ic];
      }

      /* '<S51>:1:168' */
      b_a_sizes_idx = v_sizes + ar;
      varargin_1_data[0] = mtmp;
      loop_ub = v_sizes - 1;
      for (h_ic = 0; h_ic < loop_ub; h_ic++) {
        varargin_1_data[h_ic + 1] = -fu1p[(int32_T)v_data[h_ic] - 1] / (dx
          [(int32_T)v_data[h_ic] - 1] - sig[(int32_T)v_data[h_ic] - 1]);
      }

      for (h_ic = 0; h_ic < ar; h_ic++) {
        varargin_1_data[h_ic + v_sizes] = -fu2p[indn_data[h_ic] - 1] /
          (-dx[indn_data[h_ic] - 1] - sig[indn_data[h_ic] - 1]);
      }

      mtmp = varargin_1_data[0];
      if ((b_a_sizes_idx > 1) && (1 < b_a_sizes_idx)) {
        for (ar = 1; ar + 1 <= b_a_sizes_idx; ar++) {
          if (varargin_1_data[ar] < mtmp) {
            mtmp = varargin_1_data[ar];
          }
        }
      }

      s = 0.99 * mtmp;

      /*  backtracking line search  */
      /* '<S51>:1:171' */
      backiter = 0;

      /* '<S51>:1:172' */
      x0[0] = s * dx[0] + x_idx;
      x0[1] = s * dx[1] + x_idx_0;
      x0[2] = s * dx[2] + x_idx_1;

      /* '<S51>:1:172' */
      up[0] = s * sig[0] + u[0];
      up[1] = s * sig[1] + u[1];
      up[2] = s * sig[2] + u[2];

      /* '<S51>:1:173' */
      w[0] = s * Atdv_idx + Atv_idx;
      w[1] = s * Atdv_idx_0 + Atv_idx_0;
      w[2] = s * Atdv_idx_1 + Atv_idx_1;

      /* '<S51>:1:174' */
      sigx[0] = s * sig_0[0] + lamu[0];
      sigx[1] = s * sig_0[1] + lamu[1];
      sigx[2] = s * sig_0[2] + lamu[2];

      /* '<S51>:1:174' */
      w2_idx = s * dlamu[0] + lamu_0[0];
      w2_idx_0 = s * dlamu[1] + lamu_0[1];
      w2_idx_1 = s * dlamu[2] + lamu_0[2];

      /* '<S51>:1:175' */
      fu1p[0] = x0[0] - up[0];
      fu1p[1] = x0[1] - up[1];
      fu1p[2] = x0[2] - up[2];

      /* '<S51>:1:175' */
      fu2p[0] = -x0[0] - up[0];
      fu2p[1] = -x0[1] - up[1];
      fu2p[2] = -x0[2] - up[2];

      /* '<S51>:1:176' */
      rdp[0] = (sigx[0] - w2_idx) + w[0];
      rdp[1] = (sigx[1] - w2_idx_0) + w[1];
      rdp[2] = (sigx[2] - w2_idx_1) + w[2];
      rdp[3] = (-sigx[0] - w2_idx) + 1.0;
      rdp[4] = (-sigx[1] - w2_idx_0) + 1.0;
      rdp[5] = (-sigx[2] - w2_idx_1) + 1.0;

      /* '<S51>:1:177' */
      mtmp = 1.0 / tau;
      rcp[0] = -sigx[0] * fu1p[0] - mtmp;
      rcp[1] = -sigx[1] * fu1p[1] - mtmp;
      rcp[2] = -sigx[2] * fu1p[2] - mtmp;
      rcp[3] = -w2_idx * fu2p[0] - mtmp;
      rcp[4] = -w2_idx_0 * fu2p[1] - mtmp;
      rcp[5] = -w2_idx_1 * fu2p[2] - mtmp;

      /* '<S51>:1:178' */
      v_sizes = rpri_sizes;
      for (h_ic = 0; h_ic < rpri_sizes; h_ic++) {
        v_data[h_ic] = s * Adx_data[h_ic] + rpri_data[h_ic];
      }

      do {
        ar = 0;
        b_a_sizes_idx = 12 + v_sizes;
        for (h_ic = 0; h_ic < 6; h_ic++) {
          gradf0_data[h_ic] = rdp[h_ic];
        }

        for (h_ic = 0; h_ic < 6; h_ic++) {
          gradf0_data[h_ic + 6] = rcp[h_ic];
        }

        for (h_ic = 0; h_ic < v_sizes; h_ic++) {
          gradf0_data[h_ic + 12] = v_data[h_ic];
        }

        if (LocalizationCore_norm_c(gradf0_data, &b_a_sizes_idx) > (1.0 - 0.01 *
             s) * resnorm) {
          /* '<S51>:1:179' */
          /* '<S51>:1:180' */
          s *= 0.5;

          /* '<S51>:1:181' */
          x0[0] = s * dx[0] + x_idx;
          x0[1] = s * dx[1] + x_idx_0;
          x0[2] = s * dx[2] + x_idx_1;

          /* '<S51>:1:181' */
          up[0] = s * sig[0] + u[0];
          up[1] = s * sig[1] + u[1];
          up[2] = s * sig[2] + u[2];

          /* '<S51>:1:182' */
          w[0] = s * Atdv_idx + Atv_idx;
          w[1] = s * Atdv_idx_0 + Atv_idx_0;
          w[2] = s * Atdv_idx_1 + Atv_idx_1;

          /* '<S51>:1:183' */
          sigx[0] = s * sig_0[0] + lamu[0];
          sigx[1] = s * sig_0[1] + lamu[1];
          sigx[2] = s * sig_0[2] + lamu[2];

          /* '<S51>:1:183' */
          w2_idx = s * dlamu[0] + lamu_0[0];
          w2_idx_0 = s * dlamu[1] + lamu_0[1];
          w2_idx_1 = s * dlamu[2] + lamu_0[2];

          /* '<S51>:1:184' */
          fu1p[0] = x0[0] - up[0];
          fu1p[1] = x0[1] - up[1];
          fu1p[2] = x0[2] - up[2];

          /* '<S51>:1:184' */
          fu2p[0] = -x0[0] - up[0];
          fu2p[1] = -x0[1] - up[1];
          fu2p[2] = -x0[2] - up[2];

          /* '<S51>:1:185' */
          rdp[0] = (sigx[0] - w2_idx) + w[0];
          rdp[1] = (sigx[1] - w2_idx_0) + w[1];
          rdp[2] = (sigx[2] - w2_idx_1) + w[2];
          rdp[3] = (-sigx[0] - w2_idx) + 1.0;
          rdp[4] = (-sigx[1] - w2_idx_0) + 1.0;
          rdp[5] = (-sigx[2] - w2_idx_1) + 1.0;

          /* '<S51>:1:186' */
          mtmp = 1.0 / tau;
          rcp[0] = -sigx[0] * fu1p[0] - mtmp;
          rcp[1] = -sigx[1] * fu1p[1] - mtmp;
          rcp[2] = -sigx[2] * fu1p[2] - mtmp;
          rcp[3] = -w2_idx * fu2p[0] - mtmp;
          rcp[4] = -w2_idx_0 * fu2p[1] - mtmp;
          rcp[5] = -w2_idx_1 * fu2p[2] - mtmp;

          /* '<S51>:1:187' */
          for (h_ic = 0; h_ic < rpri_sizes; h_ic++) {
            v_data[h_ic] = s * Adx_data[h_ic] + rpri_data[h_ic];
          }

          /* '<S51>:1:188' */
          backiter++;
          if (backiter > 32) {
            /* '<S51>:1:189' */
            /* '<S51>:1:191' */
            x0[0] = x_idx;
            x0[1] = x_idx_0;
            x0[2] = x_idx_1;
            ar = 1;
          }
        } else {
          /*  next iteration */
          /* '<S51>:1:198' */
          x_idx = x0[0];
          x_idx_0 = x0[1];
          x_idx_1 = x0[2];

          /* '<S51>:1:198' */
          u[0] = up[0];
          u[1] = up[1];
          u[2] = up[2];

          /* '<S51>:1:199' */
          Atv_idx = w[0];
          Atv_idx_0 = w[1];
          Atv_idx_1 = w[2];

          /* '<S51>:1:200' */
          lamu[0] = sigx[0];
          lamu[1] = sigx[1];
          lamu[2] = sigx[2];

          /* '<S51>:1:200' */
          lamu_0[0] = w2_idx;
          lamu_0[1] = w2_idx_0;
          lamu_0[2] = w2_idx_1;

          /* '<S51>:1:201' */
          /*  surrogate duality gap */
          /* '<S51>:1:204' */
          sdg = -(((fu1p[0] * sigx[0] + fu1p[1] * sigx[1]) + fu1p[2] * sigx[2])
                  + ((fu2p[0] * w2_idx + fu2p[1] * w2_idx_0) + fu2p[2] *
                     w2_idx_1));

          /* '<S51>:1:205' */
          tau = 60.0 / sdg;

          /* '<S51>:1:206' */
          for (h_ic = 0; h_ic < v_sizes; h_ic++) {
            rpri_data[h_ic] = v_data[h_ic];
          }

          /* '<S51>:1:207' */
          mtmp = 1.0 / tau;

          /* '<S51>:1:208' */
          /* '<S51>:1:209' */
          sigx_1[0] = sigx[0] - w2_idx;
          sigx_1[1] = sigx[1] - w2_idx_0;
          sigx_1[2] = sigx[2] - w2_idx_1;
          sigx_1[3] = -sigx[0] - w2_idx;
          sigx_1[4] = -sigx[1] - w2_idx_0;
          sigx_1[5] = -sigx[2] - w2_idx_1;
          w_4[0] = w[0];
          w_4[1] = w[1];
          w_4[2] = w[2];
          w_4[3] = 0.0;
          w_4[4] = 0.0;
          w_4[5] = 0.0;
          sigx_2[0] = -sigx[0] * fu1p[0];
          sigx_2[1] = -sigx[1] * fu1p[1];
          sigx_2[2] = -sigx[2] * fu1p[2];
          sigx_2[3] = -w2_idx * fu2p[0];
          sigx_2[4] = -w2_idx_0 * fu2p[1];
          sigx_2[5] = -w2_idx_1 * fu2p[2];
          b_a_sizes_idx = 12 + v_sizes;
          for (h_ic = 0; h_ic < 6; h_ic++) {
            gradf0_data[h_ic] = ((real_T)gradf[h_ic] + sigx_1[h_ic]) + w_4[h_ic];
          }

          for (h_ic = 0; h_ic < 6; h_ic++) {
            gradf0_data[h_ic + 6] = sigx_2[h_ic] - mtmp;
          }

          for (h_ic = 0; h_ic < v_sizes; h_ic++) {
            gradf0_data[h_ic + 12] = v_data[h_ic];
          }

          resnorm = LocalizationCore_norm_c(gradf0_data, &b_a_sizes_idx);

          /* '<S51>:1:211' */
          done = ((sdg < 0.001) || (pditer >= 50.0));

          /*    disp(sprintf('Iteration = %d, tau = %8.3e, Primal = %8.3e, PDGap = %8.3e, Dual res = %8.3e, Primal res = %8.3e',... */
          /*      pditer, tau, sum(u), sdg, norm(rdual), norm(rpri))); */
          /*    if (largescale) */
          /*      disp(sprintf('                  CG Res = %8.3e, CG Iter = %d', cgres, cgiter)); */
          /*    else */
          /*      disp(sprintf('                  H11p condition number = %8.3e', hcond)); */
          /*    end */
          ar = 2;
        }
      } while (ar == 0);

      if (ar == 1) {
        exitg = TRUE;
      }
    }
  }
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static void LocalizationCore_eml_sort_lvl1f(const real32_T x[3], real32_T y[3],
  int32_T idx[3])
{
  if (x[0] >= x[1]) {
    if (x[1] >= x[2]) {
      idx[0] = 1;
      idx[1] = 2;
      idx[2] = 3;
      y[0] = x[0];
      y[1] = x[1];
      y[2] = x[2];
    } else if (x[0] >= x[2]) {
      idx[0] = 1;
      idx[1] = 3;
      idx[2] = 2;
      y[0] = x[0];
      y[1] = x[2];
      y[2] = x[1];
    } else {
      idx[0] = 3;
      idx[1] = 1;
      idx[2] = 2;
      y[0] = x[2];
      y[1] = x[0];
      y[2] = x[1];
    }
  } else if (x[0] >= x[2]) {
    idx[0] = 2;
    idx[1] = 1;
    idx[2] = 3;
    y[0] = x[1];
    y[1] = x[0];
    y[2] = x[2];
  } else if (x[1] >= x[2]) {
    idx[0] = 2;
    idx[1] = 3;
    idx[2] = 1;
    y[0] = x[1];
    y[1] = x[2];
    y[2] = x[0];
  } else {
    idx[0] = 3;
    idx[1] = 2;
    idx[2] = 1;
    y[0] = x[2];
    y[1] = x[1];
    y[2] = x[0];
  }
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static void LocalizationCore_eml_sort_lvl1(const real32_T x[3], real32_T y[3],
  int32_T idx[3])
{
  LocalizationCore_eml_sort_lvl1f(x, y, idx);
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static real32_T LocalizationCore_sum(const real32_T x[3])
{
  return (x[0] + x[1]) + x[2];
}

/* Function for MATLAB Function: '<S50>/kNN Localization core' */
static void LocalizationCore_eml_sort_d(const real_T x[2048], real_T y[2048],
  int32_T idx[2048])
{
  int32_T k;
  int16_T idx_0[2048];
  int32_T i;
  int32_T j;
  int32_T pEnd;
  int32_T p;
  int32_T q;
  int32_T qEnd;
  int32_T kEnd;
  int32_T i_0;
  for (pEnd = 0; pEnd < 2048; pEnd++) {
    idx[pEnd] = pEnd + 1;
  }

  for (pEnd = 0; pEnd <= 2047; pEnd += 2) {
    if (!(x[pEnd] <= x[pEnd + 1])) {
      idx[pEnd] = pEnd + 2;
      idx[pEnd + 1] = pEnd + 1;
    }
  }

  for (i_0 = 0; i_0 < 2048; i_0++) {
    idx_0[i_0] = 1;
  }

  i_0 = 2;
  while (i_0 < 2048) {
    i = i_0 << 1;
    j = 1;
    pEnd = 1 + i_0;
    while (pEnd < 2049) {
      p = j;
      q = pEnd;
      qEnd = j + i;
      k = 0;
      kEnd = qEnd - j;
      while (k + 1 <= kEnd) {
        if (x[idx[p - 1] - 1] <= x[idx[q - 1] - 1]) {
          idx_0[k] = (int16_T)idx[p - 1];
          p++;
          if (p == pEnd) {
            while (q < qEnd) {
              k++;
              idx_0[k] = (int16_T)idx[q - 1];
              q++;
            }
          }
        } else {
          idx_0[k] = (int16_T)idx[q - 1];
          q++;
          if (q == qEnd) {
            while (p < pEnd) {
              k++;
              idx_0[k] = (int16_T)idx[p - 1];
              p++;
            }
          }
        }

        k++;
      }

      for (pEnd = 0; pEnd + 1 <= kEnd; pEnd++) {
        idx[(j + pEnd) - 1] = idx_0[pEnd];
      }

      j = qEnd;
      pEnd = qEnd + i_0;
    }

    i_0 = i;
  }

  for (pEnd = 0; pEnd < 2048; pEnd++) {
    y[pEnd] = x[idx[pEnd] - 1];
  }
}

/* Function for MATLAB Function: '<S22>/Trilateration core' */
static void LocalizationCore_eml_sort_h(const real32_T x[100], real32_T y[100],
  int32_T idx[100])
{
  int32_T k;
  int8_T idx_0[100];
  int32_T i;
  int32_T i_0;
  int32_T j;
  int32_T pEnd;
  int32_T p;
  int32_T q;
  int32_T qEnd;
  int32_T kEnd;
  for (pEnd = 0; pEnd < 100; pEnd++) {
    idx[pEnd] = pEnd + 1;
  }

  for (pEnd = 0; pEnd <= 99; pEnd += 2) {
    if (!(x[pEnd] >= x[pEnd + 1])) {
      idx[pEnd] = pEnd + 2;
      idx[pEnd + 1] = pEnd + 1;
    }
  }

  memset(&idx_0[0], 1, 100U * sizeof(int8_T));
  i = 2;
  while (i < 100) {
    i_0 = i << 1;
    j = 1;
    pEnd = 1 + i;
    while (pEnd < 101) {
      p = j;
      q = pEnd;
      qEnd = j + i_0;
      if (qEnd > 101) {
        qEnd = 101;
      }

      k = 0;
      kEnd = qEnd - j;
      while (k + 1 <= kEnd) {
        if (x[idx[p - 1] - 1] >= x[idx[q - 1] - 1]) {
          idx_0[k] = (int8_T)idx[p - 1];
          p++;
          if (p == pEnd) {
            while (q < qEnd) {
              k++;
              idx_0[k] = (int8_T)idx[q - 1];
              q++;
            }
          }
        } else {
          idx_0[k] = (int8_T)idx[q - 1];
          q++;
          if (q == qEnd) {
            while (p < pEnd) {
              k++;
              idx_0[k] = (int8_T)idx[p - 1];
              p++;
            }
          }
        }

        k++;
      }

      for (pEnd = 0; pEnd + 1 <= kEnd; pEnd++) {
        idx[(j + pEnd) - 1] = idx_0[pEnd];
      }

      j = qEnd;
      pEnd = qEnd + i;
    }

    i = i_0;
  }

  for (pEnd = 0; pEnd < 100; pEnd++) {
    y[pEnd] = x[idx[pEnd] - 1];
  }
}

/* Function for MATLAB Function: '<S22>/Trilateration core' */
static void LocalizationCore_eml_sort(const real32_T x[100], real32_T y[100],
  int32_T idx[100])
{
  LocalizationCore_eml_sort_h(x, y, idx);
}

/* Function for MATLAB Function: '<S22>/Trilateration core' */
static real32_T LocalizationCore_norm(const real32_T x[3])
{
  real32_T y;
  real32_T scale;
  real32_T absxk;
  real32_T t;
  scale = 1.17549435E-38F;
  absxk = fabsf(x[0]);
  if (absxk > 1.17549435E-38F) {
    y = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.17549435E-38F;
    y = t * t;
  }

  absxk = fabsf(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = fabsf(x[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  return scale * sqrtf(y);
}

/* Function for MATLAB Function: '<S11>/MAC Filter and Merge' */
static boolean_T LocalizationCore_isequaln_g(const uint8_T varargin_1[17], const
  uint8_T varargin_2[17])
{
  boolean_T p;
  boolean_T b_p;
  int32_T k;
  boolean_T exitg;
  p = FALSE;
  b_p = TRUE;
  k = 0;
  exitg = FALSE;
  while ((!exitg) && (k < 17)) {
    if (!(varargin_1[k] == varargin_2[k])) {
      b_p = FALSE;
      exitg = TRUE;
    } else {
      k++;
    }
  }

  if (b_p) {
    p = TRUE;
  }

  return p;
}

/* Function for MATLAB Function: '<S11>/MAC Filter and Merge' */
static boolean_T LocalizationCore_isequaln(const uint8_T varargin_1[8], const
  uint8_T varargin_2[8])
{
  boolean_T p;
  boolean_T b_p;
  int32_T k;
  boolean_T exitg;
  p = FALSE;
  b_p = TRUE;
  k = 0;
  exitg = FALSE;
  while ((!exitg) && (k < 8)) {
    if (!(varargin_1[k] == varargin_2[k])) {
      b_p = FALSE;
      exitg = TRUE;
    } else {
      k++;
    }
  }

  if (b_p) {
    p = TRUE;
  }

  return p;
}

/* Function for MATLAB Function: '<S50>/kNN Localization core' */
static boolean_T LocalizationCore_isequaln_k(const uint8_T varargin_1[17], const
  uint8_T varargin_2[17])
{
  boolean_T p;
  boolean_T b_p;
  int32_T k;
  boolean_T exitg;
  p = FALSE;
  b_p = TRUE;
  k = 0;
  exitg = FALSE;
  while ((!exitg) && (k < 17)) {
    if (!(varargin_1[k] == varargin_2[k])) {
      b_p = FALSE;
      exitg = TRUE;
    } else {
      k++;
    }
  }

  if (b_p) {
    p = TRUE;
  }

  return p;
}

/* Function for MATLAB Function: '<S46>/CS Localization core' */
static boolean_T LocalizationCore_isequaln_gd(const uint8_T varargin_1[17],
  const uint8_T varargin_2[17])
{
  boolean_T p;
  boolean_T b_p;
  int32_T k;
  boolean_T exitg;
  p = FALSE;
  b_p = TRUE;
  k = 0;
  exitg = FALSE;
  while ((!exitg) && (k < 17)) {
    if (!(varargin_1[k] == varargin_2[k])) {
      b_p = FALSE;
      exitg = TRUE;
    } else {
      k++;
    }
  }

  if (b_p) {
    p = TRUE;
  }

  return p;
}

/* Function for MATLAB Function: '<S12>/MAC Filter and Merge' */
static boolean_T LocalizationCore_isequaln_ig(const uint8_T varargin_1[17],
  const uint8_T varargin_2[17])
{
  boolean_T p;
  boolean_T b_p;
  int32_T k;
  boolean_T exitg;
  p = FALSE;
  b_p = TRUE;
  k = 0;
  exitg = FALSE;
  while ((!exitg) && (k < 17)) {
    if (!(varargin_1[k] == varargin_2[k])) {
      b_p = FALSE;
      exitg = TRUE;
    } else {
      k++;
    }
  }

  if (b_p) {
    p = TRUE;
  }

  return p;
}

/* Function for MATLAB Function: '<S12>/MAC Filter and Merge' */
static boolean_T LocalizationCore_isequaln_i(const uint8_T varargin_1[8], const
  uint8_T varargin_2[8])
{
  boolean_T p;
  boolean_T b_p;
  int32_T k;
  boolean_T exitg;
  p = FALSE;
  b_p = TRUE;
  k = 0;
  exitg = FALSE;
  while ((!exitg) && (k < 8)) {
    if (!(varargin_1[k] == varargin_2[k])) {
      b_p = FALSE;
      exitg = TRUE;
    } else {
      k++;
    }
  }

  if (b_p) {
    p = TRUE;
  }

  return p;
}

/* Model step function */
void LocalizationCore_step(void)
{
  /* local block i/o variables */
  real32_T rtb_MultiportSelector13_o1[3];
  real32_T rtb_cov_out[36];
  real32_T rtb_MultiportSelector13_o1_n[3];
  real32_T rtb_cov_out_e[36];
  int32_T rtb_neighbors[6];
  int32_T rtb_neighbors_e[9];
  uint8_T availableAP[3400];
  uint8_T restricted_MAC[80];
  real32_T AP_visited[200];
  real_T new_counter;
  real_T n_ap_new;
  static const uint8_T b[8] = { 48U, 48U, 58U, 48U, 48U, 58U, 48U, 48U };

  real32_T temp[2];
  real32_T AP_visited_0[100];
  real32_T rss_new[1000];
  real_T search_flag;
  static const uint8_T b_0[8] = { 48U, 48U, 58U, 48U, 48U, 58U, 48U, 48U };

  real32_T X_k_k[6];
  real32_T P_k_k[36];
  real32_T K[18];
  real32_T K_0[12];
  int8_T I[36];
  real32_T B[12];
  real32_T b_A[9];
  real32_T b_B[18];
  int32_T r3;
  static const real32_T Q[36] = { 1.99999981E-8F, 0.0F, 0.0F, 2.0E-6F, 0.0F,
    0.0F, 0.0F, 1.99999981E-8F, 0.0F, 0.0F, 2.0E-6F, 0.0F, 0.0F, 0.0F,
    1.99999981E-8F, 0.0F, 0.0F, 2.0E-6F, 2.0E-6F, 0.0F, 0.0F, 0.0002F, 0.0F,
    0.0F, 0.0F, 2.0E-6F, 0.0F, 0.0F, 0.0002F, 0.0F, 0.0F, 0.0F, 2.0E-6F, 0.0F,
    0.0F, 0.0002F };

  static const real32_T a[36] = { 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.02F, 0.0F,
    0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.02F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F,
    0.02F, 0.0F, 0.0F, 1.0F };

  static const real32_T b_1[36] = { 1.0F, 0.0F, 0.0F, 0.02F, 0.0F, 0.0F, 0.0F,
    1.0F, 0.0F, 0.0F, 0.02F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.02F, 0.0F,
    0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 1.0F };

  static const int8_T b_a[12] = { 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0 };

  static const int8_T b_b[12] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 };

  static const int8_T d[4] = { 1, 0, 0, 1 };

  static const int8_T d_b[18] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0,
    0, 0 };

  static const int8_T e[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  static const int8_T c_a[18] = { 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 0 };

  static const real32_T Q_0[36] = { 1.99999981E-8F, 0.0F, 0.0F, 2.0E-6F, 0.0F,
    0.0F, 0.0F, 1.99999981E-8F, 0.0F, 0.0F, 2.0E-6F, 0.0F, 0.0F, 0.0F,
    1.99999981E-8F, 0.0F, 0.0F, 2.0E-6F, 2.0E-6F, 0.0F, 0.0F, 0.0002F, 0.0F,
    0.0F, 0.0F, 2.0E-6F, 0.0F, 0.0F, 0.0002F, 0.0F, 0.0F, 0.0F, 2.0E-6F, 0.0F,
    0.0F, 0.0002F };

  static const real32_T a_0[36] = { 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.02F,
    0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.02F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F,
    0.0F, 0.02F, 0.0F, 0.0F, 1.0F };

  static const real32_T b_2[36] = { 1.0F, 0.0F, 0.0F, 0.02F, 0.0F, 0.0F, 0.0F,
    1.0F, 0.0F, 0.0F, 0.02F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.02F, 0.0F,
    0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 1.0F };

  static const int8_T b_a_0[12] = { 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0 };

  static const int8_T b_b_0[12] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0 };

  static const int8_T d_0[4] = { 1, 0, 0, 1 };

  static const int8_T d_b_0[18] = { 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1,
    0, 0, 0 };

  static const int8_T e_0[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  static const int8_T c_a_0[18] = { 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    0, 0, 0 };

  real32_T x0_n[3];
  real_T idx[3];
  real32_T b_b_1[150];
  int32_T br;
  int32_T ar;
  int32_T ia;
  int32_T h;
  int32_T b_ic;
  int32_T b_ia;
  uint8_T indices[2048];
  real_T wifi_rss[200];
  real32_T n_beacons;
  real32_T num[3];
  real32_T A2x[4];
  real32_T b2x[2];
  real32_T y[4];
  real32_T c_y[100];
  int32_T iidx[100];
  real32_T rtb_MultiportSwitch1;
  int32_T rtb_DataTypeConversion2_h;
  real32_T rtb_DataTypeConversion1;
  real32_T rtb_MultiportSelector1_o1;
  int32_T rtb_DataTypeConversion1_h;
  real32_T rtb_Gain;
  real32_T rtb_Gain_e;
  real32_T rtb_DataTypeConversion;
  real32_T rtb_MultiportSelector1_o3;
  real32_T rtb_MultiportSelector1_o4;
  real32_T rtb_states_out_l[6];
  real32_T rtb_MathFunction_g[9];
  int32_T rtb_MultiportSwitch[3];
  uint8_T rtb_y_j[3400];
  uint8_T rtb_y_a[1700];
  real32_T rtb_y[300];
  uint8_T AP_list_j[1700];
  int32_T i;
  real32_T tmp[3];
  real32_T idx_0[3];
  real32_T tmp_0[150];
  uint8_T rtb_y_a_0[8];
  uint8_T restricted_MAC_0[8];
  uint8_T AP_list_j_0[17];
  uint8_T rtb_y_a_1[17];
  real32_T tmp_1[3];
  real32_T test_means[50];
  real32_T a_1[36];
  real32_T b_a_1[12];
  real32_T c_a_1[18];
  real32_T Q_data[9];
  real32_T T_data[150];
  real32_T z_data[3];
  real32_T a_data[9];
  int32_T a_sizes[2];
  real_T Q_data_0[9];
  int32_T Q_sizes[2];
  real_T z_data_0[3];
  int16_T tmp_data[300];
  real32_T tmp_data_0[300];
  int32_T f_idx;
  int32_T Q_sizes_idx;
  int8_T f_idx_0;
  real32_T tmp_2;

  /* S-Function (sdspmultiportsel): '<S2>/Multiport Selector1' incorporates:
   *  Inport: '<Root>/Params'
   */
  rtb_MultiportSelector1_o3 =
    LocalizationCore_U.Params[LocalizationCore_ConstP.MultiportSelector1_IN[2]];
  rtb_MultiportSelector1_o4 =
    LocalizationCore_U.Params[LocalizationCore_ConstP.MultiportSelector1_IN[3]];

  /* DataTypeConversion: '<S2>/Data Type Conversion2' incorporates:
   *  Inport: '<Root>/Params'
   *  S-Function (sdspmultiportsel): '<S2>/Multiport Selector1'
   */
  tmp_2 = fmodf(floorf
                (LocalizationCore_U.Params[LocalizationCore_ConstP.MultiportSelector1_IN
                 [1]]), 4.2949673E+9F);
  rtb_DataTypeConversion2_h = tmp_2 < 0.0F ? -(int32_T)(uint32_T)-tmp_2 :
    (int32_T)(uint32_T)tmp_2;

  /* MATLAB Function: '<S1>/MATLAB Function2' incorporates:
   *  Inport: '<Root>/Wifi_MAC'
   */
  /* MATLAB Function 'LocalizationCore/MATLAB Function2': '<S4>:1' */
  /* '<S4>:1:3' */
  memset(&rtb_y_j[0], 0, 3400U * sizeof(uint8_T));

  /* '<S4>:1:4' */
  for (ia = 0; ia < 200; ia++) {
    /* '<S4>:1:4' */
    ar = ia * 17 + 1;
    br = (1 + ia) * 17;
    if (ar > br) {
      ar = 1;
      br = 0;
    }

    /* '<S4>:1:5' */
    b_ia = ar - 1;
    i = br - b_ia;
    for (h = 0; h < i; h++) {
      availableAP[h] = LocalizationCore_U.Wifi_MAC[b_ia + h];
    }

    for (b_ia = 0; b_ia < 17; b_ia++) {
      rtb_y_j[ia + 200 * b_ia] = availableAP[b_ia];
    }

    /* '<S4>:1:4' */
  }

  /* End of MATLAB Function: '<S1>/MATLAB Function2' */

  /* DataTypeConversion: '<S1>/Data Type Conversion1' incorporates:
   *  Inport: '<Root>/Wifi_Status'
   */
  rtb_DataTypeConversion1 = (real32_T)LocalizationCore_U.Wifi_Status;

  /* Outputs for Enabled SubSystem: '<S2>/MAC Merge and Filter WiFi' incorporates:
   *  EnablePort: '<S12>/Enable'
   */
  if ((real32_T)LocalizationCore_U.Wifi_Status > 0.0F) {
    /* MATLAB Function: '<S12>/MAC Filter and Merge' incorporates:
     *  Inport: '<Root>/Wifi_RSS'
     */
    memcpy(&availableAP[0], &rtb_y_j[0], 3400U * sizeof(uint8_T));

    /* MATLAB Function 'LocalizationCore/Core/MAC Merge and Filter WiFi/MAC Filter and Merge': '<S30>:1' */
    /* '<S30>:1:5' */
    /* '<S30>:1:4' */
    memset(&restricted_MAC[0], 0, 80U * sizeof(uint8_T));

    /* '<S30>:1:5' */
    for (b_ia = 0; b_ia < 8; b_ia++) {
      restricted_MAC[10 * b_ia] = b[b_ia];
    }

    /* restricted_MAC(2,:)=uint8('00:0c:e6'); */
    /*  MAC filtering */
    /* '<S30>:1:12' */
    memset(&availableAP[3200], 48, 200U * sizeof(uint8_T));

    /* '<S30>:1:12' */
    /* '<S30>:1:16' */
    memset(&LocalizationCore_B.AP_list[0], 0, 3400U * sizeof(uint8_T));

    /* '<S30>:1:17' */
    memset(&AP_visited[0], (int32_T)0.0F, 200U * sizeof(real32_T));

    /* '<S30>:1:18' */
    memset(&LocalizationCore_B.rss_new[0], (int32_T)0.0F, 2000U * sizeof
           (real32_T));

    /* '<S30>:1:20' */
    new_counter = 1.0;

    /* '<S30>:1:23' */
    n_ap_new = 0.0;

    /* '<S30>:1:24' */
    for (ar = 0; ar < 200; ar++) {
      /* '<S30>:1:24' */
      if (LocalizationCore_U.Wifi_RSS[ar] != -200.0F) {
        /* '<S30>:1:25' */
        /* '<S30>:1:26' */
        n_ap_new++;
      }

      /* '<S30>:1:24' */
    }

    /* '<S30>:1:35' */
    for (ar = 0; ar < (int32_T)n_ap_new; ar++) {
      /* '<S30>:1:35' */
      /* '<S30>:1:36' */
      br = 0;

      /* '<S30>:1:37' */
      for (r3 = 0; r3 < 10; r3++) {
        /* '<S30>:1:37' */
        b_ia = (int32_T)(1.0 + (real_T)ar);
        for (h = 0; h < 8; h++) {
          rtb_y_a_0[h] = availableAP[(200 * h + b_ia) - 1];
          restricted_MAC_0[h] = restricted_MAC[10 * h + r3];
        }

        if (LocalizationCore_isequaln_i(rtb_y_a_0, restricted_MAC_0)) {
          /* '<S30>:1:39' */
          br = 1;
        }

        /* '<S30>:1:37' */
      }

      if (br == 0) {
        /* '<S30>:1:43' */
        /* '<S30>:1:44' */
        search_flag = 0.0;

        /* '<S30>:1:45' */
        while (br <= (int32_T)(new_counter - 1.0) - 1) {
          /* '<S30>:1:45' */
          b_ia = (int32_T)(1.0 + (real_T)br);
          h = (int32_T)(1.0 + (real_T)ar);
          for (i = 0; i < 17; i++) {
            rtb_y_a_1[i] = LocalizationCore_B.AP_list[(200 * i + b_ia) - 1];
            AP_list_j_0[i] = availableAP[(200 * i + h) - 1];
          }

          if (LocalizationCore_isequaln_ig(rtb_y_a_1, AP_list_j_0)) {
            /* '<S30>:1:47' */
            search_flag = 1.0 + (real_T)br;
          }

          /* '<S30>:1:45' */
          br++;
        }

        if (search_flag == 0.0) {
          /* '<S30>:1:50' */
          /* '<S30>:1:51' */
          i = (int32_T)new_counter;
          b_ia = (int32_T)(1.0 + (real_T)ar);
          for (h = 0; h < 17; h++) {
            LocalizationCore_B.AP_list[(i + 200 * h) - 1] = availableAP[(200 * h
              + b_ia) - 1];
          }

          /* '<S30>:1:52' */
          AP_visited[(int32_T)new_counter - 1] = 1.0F;

          /* '<S30>:1:53' */
          LocalizationCore_B.rss_new[10 * ((int32_T)new_counter - 1)] =
            LocalizationCore_U.Wifi_RSS[(int32_T)(1.0 + (real_T)ar) - 1];

          /* '<S30>:1:54' */
          new_counter++;
        } else {
          /* '<S30>:1:56' */
          AP_visited[(int32_T)search_flag - 1]++;

          /* '<S30>:1:57' */
          LocalizationCore_B.rss_new[((int32_T)AP_visited[(int32_T)search_flag -
            1] + 10 * ((int32_T)search_flag - 1)) - 1] =
            LocalizationCore_U.Wifi_RSS[(int32_T)(1.0 + (real_T)ar) - 1];
        }
      }

      /* '<S30>:1:35' */
    }

    /* % Callculating means */
    /* '<S30>:1:65' */
    memset(&LocalizationCore_B.mean_rss[0], (int32_T)0.0F, 200U * sizeof
           (real32_T));

    /* '<S30>:1:67' */
    for (br = 0; br < 200; br++) {
      /* '<S30>:1:67' */
      /* '<S30>:1:68' */
      new_counter = 0.0;

      /* '<S30>:1:69' */
      for (ar = 0; ar < 10; ar++) {
        /* '<S30>:1:69' */
        if ((LocalizationCore_B.rss_new[10 * br + ar] != 0.0F) &&
            (LocalizationCore_B.rss_new[10 * br + ar] != -110.0F)) {
          /* '<S30>:1:70' */
          /* '<S30>:1:71' */
          LocalizationCore_B.mean_rss[br] += LocalizationCore_B.rss_new[10 * br
            + ar];

          /* '<S30>:1:72' */
          new_counter++;
        }

        /* '<S30>:1:69' */
      }

      if (new_counter == 0.0) {
        /* '<S30>:1:75' */
        /* '<S30>:1:76' */
        LocalizationCore_B.mean_rss[br] = -110.0F;
      } else {
        /* '<S30>:1:78' */
        LocalizationCore_B.mean_rss[br] /= (real32_T)new_counter;
      }

      /* '<S30>:1:67' */
    }

    /* End of MATLAB Function: '<S12>/MAC Filter and Merge' */
  }

  /* End of Outputs for SubSystem: '<S2>/MAC Merge and Filter WiFi' */

  /* DataTypeConversion: '<S2>/Data Type Conversion1' incorporates:
   *  Inport: '<Root>/Params'
   *  S-Function (sdspmultiportsel): '<S2>/Multiport Selector1'
   */
  tmp_2 = fmodf(floorf
                (LocalizationCore_U.Params[LocalizationCore_ConstP.MultiportSelector1_IN
                 [0]]), 4.2949673E+9F);
  rtb_DataTypeConversion1_h = tmp_2 < 0.0F ? -(int32_T)(uint32_T)-tmp_2 :
    (int32_T)(uint32_T)tmp_2;

  /* Outputs for Enabled SubSystem: '<S2>/Wifi Localizer' incorporates:
   *  EnablePort: '<S16>/Enable'
   */
  /* DataTypeConversion: '<S1>/Data Type Conversion1' incorporates:
   *  Constant: '<S2>/Constant'
   *  Constant: '<S47>/Constant'
   *  Constant: '<S48>/Constant'
   *  Constant: '<S49>/AP_LIST'
   *  Constant: '<S49>/MEAN_RSS'
   *  Constant: '<S49>/N_AP'
   *  Constant: '<S49>/N_POINTS'
   *  Constant: '<S49>/POINT_LIST'
   *  Inport: '<Root>/Wifi_Status'
   *  RelationalOperator: '<S47>/Compare'
   *  RelationalOperator: '<S48>/Compare'
   */
  if ((real32_T)LocalizationCore_U.Wifi_Status > 0.0F) {
    /* Outputs for Enabled SubSystem: '<S16>/RadioMap' incorporates:
     *  EnablePort: '<S49>/Enable'
     */
    if (LocalizationCore_P.Constant_Value > 0.0) {
      memcpy(&LocalizationCore_B.AP_LIST[0], &LocalizationCore_P.AP_LIST_Value[0],
             34816U * sizeof(uint8_T));
      memcpy(&LocalizationCore_B.MEAN_RSS[0],
             &LocalizationCore_P.MEAN_RSS_Value[0], sizeof(real32_T) << 22U);
      LocalizationCore_B.N_AP = LocalizationCore_P.N_AP_Value;
      LocalizationCore_B.N_POINTS = LocalizationCore_P.N_POINTS_Value;
      memcpy(&LocalizationCore_B.POINT_LIST[0],
             &LocalizationCore_P.POINT_LIST_Value[0], 6144U * sizeof(real32_T));
    }

    /* End of Outputs for SubSystem: '<S16>/RadioMap' */

    /* Outputs for Enabled SubSystem: '<S16>/CS Wifi Localizer' incorporates:
     *  EnablePort: '<S46>/Enable'
     */
    if (rtb_DataTypeConversion1_h == LocalizationCore_P.Constant_Value_f) {
      /* MATLAB Function: '<S46>/CS Localization core' */
      /* MATLAB Function 'LocalizationCore/Core/Wifi Localizer/CS Wifi Localizer/CS Localization core': '<S51>:1' */
      /*  Match Mac-list for online readings */
      /* [len1,~]=size(AP_list); */
      /* '<S51>:1:9' */
      /* '<S51>:1:10' */
      for (b_ia = 0; b_ia < 2048; b_ia++) {
        LocalizationCore_B.wifi_int[b_ia] = 0.0;
        LocalizationCore_B.indices[b_ia] = 0.0;
      }

      /* '<S51>:1:14' */
      for (ia = 0; ia < (int32_T)LocalizationCore_B.N_AP; ia++) {
        /* '<S51>:1:14' */
        /* '<S51>:1:15' */
        for (br = 0; br < 200; br++) {
          /* '<S51>:1:15' */
          b_ia = (int32_T)(1.0F + (real32_T)ia);
          for (h = 0; h < 17; h++) {
            rtb_y_a_1[h] = LocalizationCore_B.AP_LIST[((h << 11) + b_ia) - 1];
            AP_list_j_0[h] = LocalizationCore_B.AP_list[200 * h + br];
          }

          if (LocalizationCore_isequaln_gd(rtb_y_a_1, AP_list_j_0)) {
            /* '<S51>:1:17' */
            LocalizationCore_B.indices[(int32_T)(1.0F + (real32_T)ia) - 1] = 1.0
              + (real_T)br;
          }

          /* '<S51>:1:15' */
        }

        /* '<S51>:1:14' */
      }

      /* '<S51>:1:21' */
      for (ar = 0; ar < (int32_T)LocalizationCore_B.N_AP; ar++) {
        /* '<S51>:1:21' */
        if (LocalizationCore_B.indices[(int32_T)(1.0F + (real32_T)ar) - 1] !=
            0.0) {
          /* '<S51>:1:22' */
          /* '<S51>:1:23' */
          LocalizationCore_B.wifi_int[(int32_T)(1.0F + (real32_T)ar) - 1] =
            LocalizationCore_B.mean_rss[(int32_T)LocalizationCore_B.indices
            [(int32_T)(1.0F + (real32_T)ar) - 1] - 1];
        } else {
          /* '<S51>:1:25' */
          LocalizationCore_B.wifi_int[(int32_T)(1.0F + (real32_T)ar) - 1] =
            -110.0;
        }

        /* '<S51>:1:21' */
      }

      /* kNN with weighted average */
      /* [n_ap,n_points]=size(mean_rss); */
      /* '<S51>:1:42' */
      memcpy(&LocalizationCore_B.test_means[0], &LocalizationCore_B.wifi_int[0],
             sizeof(real_T) << 11U);
      if (LocalizationCore_B.N_AP + 1.0F > 2048.0F) {
        ar = 0;
        br = 0;
      } else {
        ar = (int32_T)(LocalizationCore_B.N_AP + 1.0F) - 1;
        br = 2048;
      }

      /* '<S51>:1:43' */
      i = br - ar;
      for (b_ia = 0; b_ia < i; b_ia++) {
        LocalizationCore_B.test_means[ar + b_ia] = -3.4028234663852886E+38;
      }

      /* '<S51>:1:46' */
      memset(&LocalizationCore_B.wifi_int[0], 0, sizeof(real_T) << 11U);
      if (LocalizationCore_B.N_POINTS + 1.0F > 2048.0F) {
        ar = 0;
        br = 0;
      } else {
        ar = (int32_T)(LocalizationCore_B.N_POINTS + 1.0F) - 1;
        br = 2048;
      }

      /* '<S51>:1:47' */
      i = br - ar;
      for (b_ia = 0; b_ia < i; b_ia++) {
        LocalizationCore_B.wifi_int[ar + b_ia] = 3.4028234663852886E+38;
      }

      /* '<S51>:1:48' */
      /* '<S51>:1:52' */
      for (br = 0; br < (int32_T)LocalizationCore_B.N_POINTS; br++) {
        /* '<S51>:1:52' */
        /* '<S51>:1:53' */
        for (r3 = 0; r3 < (int32_T)LocalizationCore_B.N_AP; r3++) {
          /* '<S51>:1:53' */
          if (!((LocalizationCore_B.MEAN_RSS[((((int32_T)(1.0F + (real32_T)br) -
                   1) << 11) + (int32_T)(1.0F + (real32_T)r3)) - 1] == -110.0F) &&
                (LocalizationCore_B.test_means[(int32_T)(1.0F + (real32_T)r3) -
                 1] == -110.0))) {
            if ((LocalizationCore_B.MEAN_RSS[((((int32_T)(1.0F + (real32_T)br) -
                    1) << 11) + (int32_T)(1.0F + (real32_T)r3)) - 1] != -110.0F)
                && (LocalizationCore_B.test_means[(int32_T)(1.0F + (real32_T)r3)
                    - 1] != -110.0)) {
              /* '<S51>:1:56' */
              /* '<S51>:1:57' */
              LocalizationCore_B.wifi_int[(int32_T)(1.0F + (real32_T)br) - 1] =
                fabsf(LocalizationCore_B.MEAN_RSS[((((int32_T)(1.0F + (real32_T)
                          br) - 1) << 11) + (int32_T)(1.0F + (real32_T)r3)) - 1]
                      - (real32_T)LocalizationCore_B.test_means[(int32_T)(1.0F +
                       (real32_T)r3) - 1]) + (real32_T)
                LocalizationCore_B.wifi_int[(int32_T)(1.0F + (real32_T)br) - 1];
            } else {
              if ((LocalizationCore_B.MEAN_RSS[((((int32_T)(1.0F + (real32_T)br)
                      - 1) << 11) + (int32_T)(1.0F + (real32_T)r3)) - 1] ==
                   -110.0F) && (LocalizationCore_B.test_means[(int32_T)(1.0F +
                    (real32_T)r3) - 1] != -110.0)) {
                /* '<S51>:1:59' */
                /* '<S51>:1:60' */
                LocalizationCore_B.wifi_int[(int32_T)(1.0F + (real32_T)br) - 1] =
                  fabsf(LocalizationCore_B.MEAN_RSS[((((int32_T)(1.0F +
                            (real32_T)br) - 1) << 11) + (int32_T)(1.0F +
                          (real32_T)r3)) - 1] - (real32_T)
                        LocalizationCore_B.test_means[(int32_T)(1.0F + (real32_T)
                         r3) - 1]) + (real32_T)LocalizationCore_B.wifi_int
                  [(int32_T)(1.0F + (real32_T)br) - 1];
              }
            }
          } else {
            /* '<S51>:1:54' */
          }

          /* '<S51>:1:53' */
        }

        /* '<S51>:1:52' */
      }

      memcpy(&LocalizationCore_B.indices[0], &LocalizationCore_B.wifi_int[0],
             sizeof(real_T) << 11U);
      LocalizationCore_eml_sort_l(LocalizationCore_B.indices,
        LocalizationCore_B.wifi_int, LocalizationCore_B.iidx);

      /* '<S51>:1:68' */
      /* '<S51>:1:71' */
      LocalizationCore_eml_sort_lv(LocalizationCore_B.test_means,
        LocalizationCore_B.indices, LocalizationCore_B.b_iidx);
      for (b_ia = 0; b_ia < 2048; b_ia++) {
        LocalizationCore_B.wifi_int[b_ia] = LocalizationCore_B.iidx[b_ia];
        LocalizationCore_B.indices[b_ia] = LocalizationCore_B.b_iidx[b_ia];
      }

      /* '<S51>:1:73' */
      /* '<S51>:1:74' */
      /* '<S51>:1:75' */
      /* '<S51>:1:76' */
      for (b_ia = 0; b_ia < 50; b_ia++) {
        tmp_0[3 * b_ia] = LocalizationCore_B.MEAN_RSS[((((int32_T)
          LocalizationCore_B.wifi_int[0] - 1) << 11) + (int32_T)
          LocalizationCore_B.indices[b_ia]) - 1];
        tmp_0[1 + 3 * b_ia] = LocalizationCore_B.MEAN_RSS[((((int32_T)
          LocalizationCore_B.wifi_int[1] - 1) << 11) + (int32_T)
          LocalizationCore_B.indices[b_ia]) - 1];
        tmp_0[2 + 3 * b_ia] = LocalizationCore_B.MEAN_RSS[((((int32_T)
          LocalizationCore_B.wifi_int[2] - 1) << 11) + (int32_T)
          LocalizationCore_B.indices[b_ia]) - 1];
      }

      LocalizationCore_orth(tmp_0, a_data, a_sizes);
      Q_sizes_idx = a_sizes[1];
      i = a_sizes[1];
      for (b_ia = 0; b_ia < i; b_ia++) {
        Q_data[b_ia] = a_data[a_sizes[0] * b_ia];
      }

      i = a_sizes[1];
      for (b_ia = 0; b_ia < i; b_ia++) {
        Q_data[b_ia + a_sizes[1]] = a_data[a_sizes[0] * b_ia + 1];
      }

      i = a_sizes[1];
      for (b_ia = 0; b_ia < i; b_ia++) {
        Q_data[b_ia + (a_sizes[1] << 1)] = a_data[a_sizes[0] * b_ia + 2];
      }

      /* '<S51>:1:77' */
      for (b_ia = 0; b_ia < 3; b_ia++) {
        for (h = 0; h < 50; h++) {
          tmp_0[h + 50 * b_ia] = LocalizationCore_B.MEAN_RSS[((((int32_T)
            LocalizationCore_B.wifi_int[b_ia] - 1) << 11) + (int32_T)
            LocalizationCore_B.indices[h]) - 1];
        }
      }

      LocalizationCore_pinv(tmp_0, b_b_1);
      f_idx_0 = (int8_T)a_sizes[1];
      f_idx = f_idx_0;
      i = f_idx_0 * 50;
      for (b_ia = 0; b_ia < i; b_ia++) {
        T_data[b_ia] = 0.0F;
      }

      if (a_sizes[1] != 0) {
        i = a_sizes[1] * 49;
        ar = 0;
        while ((a_sizes[1] > 0) && (ar <= i)) {
          br = ar + a_sizes[1];
          for (b_ia = ar; b_ia + 1 <= br; b_ia++) {
            T_data[b_ia] = 0.0F;
          }

          ar += a_sizes[1];
        }

        br = 0;
        r3 = 0;
        while ((a_sizes[1] > 0) && (r3 <= i)) {
          ar = 0;
          for (b_ia = br; b_ia + 1 <= br + 3; b_ia++) {
            if (b_b_1[b_ia] != 0.0F) {
              ia = ar;
              h = r3 + a_sizes[1];
              for (b_ic = r3; b_ic + 1 <= h; b_ic++) {
                ia++;
                T_data[b_ic] += Q_data[ia - 1] * b_b_1[b_ia];
              }
            }

            ar += a_sizes[1];
          }

          br += 3;
          r3 += a_sizes[1];
        }
      }

      /* '<S51>:1:78' */
      for (b_ia = 0; b_ia < 50; b_ia++) {
        test_means[b_ia] = (real32_T)LocalizationCore_B.test_means[(int32_T)
          LocalizationCore_B.indices[b_ia] - 1];
      }

      for (b_ia = 0; b_ia < f_idx; b_ia++) {
        z_data[b_ia] = 0.0F;
        for (h = 0; h < 50; h++) {
          z_data[b_ia] += T_data[f_idx_0 * h + b_ia] * test_means[h];
        }
      }

      /* '<S51>:1:79' */
      for (b_ia = 0; b_ia < Q_sizes_idx; b_ia++) {
        a_data[3 * b_ia] = Q_data[b_ia];
        a_data[1 + 3 * b_ia] = Q_data[b_ia + a_sizes[1]];
        a_data[2 + 3 * b_ia] = Q_data[(a_sizes[1] << 1) + b_ia];
      }

      if ((a_sizes[1] == 1) || (f_idx_0 == 1)) {
        x0_n[0] = 0.0F;
        i = a_sizes[1];
        rtb_MultiportSelector1_o1 = 0.0F;
        for (b_ia = 0; b_ia < i; b_ia++) {
          rtb_MultiportSelector1_o1 += a_data[3 * b_ia] * z_data[b_ia];
          x0_n[0] = rtb_MultiportSelector1_o1;
        }

        x0_n[1] = 0.0F;
        i = a_sizes[1];
        rtb_MultiportSelector1_o1 = 0.0F;
        for (b_ia = 0; b_ia < i; b_ia++) {
          rtb_MultiportSelector1_o1 += a_data[3 * b_ia + 1] * z_data[b_ia];
          x0_n[1] = rtb_MultiportSelector1_o1;
        }

        x0_n[2] = 0.0F;
        i = a_sizes[1];
        rtb_MultiportSelector1_o1 = 0.0F;
        for (b_ia = 0; b_ia < i; b_ia++) {
          rtb_MultiportSelector1_o1 += a_data[3 * b_ia + 2] * z_data[b_ia];
          x0_n[2] = rtb_MultiportSelector1_o1;
        }
      } else {
        x0_n[0] = 0.0F;
        x0_n[1] = 0.0F;
        x0_n[2] = 0.0F;
        ar = 0;
        for (br = 0; br + 1 <= a_sizes[1]; br++) {
          if (z_data[br] != 0.0F) {
            b_ia = ar + 1;
            x0_n[0] += a_data[b_ia - 1] * z_data[br];
            b_ia++;
            x0_n[1] += a_data[b_ia - 1] * z_data[br];
            b_ia++;
            x0_n[2] += a_data[b_ia - 1] * z_data[br];
          }

          ar += 3;
        }
      }

      /* '<S51>:1:80' */
      idx[0] = x0_n[0];
      idx[1] = x0_n[1];
      idx[2] = x0_n[2];
      Q_sizes[0] = a_sizes[1];
      Q_sizes[1] = 3;
      i = a_sizes[1] * 3;
      for (b_ia = 0; b_ia < i; b_ia++) {
        Q_data_0[b_ia] = Q_data[b_ia];
      }

      for (b_ia = 0; b_ia < f_idx; b_ia++) {
        z_data_0[b_ia] = z_data[b_ia];
      }

      LocalizationCore_l1eq_pd(idx, Q_data_0, Q_sizes, z_data_0);
      idx_0[0] = (real32_T)idx[0];
      idx_0[1] = (real32_T)idx[1];
      idx_0[2] = (real32_T)idx[2];
      LocalizationCore_eml_sort_lvl1(idx_0, x0_n, rtb_MultiportSwitch);

      /* '<S51>:1:83' */
      /* '<S51>:1:84' */
      /* '<S51>:1:85' */
      /* '<S51>:1:86' */
      /* '<S51>:1:83' */
      /* '<S51>:1:84' */
      /* '<S51>:1:85' */
      /* '<S51>:1:86' */
      /* '<S51>:1:83' */
      /* '<S51>:1:84' */
      /* '<S51>:1:85' */
      /* '<S51>:1:86' */
      /* '<S51>:1:83' */
      /* '<S51>:1:88' */
      /* '<S51>:1:89' */
      /* '<S51>:1:90' */
      /* '<S51>:1:94' */
      /* '<S51>:1:95' */
      new_counter = rt_roundd((real_T)(((LocalizationCore_B.POINT_LIST[(int32_T)
        LocalizationCore_B.wifi_int[(int32_T)(real_T)rtb_MultiportSwitch[0] - 1]
        - 1] * x0_n[0] + LocalizationCore_B.POINT_LIST[(int32_T)
        LocalizationCore_B.wifi_int[(int32_T)(real_T)rtb_MultiportSwitch[1] - 1]
        - 1] * x0_n[1]) + LocalizationCore_B.POINT_LIST[(int32_T)
        LocalizationCore_B.wifi_int[(int32_T)(real_T)rtb_MultiportSwitch[2] - 1]
        - 1] * x0_n[2]) / LocalizationCore_sum(x0_n)));
      if (new_counter < 2.147483648E+9) {
        if (new_counter >= -2.147483648E+9) {
          LocalizationCore_B.position_out_i[0] = (int32_T)new_counter;
        } else {
          LocalizationCore_B.position_out_i[0] = MIN_int32_T;
        }
      } else {
        LocalizationCore_B.position_out_i[0] = MAX_int32_T;
      }

      new_counter = rt_roundd((real_T)(((LocalizationCore_B.POINT_LIST[(int32_T)
        LocalizationCore_B.wifi_int[(int32_T)(real_T)rtb_MultiportSwitch[0] - 1]
        + 2047] * x0_n[0] + LocalizationCore_B.POINT_LIST[(int32_T)
        LocalizationCore_B.wifi_int[(int32_T)(real_T)rtb_MultiportSwitch[1] - 1]
        + 2047] * x0_n[1]) + LocalizationCore_B.POINT_LIST[(int32_T)
        LocalizationCore_B.wifi_int[(int32_T)(real_T)rtb_MultiportSwitch[2] - 1]
        + 2047] * x0_n[2]) / LocalizationCore_sum(x0_n)));
      if (new_counter < 2.147483648E+9) {
        if (new_counter >= -2.147483648E+9) {
          LocalizationCore_B.position_out_i[1] = (int32_T)new_counter;
        } else {
          LocalizationCore_B.position_out_i[1] = MIN_int32_T;
        }
      } else {
        LocalizationCore_B.position_out_i[1] = MAX_int32_T;
      }

      new_counter = rt_roundd((real_T)(((LocalizationCore_B.POINT_LIST[(int32_T)
        LocalizationCore_B.wifi_int[(int32_T)(real_T)rtb_MultiportSwitch[0] - 1]
        + 4095] * x0_n[0] + LocalizationCore_B.POINT_LIST[(int32_T)
        LocalizationCore_B.wifi_int[(int32_T)(real_T)rtb_MultiportSwitch[1] - 1]
        + 4095] * x0_n[1]) + LocalizationCore_B.POINT_LIST[(int32_T)
        LocalizationCore_B.wifi_int[(int32_T)(real_T)rtb_MultiportSwitch[2] - 1]
        + 4095] * x0_n[2]) / LocalizationCore_sum(x0_n)));
      if (new_counter < 2.147483648E+9) {
        if (new_counter >= -2.147483648E+9) {
          LocalizationCore_B.position_out_i[2] = (int32_T)new_counter;
        } else {
          LocalizationCore_B.position_out_i[2] = MIN_int32_T;
        }
      } else {
        LocalizationCore_B.position_out_i[2] = MAX_int32_T;
      }

      for (b_ia = 0; b_ia < 3; b_ia++) {
        tmp_2 = roundf(LocalizationCore_B.POINT_LIST[((b_ia << 11) + (int32_T)
          LocalizationCore_B.wifi_int[0]) - 1]);
        if (tmp_2 < 2.14748365E+9F) {
          if (tmp_2 >= -2.14748365E+9F) {
            rtb_neighbors_e[3 * b_ia] = (int32_T)tmp_2;
          } else {
            rtb_neighbors_e[3 * b_ia] = MIN_int32_T;
          }
        } else {
          rtb_neighbors_e[3 * b_ia] = MAX_int32_T;
        }

        tmp_2 = roundf(LocalizationCore_B.POINT_LIST[((b_ia << 11) + (int32_T)
          LocalizationCore_B.wifi_int[1]) - 1]);
        if (tmp_2 < 2.14748365E+9F) {
          if (tmp_2 >= -2.14748365E+9F) {
            rtb_neighbors_e[1 + 3 * b_ia] = (int32_T)tmp_2;
          } else {
            rtb_neighbors_e[1 + 3 * b_ia] = MIN_int32_T;
          }
        } else {
          rtb_neighbors_e[1 + 3 * b_ia] = MAX_int32_T;
        }

        tmp_2 = roundf(LocalizationCore_B.POINT_LIST[((b_ia << 11) + (int32_T)
          LocalizationCore_B.wifi_int[2]) - 1]);
        if (tmp_2 < 2.14748365E+9F) {
          if (tmp_2 >= -2.14748365E+9F) {
            rtb_neighbors_e[2 + 3 * b_ia] = (int32_T)tmp_2;
          } else {
            rtb_neighbors_e[2 + 3 * b_ia] = MIN_int32_T;
          }
        } else {
          rtb_neighbors_e[2 + 3 * b_ia] = MAX_int32_T;
        }
      }

      LocalizationCore_B.accuracy_f = 5.0F;

      /* End of MATLAB Function: '<S46>/CS Localization core' */
    }

    /* End of Outputs for SubSystem: '<S16>/CS Wifi Localizer' */

    /* Outputs for Enabled SubSystem: '<S16>/kNN Localizer' incorporates:
     *  EnablePort: '<S50>/Enable'
     */
    if (rtb_DataTypeConversion1_h == LocalizationCore_P.Constant_Value_a) {
      /* MATLAB Function: '<S50>/kNN Localization core' */
      /* MATLAB Function 'LocalizationCore/Core/Wifi Localizer/kNN Localizer/kNN Localization core': '<S52>:1' */
      /* '<S52>:1:3' */
      for (b_ia = 0; b_ia < 200; b_ia++) {
        wifi_rss[b_ia] = LocalizationCore_B.mean_rss[b_ia];
      }

      /*  Match Mac-list for online readings */
      /* [len1,~]=size(AP_list); */
      /* '<S52>:1:9' */
      /* '<S52>:1:10' */
      for (b_ia = 0; b_ia < 2048; b_ia++) {
        LocalizationCore_B.wifi_int[b_ia] = 0.0;
        indices[b_ia] = 0U;
      }

      /* '<S52>:1:14' */
      for (ia = 0; ia < (int32_T)LocalizationCore_B.N_AP; ia++) {
        /* '<S52>:1:14' */
        /* '<S52>:1:15' */
        for (br = 0; br < 200; br++) {
          /* '<S52>:1:15' */
          b_ia = (int32_T)(1.0F + (real32_T)ia);
          for (h = 0; h < 17; h++) {
            rtb_y_a_1[h] = LocalizationCore_B.AP_LIST[((h << 11) + b_ia) - 1];
            AP_list_j_0[h] = LocalizationCore_B.AP_list[200 * h + br];
          }

          if (LocalizationCore_isequaln_k(rtb_y_a_1, AP_list_j_0)) {
            /* '<S52>:1:17' */
            indices[(int32_T)(1.0F + (real32_T)ia) - 1] = (uint8_T)(1 + br);
          }

          /* '<S52>:1:15' */
        }

        /* '<S52>:1:14' */
      }

      /* '<S52>:1:21' */
      for (ar = 0; ar < (int32_T)LocalizationCore_B.N_AP; ar++) {
        /* '<S52>:1:21' */
        if (indices[(int32_T)(1.0F + (real32_T)ar) - 1] != 0) {
          /* '<S52>:1:22' */
          /* '<S52>:1:23' */
          LocalizationCore_B.wifi_int[(int32_T)(1.0F + (real32_T)ar) - 1] =
            wifi_rss[indices[(int32_T)(1.0F + (real32_T)ar) - 1] - 1];
        } else {
          /* '<S52>:1:25' */
          LocalizationCore_B.wifi_int[(int32_T)(1.0F + (real32_T)ar) - 1] =
            -110.0;
        }

        /* '<S52>:1:21' */
      }

      /* kNN with weighted average */
      /* [n_ap,n_points]=size(mean_rss); */
      /* '<S52>:1:42' */
      /* '<S52>:1:45' */
      memset(&LocalizationCore_B.indices[0], 0, sizeof(real_T) << 11U);
      if (LocalizationCore_B.N_POINTS + 1.0F > 2048.0F) {
        ar = 0;
        br = 0;
      } else {
        ar = (int32_T)(LocalizationCore_B.N_POINTS + 1.0F) - 1;
        br = 2048;
      }

      /* '<S52>:1:46' */
      i = br - ar;
      for (b_ia = 0; b_ia < i; b_ia++) {
        LocalizationCore_B.indices[ar + b_ia] = 3.4028234663852886E+38;
      }

      /* '<S52>:1:47' */
      /* '<S52>:1:51' */
      for (br = 0; br < (int32_T)LocalizationCore_B.N_POINTS; br++) {
        /* '<S52>:1:51' */
        /* '<S52>:1:52' */
        for (r3 = 0; r3 < (int32_T)LocalizationCore_B.N_AP; r3++) {
          /* '<S52>:1:52' */
          if (!((LocalizationCore_B.MEAN_RSS[((((int32_T)(1.0F + (real32_T)br) -
                   1) << 11) + (int32_T)(1.0F + (real32_T)r3)) - 1] == -110.0F) &&
                (LocalizationCore_B.wifi_int[(int32_T)(1.0F + (real32_T)r3) - 1]
                 == -110.0))) {
            if ((LocalizationCore_B.MEAN_RSS[((((int32_T)(1.0F + (real32_T)br) -
                    1) << 11) + (int32_T)(1.0F + (real32_T)r3)) - 1] != -110.0F)
                && (LocalizationCore_B.wifi_int[(int32_T)(1.0F + (real32_T)r3) -
                    1] != -110.0)) {
              /* '<S52>:1:55' */
              /* '<S52>:1:56' */
              LocalizationCore_B.indices[(int32_T)(1.0F + (real32_T)br) - 1] =
                fabsf(LocalizationCore_B.MEAN_RSS[((((int32_T)(1.0F + (real32_T)
                          br) - 1) << 11) + (int32_T)(1.0F + (real32_T)r3)) - 1]
                      - (real32_T)LocalizationCore_B.wifi_int[(int32_T)(1.0F +
                       (real32_T)r3) - 1]) + (real32_T)
                LocalizationCore_B.indices[(int32_T)(1.0F + (real32_T)br) - 1];
            } else {
              if ((LocalizationCore_B.MEAN_RSS[((((int32_T)(1.0F + (real32_T)br)
                      - 1) << 11) + (int32_T)(1.0F + (real32_T)r3)) - 1] ==
                   -110.0F) && (LocalizationCore_B.wifi_int[(int32_T)(1.0F +
                    (real32_T)r3) - 1] != -110.0)) {
                /* '<S52>:1:58' */
                /* '<S52>:1:59' */
                LocalizationCore_B.indices[(int32_T)(1.0F + (real32_T)br) - 1] =
                  fabsf(LocalizationCore_B.MEAN_RSS[((((int32_T)(1.0F +
                            (real32_T)br) - 1) << 11) + (int32_T)(1.0F +
                          (real32_T)r3)) - 1] - (real32_T)
                        LocalizationCore_B.wifi_int[(int32_T)(1.0F + (real32_T)
                         r3) - 1]) + (real32_T)LocalizationCore_B.indices
                  [(int32_T)(1.0F + (real32_T)br) - 1];
              }
            }
          } else {
            /* '<S52>:1:53' */
          }

          /* '<S52>:1:52' */
        }

        /* '<S52>:1:51' */
      }

      LocalizationCore_eml_sort_d(LocalizationCore_B.indices,
        LocalizationCore_B.wifi_int, LocalizationCore_B.iidx);

      /* '<S52>:1:67' */
      /* '<S52>:1:68' */
      /* '<S52>:1:69' */
      for (b_ia = 0; b_ia < 2048; b_ia++) {
        LocalizationCore_B.indices[b_ia] = LocalizationCore_B.iidx[b_ia];
        LocalizationCore_B.test_means[b_ia] = 1.0 /
          LocalizationCore_B.wifi_int[b_ia];
      }

      /* '<S52>:1:70' */
      /* '<S52>:1:71' */
      /* '<S52>:1:72' */
      /* '<S52>:1:73' */
      /* '<S52>:1:70' */
      /* '<S52>:1:71' */
      /* '<S52>:1:72' */
      /* '<S52>:1:73' */
      /* '<S52>:1:70' */
      /* '<S52>:1:75' */
      /* '<S52>:1:76' */
      /* '<S52>:1:77' */
      /* '<S52>:1:81' */
      /* '<S52>:1:82' */
      new_counter = rt_roundd((LocalizationCore_B.POINT_LIST[(int32_T)
        LocalizationCore_B.indices[0] - 1] * (real32_T)
        LocalizationCore_B.test_means[0] + LocalizationCore_B.POINT_LIST
        [(int32_T)LocalizationCore_B.indices[1] - 1] * (real32_T)
        LocalizationCore_B.test_means[1]) / (LocalizationCore_B.test_means[0] +
        LocalizationCore_B.test_means[1]));
      if (new_counter < 2.147483648E+9) {
        if (new_counter >= -2.147483648E+9) {
          LocalizationCore_B.position_out[0] = (int32_T)new_counter;
        } else {
          LocalizationCore_B.position_out[0] = MIN_int32_T;
        }
      } else {
        LocalizationCore_B.position_out[0] = MAX_int32_T;
      }

      new_counter = rt_roundd((LocalizationCore_B.POINT_LIST[(int32_T)
        LocalizationCore_B.indices[0] + 2047] * (real32_T)
        LocalizationCore_B.test_means[0] + LocalizationCore_B.POINT_LIST
        [(int32_T)LocalizationCore_B.indices[1] + 2047] * (real32_T)
        LocalizationCore_B.test_means[1]) / (LocalizationCore_B.test_means[0] +
        LocalizationCore_B.test_means[1]));
      if (new_counter < 2.147483648E+9) {
        if (new_counter >= -2.147483648E+9) {
          LocalizationCore_B.position_out[1] = (int32_T)new_counter;
        } else {
          LocalizationCore_B.position_out[1] = MIN_int32_T;
        }
      } else {
        LocalizationCore_B.position_out[1] = MAX_int32_T;
      }

      new_counter = rt_roundd((LocalizationCore_B.POINT_LIST[(int32_T)
        LocalizationCore_B.indices[0] + 4095] * (real32_T)
        LocalizationCore_B.test_means[0] + LocalizationCore_B.POINT_LIST
        [(int32_T)LocalizationCore_B.indices[1] + 4095] * (real32_T)
        LocalizationCore_B.test_means[1]) / (LocalizationCore_B.test_means[0] +
        LocalizationCore_B.test_means[1]));
      if (new_counter < 2.147483648E+9) {
        if (new_counter >= -2.147483648E+9) {
          LocalizationCore_B.position_out[2] = (int32_T)new_counter;
        } else {
          LocalizationCore_B.position_out[2] = MIN_int32_T;
        }
      } else {
        LocalizationCore_B.position_out[2] = MAX_int32_T;
      }

      for (b_ia = 0; b_ia < 3; b_ia++) {
        tmp_2 = roundf(LocalizationCore_B.POINT_LIST[((b_ia << 11) + (int32_T)
          LocalizationCore_B.indices[0]) - 1]);
        if (tmp_2 < 2.14748365E+9F) {
          if (tmp_2 >= -2.14748365E+9F) {
            rtb_neighbors[b_ia << 1] = (int32_T)tmp_2;
          } else {
            rtb_neighbors[b_ia << 1] = MIN_int32_T;
          }
        } else {
          rtb_neighbors[b_ia << 1] = MAX_int32_T;
        }

        tmp_2 = roundf(LocalizationCore_B.POINT_LIST[((b_ia << 11) + (int32_T)
          LocalizationCore_B.indices[1]) - 1]);
        if (tmp_2 < 2.14748365E+9F) {
          if (tmp_2 >= -2.14748365E+9F) {
            rtb_neighbors[1 + (b_ia << 1)] = (int32_T)tmp_2;
          } else {
            rtb_neighbors[1 + (b_ia << 1)] = MIN_int32_T;
          }
        } else {
          rtb_neighbors[1 + (b_ia << 1)] = MAX_int32_T;
        }
      }

      LocalizationCore_B.accuracy = (real32_T)(LocalizationCore_B.wifi_int[0] *
        1.58 / 14.0);

      /* End of MATLAB Function: '<S50>/kNN Localization core' */
    }

    /* End of Outputs for SubSystem: '<S16>/kNN Localizer' */

    /* MultiPortSwitch: '<S16>/Multiport Switch' incorporates:
     *  Constant: '<S2>/Constant'
     *  Constant: '<S47>/Constant'
     *  Constant: '<S48>/Constant'
     *  Constant: '<S49>/AP_LIST'
     *  Constant: '<S49>/MEAN_RSS'
     *  Constant: '<S49>/N_AP'
     *  Constant: '<S49>/N_POINTS'
     *  Constant: '<S49>/POINT_LIST'
     *  RelationalOperator: '<S47>/Compare'
     *  RelationalOperator: '<S48>/Compare'
     */
    if (rtb_DataTypeConversion1_h == 1) {
      LocalizationCore_B.MultiportSwitch[0] = LocalizationCore_B.position_out[0];
      LocalizationCore_B.MultiportSwitch[1] = LocalizationCore_B.position_out[1];
      LocalizationCore_B.MultiportSwitch[2] = LocalizationCore_B.position_out[2];

      /* MultiPortSwitch: '<S16>/Multiport Switch1' */
      LocalizationCore_B.MultiportSwitch1 = LocalizationCore_B.accuracy;
    } else {
      LocalizationCore_B.MultiportSwitch[0] = LocalizationCore_B.position_out_i
        [0];
      LocalizationCore_B.MultiportSwitch[1] = LocalizationCore_B.position_out_i
        [1];
      LocalizationCore_B.MultiportSwitch[2] = LocalizationCore_B.position_out_i
        [2];

      /* MultiPortSwitch: '<S16>/Multiport Switch1' */
      LocalizationCore_B.MultiportSwitch1 = LocalizationCore_B.accuracy_f;
    }

    /* End of MultiPortSwitch: '<S16>/Multiport Switch' */
  }

  /* End of Outputs for SubSystem: '<S2>/Wifi Localizer' */

  /* Math: '<S6>/Math Function' incorporates:
   *  Inport: '<Root>/RotationMatrix'
   *  Reshape: '<S1>/Reshape'
   */
  for (b_ia = 0; b_ia < 3; b_ia++) {
    rtb_MathFunction_g[3 * b_ia] = LocalizationCore_U.RotationMatrix[b_ia];
    rtb_MathFunction_g[1 + 3 * b_ia] = LocalizationCore_U.RotationMatrix[b_ia +
      3];
    rtb_MathFunction_g[2 + 3 * b_ia] = LocalizationCore_U.RotationMatrix[b_ia +
      6];
  }

  /* End of Math: '<S6>/Math Function' */

  /* Product: '<S13>/Product' incorporates:
   *  Inport: '<Root>/Gyro'
   *  Math: '<S32>/Math Function'
   */
  for (b_ia = 0; b_ia < 3; b_ia++) {
    x0_n[b_ia] = rtb_MathFunction_g[b_ia + 6] * LocalizationCore_U.Gyro[2] +
      (rtb_MathFunction_g[b_ia + 3] * LocalizationCore_U.Gyro[1] +
       rtb_MathFunction_g[b_ia] * LocalizationCore_U.Gyro[0]);
  }

  /* End of Product: '<S13>/Product' */

  /* S-Function (sdspmultiportsel): '<S13>/Select Z-Axis' */
  LocalizationCore_B.Differentiate = x0_n[LocalizationCore_ConstP.pooled4];

  /* S-Function (sdspfilter2): '<S13>/Digital Filter' */
  rtb_MultiportSwitch1 = LocalizationCore_B.Differentiate *
    LocalizationCore_P.DigitalFilter_RTP1COEFF[0];
  ar = 1;
  for (b_ia = LocalizationCore_DW.DigitalFilter_CIRCBUFFIDX; b_ia < 4; b_ia++) {
    rtb_MultiportSwitch1 += LocalizationCore_DW.DigitalFilter_FILT_STATES[b_ia] *
      LocalizationCore_P.DigitalFilter_RTP1COEFF[ar];
    ar++;
  }

  for (b_ia = 0; b_ia < LocalizationCore_DW.DigitalFilter_CIRCBUFFIDX; b_ia++) {
    rtb_MultiportSwitch1 += LocalizationCore_DW.DigitalFilter_FILT_STATES[b_ia] *
      LocalizationCore_P.DigitalFilter_RTP1COEFF[ar];
    ar++;
  }

  i = LocalizationCore_DW.DigitalFilter_CIRCBUFFIDX - 1;
  if (i < 0) {
    i = 3;
  }

  LocalizationCore_DW.DigitalFilter_FILT_STATES[i] =
    LocalizationCore_B.Differentiate;
  LocalizationCore_B.SelectZAxis = rtb_MultiportSwitch1;
  LocalizationCore_DW.DigitalFilter_CIRCBUFFIDX = i;

  /* End of S-Function (sdspfilter2): '<S13>/Digital Filter' */

  /* Gain: '<S14>/Gain' incorporates:
   *  DiscreteIntegrator: '<S14>/Discrete-Time Integrator'
   */
  rtb_Gain = LocalizationCore_P.Gain_Gain *
    LocalizationCore_DW.DiscreteTimeIntegrator_DSTATE;

  /* Product: '<S14>/Product' incorporates:
   *  Inport: '<Root>/LinearAccel'
   *  Math: '<S35>/Math Function'
   */
  for (b_ia = 0; b_ia < 3; b_ia++) {
    x0_n[b_ia] = rtb_MathFunction_g[b_ia + 6] * LocalizationCore_U.LinearAccel[2]
      + (rtb_MathFunction_g[b_ia + 3] * LocalizationCore_U.LinearAccel[1] +
         rtb_MathFunction_g[b_ia] * LocalizationCore_U.LinearAccel[0]);
  }

  /* End of Product: '<S14>/Product' */

  /* S-Function (sdspfilter2): '<S8>/LPF' incorporates:
   *  Inport: '<Root>/Pressure'
   */
  rtb_MultiportSwitch1 = LocalizationCore_U.Pressure *
    LocalizationCore_P.LPF_RTP1COEFF[0];
  ar = 1;
  for (b_ia = LocalizationCore_DW.LPF_CIRCBUFFIDX; b_ia < 49; b_ia++) {
    rtb_MultiportSwitch1 += LocalizationCore_DW.LPF_FILT_STATES[b_ia] *
      LocalizationCore_P.LPF_RTP1COEFF[ar];
    ar++;
  }

  for (b_ia = 0; b_ia < LocalizationCore_DW.LPF_CIRCBUFFIDX; b_ia++) {
    rtb_MultiportSwitch1 += LocalizationCore_DW.LPF_FILT_STATES[b_ia] *
      LocalizationCore_P.LPF_RTP1COEFF[ar];
    ar++;
  }

  i = LocalizationCore_DW.LPF_CIRCBUFFIDX - 1;
  if (i < 0) {
    i = 48;
  }

  LocalizationCore_DW.LPF_FILT_STATES[i] = LocalizationCore_U.Pressure;
  LocalizationCore_B.SelectZAxis = rtb_MultiportSwitch1;
  LocalizationCore_DW.LPF_CIRCBUFFIDX = i;

  /* End of S-Function (sdspfilter2): '<S8>/LPF' */

  /* S-Function (sdspfilter2): '<S8>/Differentiate' */
  rtb_MultiportSwitch1 = LocalizationCore_B.SelectZAxis *
    LocalizationCore_P.Differentiate_RTP1COEFF[0];
  ar = 1;
  for (b_ia = LocalizationCore_DW.Differentiate_CIRCBUFFIDX; b_ia < 1; b_ia++) {
    rtb_MultiportSwitch1 += LocalizationCore_DW.Differentiate_FILT_STATES[b_ia] *
      LocalizationCore_P.Differentiate_RTP1COEFF[ar];
    ar++;
  }

  for (b_ia = 0; b_ia < LocalizationCore_DW.Differentiate_CIRCBUFFIDX; b_ia++) {
    rtb_MultiportSwitch1 += LocalizationCore_DW.Differentiate_FILT_STATES[b_ia] *
      LocalizationCore_P.Differentiate_RTP1COEFF[ar];
    ar++;
  }

  i = LocalizationCore_DW.Differentiate_CIRCBUFFIDX - 1;
  if (i < 0) {
    i = 0;
  }

  LocalizationCore_DW.Differentiate_FILT_STATES[i] =
    LocalizationCore_B.SelectZAxis;
  LocalizationCore_B.Differentiate = rtb_MultiportSwitch1;
  LocalizationCore_DW.Differentiate_CIRCBUFFIDX = i;

  /* End of S-Function (sdspfilter2): '<S8>/Differentiate' */

  /* Gain: '<S8>/Gain' */
  rtb_Gain_e = LocalizationCore_P.Gain_Gain_m * LocalizationCore_B.Differentiate;

  /* DataTypeConversion: '<S1>/Data Type Conversion' incorporates:
   *  Inport: '<Root>/GPS_Status'
   */
  rtb_DataTypeConversion = (real32_T)LocalizationCore_U.GPS_Status;

  /* Outputs for Enabled SubSystem: '<S2>/GPS to Pixel' incorporates:
   *  EnablePort: '<S9>/Enable'
   */
  if ((real32_T)LocalizationCore_U.GPS_Status > 0.0F) {
    /* MATLAB Function: '<S9>/MATLAB Function' incorporates:
     *  Inport: '<Root>/GPS'
     */
    /* MATLAB Function 'LocalizationCore/Core/GPS to Pixel/MATLAB Function': '<S26>:1' */
    /*  GPS to pixel conversion for bahen */
    /* '<S26>:1:5' */
    /* '<S26>:1:7' */
    temp[1] = ((LocalizationCore_U.GPS[0] - 43.660038F) -
               (LocalizationCore_U.GPS[1] - -79.3981247F) * 0.234756231F) /
      -6.88445539E-7F;
    temp[0] = ((LocalizationCore_U.GPS[1] - -79.3981247F) - temp[1] *
               3.01265828E-7F) / 8.49406831E-7F;

    /* '<S26>:1:8' */
    /* '<S26>:1:9' */
    LocalizationCore_B.position[0] = temp[0];
    LocalizationCore_B.position[1] = temp[1];
    LocalizationCore_B.accuracy_n = LocalizationCore_U.GPS[5];
  }

  /* End of Outputs for SubSystem: '<S2>/GPS to Pixel' */

  /* MATLAB Function: '<S1>/MATLAB Function5' incorporates:
   *  Inport: '<Root>/Ble_MAC'
   */
  /* MATLAB Function 'LocalizationCore/MATLAB Function5': '<S5>:1' */
  /* '<S5>:1:3' */
  memset(&rtb_y_a[0], 0, 1700U * sizeof(uint8_T));

  /* '<S5>:1:4' */
  for (ia = 0; ia < 100; ia++) {
    /* '<S5>:1:4' */
    ar = ia * 17 + 1;
    br = (1 + ia) * 17;
    if (ar > br) {
      ar = 1;
      br = 0;
    }

    /* '<S5>:1:5' */
    b_ia = ar - 1;
    i = br - b_ia;
    for (h = 0; h < i; h++) {
      AP_list_j[h] = LocalizationCore_U.Ble_MAC[b_ia + h];
    }

    for (b_ia = 0; b_ia < 17; b_ia++) {
      rtb_y_a[ia + 100 * b_ia] = AP_list_j[b_ia];
    }

    /* '<S5>:1:4' */
  }

  /* End of MATLAB Function: '<S1>/MATLAB Function5' */

  /* MATLAB Function: '<S1>/MATLAB Function1' incorporates:
   *  Inport: '<Root>/Ble_Coordinates'
   */
  /* MATLAB Function 'LocalizationCore/MATLAB Function1': '<S3>:1' */
  /* '<S3>:1:3' */
  memset(&rtb_y[0], (int32_T)0.0F, 300U * sizeof(real32_T));

  /* '<S3>:1:4' */
  for (ia = 0; ia < 100; ia++) {
    /* '<S3>:1:4' */
    ar = ia * 3 + 1;
    br = (1 + ia) * 3;
    if (ar > br) {
      ar = 1;
      br = 0;
    }

    /* '<S3>:1:5' */
    i = br - ar;
    for (b_ia = 0; b_ia <= i; b_ia++) {
      tmp_data[b_ia] = (int16_T)(ar + b_ia);
    }

    i = (br - ar) + 1;
    for (b_ia = 0; b_ia < i; b_ia++) {
      for (h = 0; h < 1; h++) {
        tmp_data_0[b_ia] = LocalizationCore_U.Ble_Coordinates[tmp_data[b_ia] - 1];
      }
    }

    rtb_y[ia] = tmp_data_0[0];
    rtb_y[ia + 100] = tmp_data_0[1];
    rtb_y[ia + 200] = tmp_data_0[2];

    /* '<S3>:1:4' */
  }

  /* End of MATLAB Function: '<S1>/MATLAB Function1' */

  /* Outputs for Enabled SubSystem: '<S2>/MAC Merge and Filter BLE' incorporates:
   *  EnablePort: '<S11>/Enable'
   */
  /* DataTypeConversion: '<S1>/Data Type Conversion2' incorporates:
   *  Constant: '<S19>/Constant'
   *  Constant: '<S20>/Constant'
   *  Constant: '<S21>/Constant'
   *  Inport: '<Root>/Ble_Status'
   *  MATLAB Function: '<S22>/Trilateration core'
   *  RelationalOperator: '<S19>/Compare'
   *  RelationalOperator: '<S20>/Compare'
   *  RelationalOperator: '<S21>/Compare'
   */
  if ((real32_T)LocalizationCore_U.Ble_Status > 0.0F) {
    /* MATLAB Function: '<S11>/MAC Filter and Merge' incorporates:
     *  Inport: '<Root>/Ble_RSS'
     *  Inport: '<Root>/Ble_TX_Powers'
     */
    /* MATLAB Function 'LocalizationCore/Core/MAC Merge and Filter BLE/MAC Filter and Merge': '<S29>:1' */
    /* '<S29>:1:5' */
    /* '<S29>:1:4' */
    memset(&restricted_MAC[0], 0, 80U * sizeof(uint8_T));

    /* '<S29>:1:5' */
    for (b_ia = 0; b_ia < 8; b_ia++) {
      restricted_MAC[10 * b_ia] = b_0[b_ia];
    }

    /* restricted_MAC(2,:)=uint8('00:0c:e6'); */
    /*  MAC filtering */
    /* for k=1:n_ap */
    /*     availableAP(k,17)=uint8('0'); */
    /* end */
    /* '<S29>:1:16' */
    memset(&AP_list_j[0], 0, 1700U * sizeof(uint8_T));

    /* '<S29>:1:17' */
    memset(&LocalizationCore_B.Coordinates[0], (int32_T)0.0F, 300U * sizeof
           (real32_T));

    /* '<S29>:1:18' */
    /* '<S29>:1:19' */
    for (i = 0; i < 100; i++) {
      LocalizationCore_B.TX_powers[i] = 0.0F;
      AP_visited_0[i] = 0.0F;
    }

    /* '<S29>:1:20' */
    memset(&rss_new[0], (int32_T)0.0F, 1000U * sizeof(real32_T));

    /* '<S29>:1:22' */
    new_counter = 1.0;

    /* '<S29>:1:25' */
    n_ap_new = 0.0;

    /* '<S29>:1:26' */
    for (ia = 0; ia < 100; ia++) {
      /* '<S29>:1:26' */
      if (LocalizationCore_U.Ble_RSS[ia] != -200.0F) {
        /* '<S29>:1:27' */
        /* '<S29>:1:28' */
        n_ap_new++;
      }

      /* '<S29>:1:26' */
    }

    /* '<S29>:1:37' */
    for (ar = 0; ar < (int32_T)n_ap_new; ar++) {
      /* '<S29>:1:37' */
      /* '<S29>:1:38' */
      br = 0;

      /* '<S29>:1:39' */
      for (r3 = 0; r3 < 10; r3++) {
        /* '<S29>:1:39' */
        b_ia = (int32_T)(1.0 + (real_T)ar);
        for (h = 0; h < 8; h++) {
          rtb_y_a_0[h] = rtb_y_a[(100 * h + b_ia) - 1];
          restricted_MAC_0[h] = restricted_MAC[10 * h + r3];
        }

        if (LocalizationCore_isequaln(rtb_y_a_0, restricted_MAC_0)) {
          /* '<S29>:1:41' */
          br = 1;
        }

        /* '<S29>:1:39' */
      }

      if (br == 0) {
        /* '<S29>:1:45' */
        /* '<S29>:1:46' */
        search_flag = 0.0;

        /* '<S29>:1:47' */
        while (br <= (int32_T)(new_counter - 1.0) - 1) {
          /* '<S29>:1:47' */
          b_ia = (int32_T)(1.0 + (real_T)br);
          h = (int32_T)(1.0 + (real_T)ar);
          for (i = 0; i < 17; i++) {
            AP_list_j_0[i] = AP_list_j[(100 * i + b_ia) - 1];
            rtb_y_a_1[i] = rtb_y_a[(100 * i + h) - 1];
          }

          if (LocalizationCore_isequaln_g(AP_list_j_0, rtb_y_a_1)) {
            /* '<S29>:1:49' */
            search_flag = 1.0 + (real_T)br;
          }

          /* '<S29>:1:47' */
          br++;
        }

        if (search_flag == 0.0) {
          /* '<S29>:1:52' */
          /* '<S29>:1:53' */
          i = (int32_T)new_counter;
          b_ia = (int32_T)(1.0 + (real_T)ar);
          for (h = 0; h < 17; h++) {
            AP_list_j[(i + 100 * h) - 1] = rtb_y_a[(100 * h + b_ia) - 1];
          }

          /* '<S29>:1:54' */
          i = (int32_T)new_counter;
          b_ia = (int32_T)(1.0 + (real_T)ar);
          LocalizationCore_B.Coordinates[i - 1] = rtb_y[b_ia - 1];
          LocalizationCore_B.Coordinates[i + 99] = rtb_y[b_ia + 99];
          LocalizationCore_B.Coordinates[i + 199] = rtb_y[b_ia + 199];

          /* '<S29>:1:55' */
          LocalizationCore_B.TX_powers[(int32_T)new_counter - 1] =
            LocalizationCore_U.Ble_TX_Powers[(int32_T)(1.0 + (real_T)ar) - 1];

          /* '<S29>:1:56' */
          AP_visited_0[(int32_T)new_counter - 1] = 1.0F;

          /* '<S29>:1:57' */
          rss_new[10 * ((int32_T)new_counter - 1)] = LocalizationCore_U.Ble_RSS
            [(int32_T)(1.0 + (real_T)ar) - 1];

          /* '<S29>:1:58' */
          new_counter++;
        } else {
          /* '<S29>:1:60' */
          AP_visited_0[(int32_T)search_flag - 1]++;

          /* '<S29>:1:61' */
          rss_new[((int32_T)AP_visited_0[(int32_T)search_flag - 1] + 10 *
                   ((int32_T)search_flag - 1)) - 1] =
            LocalizationCore_U.Ble_RSS[(int32_T)(1.0 + (real_T)ar) - 1];
        }
      }

      /* '<S29>:1:37' */
    }

    /* % Callculating means */
    /* '<S29>:1:69' */
    memset(&LocalizationCore_B.mean_rss_e[0], (int32_T)0.0F, 100U * sizeof
           (real32_T));

    /* '<S29>:1:71' */
    for (br = 0; br < 100; br++) {
      /* '<S29>:1:71' */
      /* '<S29>:1:72' */
      new_counter = 0.0;

      /* '<S29>:1:73' */
      for (ar = 0; ar < 10; ar++) {
        /* '<S29>:1:73' */
        if ((rss_new[10 * br + ar] != 0.0F) && (rss_new[10 * br + ar] != -110.0F)
            && (rss_new[10 * br + ar] <= -20.0F)) {
          /* '<S29>:1:74' */
          /* '<S29>:1:75' */
          LocalizationCore_B.mean_rss_e[br] += rss_new[10 * br + ar];

          /* '<S29>:1:76' */
          new_counter++;
        }

        /* '<S29>:1:73' */
      }

      if (new_counter == 0.0) {
        /* '<S29>:1:79' */
        /* '<S29>:1:80' */
        LocalizationCore_B.mean_rss_e[br] = -110.0F;
      } else {
        /* '<S29>:1:82' */
        LocalizationCore_B.mean_rss_e[br] /= (real32_T)new_counter;
      }

      /* '<S29>:1:71' */
    }

    /* End of MATLAB Function: '<S11>/MAC Filter and Merge' */
  }

  /* End of Outputs for SubSystem: '<S2>/MAC Merge and Filter BLE' */

  /* Outputs for Enabled SubSystem: '<S2>/BLE Localizer' incorporates:
   *  EnablePort: '<S7>/Enable'
   */
  if ((real32_T)LocalizationCore_U.Ble_Status > 0.0F) {
    /* Outputs for Enabled SubSystem: '<S7>/ Average Localizer' incorporates:
     *  EnablePort: '<S17>/Enable'
     */
    if (rtb_MultiportSelector1_o3 == LocalizationCore_P.Constant_Value_b) {
      /* MATLAB Function: '<S17>/Avreage core' */
      /* MATLAB Function 'LocalizationCore/Core/BLE Localizer/ Average Localizer/Avreage core': '<S23>:1' */
      /* '<S23>:1:3' */
      n_beacons = 0.0F;

      /* '<S23>:1:4' */
      memset(&AP_visited_0[0], (int32_T)0.0F, 100U * sizeof(real32_T));

      /* '<S23>:1:5' */
      rtb_MultiportSwitch1 = 0.0F;

      /* '<S23>:1:6' */
      num[0] = 0.0F;
      num[1] = 0.0F;
      num[2] = 0.0F;

      /* '<S23>:1:8' */
      for (ia = 0; ia < 100; ia++) {
        rtb_MultiportSelector1_o1 = AP_visited_0[ia];

        /* '<S23>:1:8' */
        if (LocalizationCore_B.mean_rss_e[ia] != -110.0F) {
          /* '<S23>:1:9' */
          /* '<S23>:1:10' */
          rtb_MultiportSelector1_o1 = powf(10.0F,
            (LocalizationCore_B.TX_powers[ia] - LocalizationCore_B.mean_rss_e[ia])
            / rtb_MultiportSelector1_o4);

          /* '<S23>:1:11' */
          num[0] += LocalizationCore_B.Coordinates[ia] /
            rtb_MultiportSelector1_o1;
          num[1] += LocalizationCore_B.Coordinates[ia + 100] /
            rtb_MultiportSelector1_o1;
          num[2] += LocalizationCore_B.Coordinates[ia + 200] /
            rtb_MultiportSelector1_o1;

          /* '<S23>:1:12' */
          rtb_MultiportSwitch1 += 1.0F / rtb_MultiportSelector1_o1;

          /* '<S23>:1:13' */
          n_beacons++;
        }

        /* '<S23>:1:8' */
        AP_visited_0[ia] = rtb_MultiportSelector1_o1;
      }

      if (n_beacons != 0.0F) {
        /* '<S23>:1:17' */
        /* '<S23>:1:18' */
        tmp_2 = roundf(num[0] / rtb_MultiportSwitch1);
        if (tmp_2 < 2.14748365E+9F) {
          if (tmp_2 >= -2.14748365E+9F) {
            LocalizationCore_B.position_out_m[0] = (int32_T)tmp_2;
          } else {
            LocalizationCore_B.position_out_m[0] = MIN_int32_T;
          }
        } else {
          LocalizationCore_B.position_out_m[0] = MAX_int32_T;
        }

        tmp_2 = roundf(num[1] / rtb_MultiportSwitch1);
        if (tmp_2 < 2.14748365E+9F) {
          if (tmp_2 >= -2.14748365E+9F) {
            LocalizationCore_B.position_out_m[1] = (int32_T)tmp_2;
          } else {
            LocalizationCore_B.position_out_m[1] = MIN_int32_T;
          }
        } else {
          LocalizationCore_B.position_out_m[1] = MAX_int32_T;
        }

        tmp_2 = roundf(num[2] / rtb_MultiportSwitch1);
        if (tmp_2 < 2.14748365E+9F) {
          if (tmp_2 >= -2.14748365E+9F) {
            LocalizationCore_B.position_out_m[2] = (int32_T)tmp_2;
          } else {
            LocalizationCore_B.position_out_m[2] = MIN_int32_T;
          }
        } else {
          LocalizationCore_B.position_out_m[2] = MAX_int32_T;
        }

        /* '<S23>:1:19' */
        LocalizationCore_B.accuracy_b = 5.0F;
      } else {
        /* '<S23>:1:21' */
        LocalizationCore_B.position_out_m[0] = 0;
        LocalizationCore_B.position_out_m[1] = 0;
        LocalizationCore_B.position_out_m[2] = 0;

        /* '<S23>:1:22' */
        LocalizationCore_B.accuracy_b = 5.0F;
      }

      /* End of MATLAB Function: '<S17>/Avreage core' */
    }

    /* End of Outputs for SubSystem: '<S7>/ Average Localizer' */

    /* Outputs for Enabled SubSystem: '<S7>/Closest Localizer' incorporates:
     *  EnablePort: '<S18>/Enable'
     */
    if (rtb_MultiportSelector1_o3 == LocalizationCore_P.Constant_Value_j) {
      /* MATLAB Function: '<S18>/Closest core' */
      /* MATLAB Function 'LocalizationCore/Core/BLE Localizer/Closest Localizer/Closest core': '<S24>:1' */
      rtb_MultiportSwitch1 = LocalizationCore_B.mean_rss_e[0];
      ar = -1;
      for (br = 0; br < 99; br++) {
        if (LocalizationCore_B.mean_rss_e[br + 1] > rtb_MultiportSwitch1) {
          rtb_MultiportSwitch1 = LocalizationCore_B.mean_rss_e[br + 1];
          ar = br;
        }
      }

      /* '<S24>:1:5' */
      tmp_2 = roundf(LocalizationCore_B.Coordinates[ar + 1]);
      if (tmp_2 < 2.14748365E+9F) {
        if (tmp_2 >= -2.14748365E+9F) {
          LocalizationCore_B.position_out_p[0] = (int32_T)tmp_2;
        } else {
          LocalizationCore_B.position_out_p[0] = MIN_int32_T;
        }
      } else {
        LocalizationCore_B.position_out_p[0] = MAX_int32_T;
      }

      tmp_2 = roundf(LocalizationCore_B.Coordinates[ar + 101]);
      if (tmp_2 < 2.14748365E+9F) {
        if (tmp_2 >= -2.14748365E+9F) {
          LocalizationCore_B.position_out_p[1] = (int32_T)tmp_2;
        } else {
          LocalizationCore_B.position_out_p[1] = MIN_int32_T;
        }
      } else {
        LocalizationCore_B.position_out_p[1] = MAX_int32_T;
      }

      tmp_2 = roundf(LocalizationCore_B.Coordinates[ar + 201]);
      if (tmp_2 < 2.14748365E+9F) {
        if (tmp_2 >= -2.14748365E+9F) {
          LocalizationCore_B.position_out_p[2] = (int32_T)tmp_2;
        } else {
          LocalizationCore_B.position_out_p[2] = MIN_int32_T;
        }
      } else {
        LocalizationCore_B.position_out_p[2] = MAX_int32_T;
      }

      /* '<S24>:1:6' */
      LocalizationCore_B.accuracy_g = 5.0F;

      /* End of MATLAB Function: '<S18>/Closest core' */
    }

    /* End of Outputs for SubSystem: '<S7>/Closest Localizer' */

    /* Outputs for Enabled SubSystem: '<S7>/Trilateration Localizer' incorporates:
     *  EnablePort: '<S22>/Enable'
     */
    if (rtb_MultiportSelector1_o3 == LocalizationCore_P.Constant_Value_o) {
      /* MATLAB Function: '<S22>/Trilateration core' */
      /* MATLAB Function 'LocalizationCore/Core/BLE Localizer/Trilateration Localizer/Trilateration core': '<S25>:1' */
      /* '<S25>:1:5' */
      n_beacons = 0.0F;

      /* '<S25>:1:6' */
      LocalizationCore_eml_sort(LocalizationCore_B.mean_rss_e, c_y, iidx);

      /* '<S25>:1:9' */
      /* '<S25>:1:10' */
      /* '<S25>:1:12' */
      /* '<S25>:1:22' */
      for (i = 0; i < 100; i++) {
        rtb_MultiportSelector1_o1 = 0.0F;

        /* '<S25>:1:12' */
        if (c_y[i] != -110.0F) {
          /* '<S25>:1:13' */
          /* '<S25>:1:14' */
          rtb_MultiportSelector1_o1 = powf(10.0F,
            (LocalizationCore_B.TX_powers[iidx[i] - 1] - c_y[i]) /
            rtb_MultiportSelector1_o4);

          /*         num = num + coordinates(k,:)/distances(k); */
          /*         denum = denum + 1/distances(k); */
          /* '<S25>:1:17' */
          n_beacons++;
        }

        /* '<S25>:1:12' */
        rtb_MultiportSelector1_o1 *= 13.5F;
        AP_visited_0[i] = rtb_MultiportSelector1_o1;
      }

      /* '<S25>:1:24' */
      A2x[0] = 0.0F;
      A2x[1] = 0.0F;
      A2x[2] = 0.0F;
      A2x[3] = 0.0F;

      /* '<S25>:1:25' */
      /* '<S25>:1:27' */
      for (b_ia = 0; b_ia < 3; b_ia++) {
        for (h = 0; h < 100; h++) {
          rtb_y[h + 100 * b_ia] = LocalizationCore_B.Coordinates[(100 * b_ia +
            iidx[h]) - 1];
        }
      }

      tmp[0] = rtb_y[0];
      tmp[1] = rtb_y[100];
      tmp[2] = rtb_y[200];
      rtb_MultiportSwitch1 = LocalizationCore_norm(tmp);
      rtb_MultiportSwitch1 = AP_visited_0[0] * AP_visited_0[0] -
        rtb_MultiportSwitch1 * rtb_MultiportSwitch1;

      /* '<S25>:1:29' */
      for (ar = 0; ar < 2; ar++) {
        /* '<S25>:1:29' */
        /* '<S25>:1:30' */
        for (b_ia = 0; b_ia < 3; b_ia++) {
          for (h = 0; h < 100; h++) {
            rtb_y[h + 100 * b_ia] = LocalizationCore_B.Coordinates[(100 * b_ia +
              iidx[h]) - 1];
          }
        }

        for (b_ia = 0; b_ia < 3; b_ia++) {
          for (h = 0; h < 100; h++) {
            tmp_data_0[h + 100 * b_ia] = LocalizationCore_B.Coordinates[(100 *
              b_ia + iidx[h]) - 1];
          }
        }

        A2x[ar] = rtb_y[ar + 1] - tmp_data_0[0];
        A2x[ar + 2] = rtb_y[ar + 101] - tmp_data_0[100];

        /* '<S25>:1:31' */
        for (b_ia = 0; b_ia < 3; b_ia++) {
          for (h = 0; h < 100; h++) {
            rtb_y[h + 100 * b_ia] = LocalizationCore_B.Coordinates[(100 * b_ia +
              iidx[h]) - 1];
          }
        }

        tmp_1[0] = rtb_y[ar + 1];
        tmp_1[1] = rtb_y[ar + 101];
        tmp_1[2] = rtb_y[ar + 201];
        rtb_MultiportSelector1_o1 = LocalizationCore_norm(tmp_1);
        b2x[ar] = (rtb_MultiportSwitch1 - AP_visited_0[1 + ar] * AP_visited_0[1
                   + ar]) + rtb_MultiportSelector1_o1 *
          rtb_MultiportSelector1_o1;

        /* '<S25>:1:29' */
      }

      /* '<S25>:1:34' */
      A2x[0] *= 2.0F;
      A2x[1] *= 2.0F;
      A2x[2] *= 2.0F;

      /* MATLAB Function: '<S22>/Trilateration core' */
      rtb_MultiportSelector1_o1 = A2x[3] * 2.0F;
      A2x[3] = rtb_MultiportSelector1_o1;

      /* MATLAB Function: '<S22>/Trilateration core' */
      /* '<S25>:1:36' */
      for (b_ia = 0; b_ia < 2; b_ia++) {
        y[b_ia] = 0.0F;
        y[b_ia] += A2x[b_ia << 1] * A2x[0];
        y[b_ia] += A2x[(b_ia << 1) + 1] * A2x[1];
        y[b_ia + 2] = 0.0F;
        y[b_ia + 2] += A2x[b_ia << 1] * A2x[2];
        y[b_ia + 2] += A2x[(b_ia << 1) + 1] * rtb_MultiportSelector1_o1;
      }

      temp[0] = A2x[0] * b2x[0] + A2x[1] * b2x[1];
      temp[1] = A2x[2] * b2x[0];
      temp[1] += rtb_MultiportSelector1_o1 * b2x[1];

      /* MATLAB Function: '<S22>/Trilateration core' */
      if (fabsf(y[1]) > fabsf(y[0])) {
        ar = 1;
        br = 0;
      } else {
        ar = 0;
        br = 1;
      }

      rtb_MultiportSelector1_o1 = y[br] / y[ar];
      rtb_MultiportSelector1_o1 = (temp[br] - temp[ar] *
        rtb_MultiportSelector1_o1) / (y[2 + br] - y[2 + ar] *
        rtb_MultiportSelector1_o1);
      if (n_beacons > 2.0F) {
        /* '<S25>:1:38' */
        /* '<S25>:1:39' */
        tmp_2 = roundf((temp[ar] - y[2 + ar] * rtb_MultiportSelector1_o1) / y[ar]);
        if (tmp_2 < 2.14748365E+9F) {
          if (tmp_2 >= -2.14748365E+9F) {
            LocalizationCore_B.position_out_n[0] = (int32_T)tmp_2;
          } else {
            LocalizationCore_B.position_out_n[0] = MIN_int32_T;
          }
        } else {
          LocalizationCore_B.position_out_n[0] = MAX_int32_T;
        }

        tmp_2 = roundf(rtb_MultiportSelector1_o1);
        if (tmp_2 < 2.14748365E+9F) {
          if (tmp_2 >= -2.14748365E+9F) {
            LocalizationCore_B.position_out_n[1] = (int32_T)tmp_2;
          } else {
            LocalizationCore_B.position_out_n[1] = MIN_int32_T;
          }
        } else {
          LocalizationCore_B.position_out_n[1] = MAX_int32_T;
        }

        LocalizationCore_B.position_out_n[2] = 4;

        /* '<S25>:1:40' */
      } else {
        /* '<S25>:1:42' */
        LocalizationCore_B.position_out_n[0] = 0;
        LocalizationCore_B.position_out_n[1] = 0;
        LocalizationCore_B.position_out_n[2] = 0;

        /* '<S25>:1:43' */
      }

      LocalizationCore_B.accuracy_e = 5.0F;
    }

    /* End of Outputs for SubSystem: '<S7>/Trilateration Localizer' */

    /* MultiPortSwitch: '<S7>/Multiport Switch' incorporates:
     *  Constant: '<S19>/Constant'
     *  Constant: '<S20>/Constant'
     *  Constant: '<S21>/Constant'
     *  MATLAB Function: '<S22>/Trilateration core'
     *  RelationalOperator: '<S19>/Compare'
     *  RelationalOperator: '<S20>/Compare'
     *  RelationalOperator: '<S21>/Compare'
     */
    switch ((int32_T)rtb_MultiportSelector1_o3) {
     case 1:
      LocalizationCore_B.MultiportSwitch_c[0] =
        LocalizationCore_B.position_out_p[0];
      LocalizationCore_B.MultiportSwitch_c[1] =
        LocalizationCore_B.position_out_p[1];
      LocalizationCore_B.MultiportSwitch_c[2] =
        LocalizationCore_B.position_out_p[2];

      /* MultiPortSwitch: '<S7>/Multiport Switch1' */
      LocalizationCore_B.MultiportSwitch1_j = LocalizationCore_B.accuracy_g;
      break;

     case 2:
      LocalizationCore_B.MultiportSwitch_c[0] =
        LocalizationCore_B.position_out_m[0];
      LocalizationCore_B.MultiportSwitch_c[1] =
        LocalizationCore_B.position_out_m[1];
      LocalizationCore_B.MultiportSwitch_c[2] =
        LocalizationCore_B.position_out_m[2];

      /* MultiPortSwitch: '<S7>/Multiport Switch1' */
      LocalizationCore_B.MultiportSwitch1_j = LocalizationCore_B.accuracy_b;
      break;

     default:
      LocalizationCore_B.MultiportSwitch_c[0] =
        LocalizationCore_B.position_out_n[0];
      LocalizationCore_B.MultiportSwitch_c[1] =
        LocalizationCore_B.position_out_n[1];
      LocalizationCore_B.MultiportSwitch_c[2] =
        LocalizationCore_B.position_out_n[2];

      /* MultiPortSwitch: '<S7>/Multiport Switch1' */
      LocalizationCore_B.MultiportSwitch1_j = LocalizationCore_B.accuracy_e;
      break;
    }

    /* End of MultiPortSwitch: '<S7>/Multiport Switch' */
  }

  /* End of DataTypeConversion: '<S1>/Data Type Conversion2' */
  /* End of Outputs for SubSystem: '<S2>/BLE Localizer' */

  /* Outputs for Enabled SubSystem: '<S15>/KALMAN Filter' incorporates:
   *  EnablePort: '<S38>/Enable'
   */
  /* RelationalOperator: '<S36>/Compare' incorporates:
   *  Constant: '<S36>/Constant'
   *  MATLAB Function: '<S38>/KALMAN Filter'
   */
  if (rtb_DataTypeConversion2_h == LocalizationCore_P.Constant_Value_l) {
    /* MATLAB Function 'LocalizationCore/Core/Tracking Module/KALMAN Filter/KALMAN Filter': '<S40>:1' */
    /* '<S40>:1:10' */
    /* '<S40>:1:11' */
    num[0] = (real32_T)LocalizationCore_B.MultiportSwitch[0] / 13.5F;

    /* MATLAB Function: '<S38>/KALMAN Filter' */
    rtb_MultiportSelector1_o4 = (real32_T)LocalizationCore_B.MultiportSwitch[1] /
      13.5F;
    num[1] = rtb_MultiportSelector1_o4;

    /* MATLAB Function: '<S38>/KALMAN Filter' incorporates:
     *  DataTypeConversion: '<S1>/Data Type Conversion'
     *  Inport: '<Root>/GPS_Status'
     *  UnitDelay: '<S38>/Unit Delay'
     *  UnitDelay: '<S38>/Unit Delay1'
     *  UnitDelay: '<S38>/Unit Delay2'
     */
    /* '<S40>:1:12' */
    num[2] = ((real32_T)LocalizationCore_B.MultiportSwitch[2] - 1.0F) * 4.625F;

    /* '<S40>:1:13' */
    /* '<S40>:1:17' */
    n_beacons = LocalizationCore_DW.UnitDelay2_DSTATE_i;

    /* '<S40>:1:22' */
    /* '<S40>:1:26' */
    for (b_ia = 0; b_ia < 6; b_ia++) {
      tmp_2 = 0.0F;
      for (h = 0; h < 6; h++) {
        tmp_2 += a[6 * h + b_ia] * LocalizationCore_DW.UnitDelay_DSTATE_f[h];
      }

      X_k_k[b_ia] = tmp_2;
    }

    /* '<S40>:1:27' */
    for (b_ia = 0; b_ia < 6; b_ia++) {
      for (h = 0; h < 6; h++) {
        a_1[b_ia + 6 * h] = 0.0F;
        for (i = 0; i < 6; i++) {
          a_1[b_ia + 6 * h] += a[6 * i + b_ia] *
            LocalizationCore_DW.UnitDelay1_DSTATE_i[6 * h + i];
        }
      }
    }

    for (b_ia = 0; b_ia < 6; b_ia++) {
      for (h = 0; h < 6; h++) {
        tmp_2 = 0.0F;
        for (i = 0; i < 6; i++) {
          tmp_2 += a_1[6 * i + b_ia] * b_1[6 * h + i];
        }

        P_k_k[b_ia + 6 * h] = Q[6 * h + b_ia] + tmp_2;
      }
    }

    if ((rtb_DataTypeConversion1 != 0.0F) || ((real32_T)
         LocalizationCore_U.GPS_Status != 0.0F)) {
      if (rtb_DataTypeConversion1 != 0.0F) {
        /* '<S40>:1:32' */
        rtb_MultiportSwitch1 = LocalizationCore_B.MultiportSwitch1 *
          LocalizationCore_B.MultiportSwitch1;

        /* '<S40>:1:33' */
        /* '<S40>:1:34' */
        /* '<S40>:1:35' */
        for (b_ia = 0; b_ia < 3; b_ia++) {
          for (h = 0; h < 6; h++) {
            c_a_1[b_ia + 3 * h] = 0.0F;
            for (i = 0; i < 6; i++) {
              c_a_1[b_ia + 3 * h] += (real32_T)c_a[3 * i + b_ia] * P_k_k[6 * h +
                i];
            }
          }
        }

        for (b_ia = 0; b_ia < 3; b_ia++) {
          for (h = 0; h < 3; h++) {
            tmp_2 = 0.0F;
            for (i = 0; i < 6; i++) {
              tmp_2 += c_a_1[3 * i + h] * (real32_T)d_b[6 * b_ia + i];
            }

            b_A[b_ia + 3 * h] = (real32_T)e[3 * b_ia + h] * rtb_MultiportSwitch1
              + tmp_2;
          }
        }

        for (b_ia = 0; b_ia < 3; b_ia++) {
          for (h = 0; h < 6; h++) {
            b_B[b_ia + 3 * h] = 0.0F;
            for (i = 0; i < 6; i++) {
              b_B[b_ia + 3 * h] += P_k_k[6 * i + h] * (real32_T)d_b[6 * b_ia + i];
            }
          }
        }

        br = 0;
        b_ia = 1;
        r3 = 2;
        rtb_MultiportSwitch1 = fabsf(b_A[0]);
        rtb_MultiportSelector1_o1 = fabsf(b_A[1]);
        if (rtb_MultiportSelector1_o1 > rtb_MultiportSwitch1) {
          rtb_MultiportSwitch1 = rtb_MultiportSelector1_o1;
          br = 1;
          b_ia = 0;
        }

        if (fabsf(b_A[2]) > rtb_MultiportSwitch1) {
          br = 2;
          b_ia = 1;
          r3 = 0;
        }

        b_A[b_ia] /= b_A[br];
        b_A[r3] /= b_A[br];
        b_A[3 + b_ia] -= b_A[3 + br] * b_A[b_ia];
        b_A[3 + r3] -= b_A[3 + br] * b_A[r3];
        b_A[6 + b_ia] -= b_A[6 + br] * b_A[b_ia];
        b_A[6 + r3] -= b_A[6 + br] * b_A[r3];
        if (fabsf(b_A[3 + r3]) > fabsf(b_A[3 + b_ia])) {
          ar = b_ia;
          b_ia = r3;
          r3 = ar;
        }

        b_A[3 + r3] /= b_A[3 + b_ia];
        b_A[6 + r3] -= b_A[3 + r3] * b_A[6 + b_ia];
        for (ar = 0; ar < 6; ar++) {
          c_a_1[3 * ar] = b_B[3 * ar + br];
          c_a_1[1 + 3 * ar] = b_B[3 * ar + b_ia] - c_a_1[3 * ar] * b_A[b_ia];
          c_a_1[2 + 3 * ar] = (b_B[3 * ar + r3] - c_a_1[3 * ar] * b_A[r3]) -
            c_a_1[3 * ar + 1] * b_A[3 + r3];
          c_a_1[2 + 3 * ar] /= b_A[6 + r3];
          c_a_1[3 * ar] -= c_a_1[3 * ar + 2] * b_A[6 + br];
          c_a_1[1 + 3 * ar] -= c_a_1[3 * ar + 2] * b_A[6 + b_ia];
          c_a_1[1 + 3 * ar] /= b_A[3 + b_ia];
          c_a_1[3 * ar] -= c_a_1[3 * ar + 1] * b_A[3 + br];
          c_a_1[3 * ar] /= b_A[br];
        }

        for (b_ia = 0; b_ia < 3; b_ia++) {
          for (h = 0; h < 6; h++) {
            K[h + 6 * b_ia] = c_a_1[3 * h + b_ia];
          }
        }

        /* '<S40>:1:36' */
        for (b_ia = 0; b_ia < 3; b_ia++) {
          tmp_2 = 0.0F;
          for (h = 0; h < 6; h++) {
            tmp_2 += (real32_T)c_a[3 * h + b_ia] * X_k_k[h];
          }

          z_data[b_ia] = num[b_ia] - tmp_2;
        }

        for (b_ia = 0; b_ia < 6; b_ia++) {
          rtb_states_out_l[b_ia] = ((K[b_ia + 6] * z_data[1] + K[b_ia] * z_data
            [0]) + K[b_ia + 12] * z_data[2]) +
            LocalizationCore_DW.UnitDelay_DSTATE_f[b_ia];
        }

        /* '<S40>:1:37' */
        for (b_ia = 0; b_ia < 36; b_ia++) {
          I[b_ia] = 0;
        }

        for (ar = 0; ar < 6; ar++) {
          I[ar + 6 * ar] = 1;
        }

        for (b_ia = 0; b_ia < 6; b_ia++) {
          for (h = 0; h < 6; h++) {
            a_1[b_ia + 6 * h] = (real32_T)I[6 * h + b_ia] - (((real32_T)c_a[3 *
              h + 1] * K[b_ia + 6] + (real32_T)c_a[3 * h] * K[b_ia]) + (real32_T)
              c_a[3 * h + 2] * K[b_ia + 12]);
          }
        }

        for (b_ia = 0; b_ia < 6; b_ia++) {
          for (h = 0; h < 6; h++) {
            rtb_cov_out_e[b_ia + 6 * h] = 0.0F;
            for (i = 0; i < 6; i++) {
              rtb_cov_out_e[b_ia + 6 * h] += a_1[6 * i + b_ia] * P_k_k[6 * h + i];
            }
          }
        }

        if (LocalizationCore_DW.UnitDelay2_DSTATE_i == 1.0F) {
          /* '<S40>:1:38' */
          /* '<S40>:1:39' */
          rtb_states_out_l[0] = num[0];

          /* '<S40>:1:40' */
          rtb_states_out_l[1] = rtb_MultiportSelector1_o4;

          /* '<S40>:1:41' */
          rtb_states_out_l[2] = num[2];

          /* '<S40>:1:42' */
          n_beacons = 0.0F;
        }
      } else {
        /* '<S40>:1:45' */
        rtb_MultiportSwitch1 = LocalizationCore_B.accuracy_n *
          LocalizationCore_B.accuracy_n;

        /* '<S40>:1:46' */
        /* '<S40>:1:47' */
        /* '<S40>:1:48' */
        for (b_ia = 0; b_ia < 2; b_ia++) {
          for (h = 0; h < 6; h++) {
            b_a_1[b_ia + (h << 1)] = 0.0F;
            for (i = 0; i < 6; i++) {
              b_a_1[b_ia + (h << 1)] += (real32_T)b_a[(i << 1) + b_ia] * P_k_k[6
                * h + i];
            }
          }
        }

        for (b_ia = 0; b_ia < 2; b_ia++) {
          for (h = 0; h < 2; h++) {
            tmp_2 = 0.0F;
            for (i = 0; i < 6; i++) {
              tmp_2 += b_a_1[(i << 1) + h] * (real32_T)b_b[6 * b_ia + i];
            }

            A2x[b_ia + (h << 1)] = (real32_T)d[(b_ia << 1) + h] *
              rtb_MultiportSwitch1 + tmp_2;
          }
        }

        for (b_ia = 0; b_ia < 2; b_ia++) {
          for (h = 0; h < 6; h++) {
            B[b_ia + (h << 1)] = 0.0F;
            for (i = 0; i < 6; i++) {
              B[b_ia + (h << 1)] += P_k_k[6 * i + h] * (real32_T)b_b[6 * b_ia +
                i];
            }
          }
        }

        if (fabsf(A2x[1]) > fabsf(A2x[0])) {
          ar = 1;
          br = 0;
        } else {
          ar = 0;
          br = 1;
        }

        rtb_MultiportSelector1_o1 = A2x[br] / A2x[ar];
        rtb_MultiportSwitch1 = A2x[2 + br] - A2x[2 + ar] *
          rtb_MultiportSelector1_o1;
        for (ia = 0; ia < 6; ia++) {
          b_a_1[1 + (ia << 1)] = (B[(ia << 1) + br] - B[(ia << 1) + ar] *
            rtb_MultiportSelector1_o1) / rtb_MultiportSwitch1;
          b_a_1[ia << 1] = (B[(ia << 1) + ar] - b_a_1[(ia << 1) + 1] * A2x[2 +
                            ar]) / A2x[ar];
        }

        for (b_ia = 0; b_ia < 2; b_ia++) {
          for (h = 0; h < 6; h++) {
            K_0[h + 6 * b_ia] = b_a_1[(h << 1) + b_ia];
          }
        }

        /* '<S40>:1:49' */
        for (b_ia = 0; b_ia < 2; b_ia++) {
          tmp_2 = 0.0F;
          for (h = 0; h < 6; h++) {
            tmp_2 += (real32_T)b_a[(h << 1) + b_ia] * X_k_k[h];
          }

          temp[b_ia] = LocalizationCore_B.position[b_ia] / 13.5F - tmp_2;
        }

        for (b_ia = 0; b_ia < 6; b_ia++) {
          rtb_states_out_l[b_ia] = (K_0[b_ia + 6] * temp[1] + K_0[b_ia] * temp[0])
            + LocalizationCore_DW.UnitDelay_DSTATE_f[b_ia];
        }

        /* '<S40>:1:50' */
        for (b_ia = 0; b_ia < 36; b_ia++) {
          I[b_ia] = 0;
        }

        for (br = 0; br < 6; br++) {
          I[br + 6 * br] = 1;
        }

        for (b_ia = 0; b_ia < 6; b_ia++) {
          for (h = 0; h < 6; h++) {
            a_1[b_ia + 6 * h] = (real32_T)I[6 * h + b_ia] - ((real32_T)b_a[(h <<
              1) + 1] * K_0[b_ia + 6] + (real32_T)b_a[h << 1] * K_0[b_ia]);
          }
        }

        for (b_ia = 0; b_ia < 6; b_ia++) {
          for (h = 0; h < 6; h++) {
            rtb_cov_out_e[b_ia + 6 * h] = 0.0F;
            for (i = 0; i < 6; i++) {
              rtb_cov_out_e[b_ia + 6 * h] += a_1[6 * i + b_ia] * P_k_k[6 * h + i];
            }
          }
        }
      }
    } else {
      /* '<S40>:1:53' */
      for (i = 0; i < 6; i++) {
        rtb_states_out_l[i] = X_k_k[i];
      }

      /* '<S40>:1:54' */
      memcpy(&rtb_cov_out_e[0], &P_k_k[0], 36U * sizeof(real32_T));
    }

    /* S-Function (sdspmultiportsel): '<S38>/Multiport Selector13' */
    /* '<S40>:1:59' */
    /* '<S40>:1:60' */
    rtb_MultiportSelector13_o1_n[0] =
      rtb_states_out_l[LocalizationCore_ConstP.pooled2[0]];
    rtb_MultiportSelector13_o1_n[1] =
      rtb_states_out_l[LocalizationCore_ConstP.pooled2[1]];
    rtb_MultiportSelector13_o1_n[2] =
      rtb_states_out_l[LocalizationCore_ConstP.pooled2[2]];
    LocalizationCore_B.MultiportSelector13_o2_a[0] =
      rtb_states_out_l[LocalizationCore_ConstP.pooled2[3]];
    LocalizationCore_B.MultiportSelector13_o2_a[1] =
      rtb_states_out_l[LocalizationCore_ConstP.pooled2[4]];
    LocalizationCore_B.MultiportSelector13_o2_a[2] =
      rtb_states_out_l[LocalizationCore_ConstP.pooled2[5]];

    /* MATLAB Function: '<S38>/MATLAB Function5' */
    LocalizationCor_MATLABFunction5(rtb_MultiportSelector13_o1_n,
      &LocalizationCore_B.sf_MATLABFunction5_h);

    /* DataTypeConversion: '<S38>/Data Type Conversion' */
    LocalizationCore_B.DataTypeConversion_n[0] =
      LocalizationCore_B.sf_MATLABFunction5_h.y[0];
    LocalizationCore_B.DataTypeConversion_n[1] =
      LocalizationCore_B.sf_MATLABFunction5_h.y[1];
    LocalizationCore_B.DataTypeConversion_n[2] =
      LocalizationCore_B.sf_MATLABFunction5_h.y[2];

    /* MATLAB Function: '<S38>/MATLAB Function4' */
    LocalizationCor_MATLABFunction4(rtb_cov_out_e,
      &LocalizationCore_B.sf_MATLABFunction4);

    /* Update for UnitDelay: '<S38>/Unit Delay' */
    for (i = 0; i < 6; i++) {
      LocalizationCore_DW.UnitDelay_DSTATE_f[i] = rtb_states_out_l[i];
    }

    /* End of Update for UnitDelay: '<S38>/Unit Delay' */

    /* Update for UnitDelay: '<S38>/Unit Delay1' */
    memcpy(&LocalizationCore_DW.UnitDelay1_DSTATE_i[0], &rtb_cov_out_e[0], 36U *
           sizeof(real32_T));

    /* Update for UnitDelay: '<S38>/Unit Delay2' incorporates:
     *  MATLAB Function: '<S38>/KALMAN Filter'
     */
    LocalizationCore_DW.UnitDelay2_DSTATE_i = n_beacons;
  }

  /* End of RelationalOperator: '<S36>/Compare' */
  /* End of Outputs for SubSystem: '<S15>/KALMAN Filter' */

  /* Outputs for Enabled SubSystem: '<S15>/Particle Filter' incorporates:
   *  EnablePort: '<S39>/Enable'
   */
  /* RelationalOperator: '<S37>/Compare' incorporates:
   *  Constant: '<S37>/Constant'
   *  MATLAB Function: '<S39>/KALMAN Filter'
   */
  if (rtb_DataTypeConversion2_h == LocalizationCore_P.Constant_Value_lr) {
    /* MATLAB Function 'LocalizationCore/Core/Tracking Module/Particle Filter/KALMAN Filter': '<S43>:1' */
    /* '<S43>:1:10' */
    /* '<S43>:1:11' */
    num[0] = (real32_T)LocalizationCore_B.MultiportSwitch[0] / 13.5F;

    /* MATLAB Function: '<S39>/KALMAN Filter' */
    rtb_MultiportSelector1_o4 = (real32_T)LocalizationCore_B.MultiportSwitch[1] /
      13.5F;
    num[1] = rtb_MultiportSelector1_o4;

    /* MATLAB Function: '<S39>/KALMAN Filter' incorporates:
     *  Constant: '<S39>/ '
     *  UnitDelay: '<S39>/Unit Delay'
     *  UnitDelay: '<S39>/Unit Delay1'
     *  UnitDelay: '<S39>/Unit Delay2'
     */
    /* '<S43>:1:12' */
    num[2] = ((real32_T)LocalizationCore_B.MultiportSwitch[2] - 1.0F) * 4.625F;

    /* '<S43>:1:13' */
    /* '<S43>:1:17' */
    n_beacons = LocalizationCore_DW.UnitDelay2_DSTATE;

    /* '<S43>:1:22' */
    /* '<S43>:1:26' */
    for (b_ia = 0; b_ia < 6; b_ia++) {
      tmp_2 = 0.0F;
      for (h = 0; h < 6; h++) {
        tmp_2 += a_0[6 * h + b_ia] * LocalizationCore_DW.UnitDelay_DSTATE_i[h];
      }

      X_k_k[b_ia] = tmp_2;
    }

    /* '<S43>:1:27' */
    for (b_ia = 0; b_ia < 6; b_ia++) {
      for (h = 0; h < 6; h++) {
        a_1[b_ia + 6 * h] = 0.0F;
        for (i = 0; i < 6; i++) {
          a_1[b_ia + 6 * h] += a_0[6 * i + b_ia] *
            LocalizationCore_DW.UnitDelay1_DSTATE[6 * h + i];
        }
      }
    }

    for (b_ia = 0; b_ia < 6; b_ia++) {
      for (h = 0; h < 6; h++) {
        tmp_2 = 0.0F;
        for (i = 0; i < 6; i++) {
          tmp_2 += a_1[6 * i + b_ia] * b_2[6 * h + i];
        }

        P_k_k[b_ia + 6 * h] = Q_0[6 * h + b_ia] + tmp_2;
      }
    }

    if ((rtb_DataTypeConversion1 != 0.0F) || (rtb_DataTypeConversion != 0.0F)) {
      if (rtb_DataTypeConversion1 != 0.0F) {
        /* '<S43>:1:32' */
        rtb_MultiportSwitch1 = LocalizationCore_P._Value *
          LocalizationCore_P._Value;

        /* '<S43>:1:33' */
        /* '<S43>:1:34' */
        /* '<S43>:1:35' */
        for (b_ia = 0; b_ia < 3; b_ia++) {
          for (h = 0; h < 6; h++) {
            c_a_1[b_ia + 3 * h] = 0.0F;
            for (i = 0; i < 6; i++) {
              c_a_1[b_ia + 3 * h] += (real32_T)c_a_0[3 * i + b_ia] * P_k_k[6 * h
                + i];
            }
          }
        }

        for (b_ia = 0; b_ia < 3; b_ia++) {
          for (h = 0; h < 3; h++) {
            tmp_2 = 0.0F;
            for (i = 0; i < 6; i++) {
              tmp_2 += c_a_1[3 * i + h] * (real32_T)d_b_0[6 * b_ia + i];
            }

            b_A[b_ia + 3 * h] = (real32_T)e_0[3 * b_ia + h] *
              rtb_MultiportSwitch1 + tmp_2;
          }
        }

        for (b_ia = 0; b_ia < 3; b_ia++) {
          for (h = 0; h < 6; h++) {
            b_B[b_ia + 3 * h] = 0.0F;
            for (i = 0; i < 6; i++) {
              b_B[b_ia + 3 * h] += P_k_k[6 * i + h] * (real32_T)d_b_0[6 * b_ia +
                i];
            }
          }
        }

        br = 0;
        b_ia = 1;
        r3 = 2;
        rtb_MultiportSwitch1 = fabsf(b_A[0]);
        rtb_MultiportSelector1_o1 = fabsf(b_A[1]);
        if (rtb_MultiportSelector1_o1 > rtb_MultiportSwitch1) {
          rtb_MultiportSwitch1 = rtb_MultiportSelector1_o1;
          br = 1;
          b_ia = 0;
        }

        if (fabsf(b_A[2]) > rtb_MultiportSwitch1) {
          br = 2;
          b_ia = 1;
          r3 = 0;
        }

        b_A[b_ia] /= b_A[br];
        b_A[r3] /= b_A[br];
        b_A[3 + b_ia] -= b_A[3 + br] * b_A[b_ia];
        b_A[3 + r3] -= b_A[3 + br] * b_A[r3];
        b_A[6 + b_ia] -= b_A[6 + br] * b_A[b_ia];
        b_A[6 + r3] -= b_A[6 + br] * b_A[r3];
        if (fabsf(b_A[3 + r3]) > fabsf(b_A[3 + b_ia])) {
          ar = b_ia;
          b_ia = r3;
          r3 = ar;
        }

        b_A[3 + r3] /= b_A[3 + b_ia];
        b_A[6 + r3] -= b_A[3 + r3] * b_A[6 + b_ia];
        for (ar = 0; ar < 6; ar++) {
          c_a_1[3 * ar] = b_B[3 * ar + br];
          c_a_1[1 + 3 * ar] = b_B[3 * ar + b_ia] - c_a_1[3 * ar] * b_A[b_ia];
          c_a_1[2 + 3 * ar] = (b_B[3 * ar + r3] - c_a_1[3 * ar] * b_A[r3]) -
            c_a_1[3 * ar + 1] * b_A[3 + r3];
          c_a_1[2 + 3 * ar] /= b_A[6 + r3];
          c_a_1[3 * ar] -= c_a_1[3 * ar + 2] * b_A[6 + br];
          c_a_1[1 + 3 * ar] -= c_a_1[3 * ar + 2] * b_A[6 + b_ia];
          c_a_1[1 + 3 * ar] /= b_A[3 + b_ia];
          c_a_1[3 * ar] -= c_a_1[3 * ar + 1] * b_A[3 + br];
          c_a_1[3 * ar] /= b_A[br];
        }

        for (b_ia = 0; b_ia < 3; b_ia++) {
          for (h = 0; h < 6; h++) {
            K[h + 6 * b_ia] = c_a_1[3 * h + b_ia];
          }
        }

        /* '<S43>:1:36' */
        for (b_ia = 0; b_ia < 3; b_ia++) {
          tmp_2 = 0.0F;
          for (h = 0; h < 6; h++) {
            tmp_2 += (real32_T)c_a_0[3 * h + b_ia] * X_k_k[h];
          }

          z_data[b_ia] = num[b_ia] - tmp_2;
        }

        for (b_ia = 0; b_ia < 6; b_ia++) {
          rtb_states_out_l[b_ia] = ((K[b_ia + 6] * z_data[1] + K[b_ia] * z_data
            [0]) + K[b_ia + 12] * z_data[2]) +
            LocalizationCore_DW.UnitDelay_DSTATE_i[b_ia];
        }

        /* '<S43>:1:37' */
        for (b_ia = 0; b_ia < 36; b_ia++) {
          I[b_ia] = 0;
        }

        for (ar = 0; ar < 6; ar++) {
          I[ar + 6 * ar] = 1;
        }

        for (b_ia = 0; b_ia < 6; b_ia++) {
          for (h = 0; h < 6; h++) {
            a_1[b_ia + 6 * h] = (real32_T)I[6 * h + b_ia] - (((real32_T)c_a_0[3 *
              h + 1] * K[b_ia + 6] + (real32_T)c_a_0[3 * h] * K[b_ia]) +
              (real32_T)c_a_0[3 * h + 2] * K[b_ia + 12]);
          }
        }

        for (b_ia = 0; b_ia < 6; b_ia++) {
          for (h = 0; h < 6; h++) {
            rtb_cov_out[b_ia + 6 * h] = 0.0F;
            for (i = 0; i < 6; i++) {
              rtb_cov_out[b_ia + 6 * h] += a_1[6 * i + b_ia] * P_k_k[6 * h + i];
            }
          }
        }

        if (LocalizationCore_DW.UnitDelay2_DSTATE == 1.0F) {
          /* '<S43>:1:38' */
          /* '<S43>:1:39' */
          rtb_states_out_l[0] = num[0];

          /* '<S43>:1:40' */
          rtb_states_out_l[1] = rtb_MultiportSelector1_o4;

          /* '<S43>:1:41' */
          rtb_states_out_l[2] = num[2];

          /* '<S43>:1:42' */
          n_beacons = 0.0F;
        }
      } else {
        /* '<S43>:1:45' */
        rtb_MultiportSwitch1 = LocalizationCore_B.accuracy_n *
          LocalizationCore_B.accuracy_n;

        /* '<S43>:1:46' */
        /* '<S43>:1:47' */
        /* '<S43>:1:48' */
        for (b_ia = 0; b_ia < 2; b_ia++) {
          for (h = 0; h < 6; h++) {
            b_a_1[b_ia + (h << 1)] = 0.0F;
            for (i = 0; i < 6; i++) {
              b_a_1[b_ia + (h << 1)] += (real32_T)b_a_0[(i << 1) + b_ia] *
                P_k_k[6 * h + i];
            }
          }
        }

        for (b_ia = 0; b_ia < 2; b_ia++) {
          for (h = 0; h < 2; h++) {
            tmp_2 = 0.0F;
            for (i = 0; i < 6; i++) {
              tmp_2 += b_a_1[(i << 1) + h] * (real32_T)b_b_0[6 * b_ia + i];
            }

            A2x[b_ia + (h << 1)] = (real32_T)d_0[(b_ia << 1) + h] *
              rtb_MultiportSwitch1 + tmp_2;
          }
        }

        for (b_ia = 0; b_ia < 2; b_ia++) {
          for (h = 0; h < 6; h++) {
            B[b_ia + (h << 1)] = 0.0F;
            for (i = 0; i < 6; i++) {
              B[b_ia + (h << 1)] += P_k_k[6 * i + h] * (real32_T)b_b_0[6 * b_ia
                + i];
            }
          }
        }

        if (fabsf(A2x[1]) > fabsf(A2x[0])) {
          ar = 1;
          br = 0;
        } else {
          ar = 0;
          br = 1;
        }

        rtb_MultiportSelector1_o1 = A2x[br] / A2x[ar];
        rtb_MultiportSwitch1 = A2x[2 + br] - A2x[2 + ar] *
          rtb_MultiportSelector1_o1;
        for (ia = 0; ia < 6; ia++) {
          b_a_1[1 + (ia << 1)] = (B[(ia << 1) + br] - B[(ia << 1) + ar] *
            rtb_MultiportSelector1_o1) / rtb_MultiportSwitch1;
          b_a_1[ia << 1] = (B[(ia << 1) + ar] - b_a_1[(ia << 1) + 1] * A2x[2 +
                            ar]) / A2x[ar];
        }

        for (b_ia = 0; b_ia < 2; b_ia++) {
          for (h = 0; h < 6; h++) {
            K_0[h + 6 * b_ia] = b_a_1[(h << 1) + b_ia];
          }
        }

        /* '<S43>:1:49' */
        for (b_ia = 0; b_ia < 2; b_ia++) {
          tmp_2 = 0.0F;
          for (h = 0; h < 6; h++) {
            tmp_2 += (real32_T)b_a_0[(h << 1) + b_ia] * X_k_k[h];
          }

          temp[b_ia] = LocalizationCore_B.position[b_ia] / 13.5F - tmp_2;
        }

        for (b_ia = 0; b_ia < 6; b_ia++) {
          rtb_states_out_l[b_ia] = (K_0[b_ia + 6] * temp[1] + K_0[b_ia] * temp[0])
            + LocalizationCore_DW.UnitDelay_DSTATE_i[b_ia];
        }

        /* '<S43>:1:50' */
        for (b_ia = 0; b_ia < 36; b_ia++) {
          I[b_ia] = 0;
        }

        for (br = 0; br < 6; br++) {
          I[br + 6 * br] = 1;
        }

        for (b_ia = 0; b_ia < 6; b_ia++) {
          for (h = 0; h < 6; h++) {
            a_1[b_ia + 6 * h] = (real32_T)I[6 * h + b_ia] - ((real32_T)b_a_0[(h <<
              1) + 1] * K_0[b_ia + 6] + (real32_T)b_a_0[h << 1] * K_0[b_ia]);
          }
        }

        for (b_ia = 0; b_ia < 6; b_ia++) {
          for (h = 0; h < 6; h++) {
            rtb_cov_out[b_ia + 6 * h] = 0.0F;
            for (i = 0; i < 6; i++) {
              rtb_cov_out[b_ia + 6 * h] += a_1[6 * i + b_ia] * P_k_k[6 * h + i];
            }
          }
        }
      }
    } else {
      /* '<S43>:1:53' */
      for (i = 0; i < 6; i++) {
        rtb_states_out_l[i] = X_k_k[i];
      }

      /* '<S43>:1:54' */
      memcpy(&rtb_cov_out[0], &P_k_k[0], 36U * sizeof(real32_T));
    }

    /* S-Function (sdspmultiportsel): '<S39>/Multiport Selector13' */
    /* '<S43>:1:59' */
    /* '<S43>:1:60' */
    rtb_MultiportSelector13_o1[0] =
      rtb_states_out_l[LocalizationCore_ConstP.pooled2[0]];
    rtb_MultiportSelector13_o1[1] =
      rtb_states_out_l[LocalizationCore_ConstP.pooled2[1]];
    rtb_MultiportSelector13_o1[2] =
      rtb_states_out_l[LocalizationCore_ConstP.pooled2[2]];
    LocalizationCore_B.MultiportSelector13_o2[0] =
      rtb_states_out_l[LocalizationCore_ConstP.pooled2[3]];
    LocalizationCore_B.MultiportSelector13_o2[1] =
      rtb_states_out_l[LocalizationCore_ConstP.pooled2[4]];
    LocalizationCore_B.MultiportSelector13_o2[2] =
      rtb_states_out_l[LocalizationCore_ConstP.pooled2[5]];

    /* MATLAB Function: '<S39>/MATLAB Function5' */
    LocalizationCor_MATLABFunction5(rtb_MultiportSelector13_o1,
      &LocalizationCore_B.sf_MATLABFunction5_n);

    /* DataTypeConversion: '<S39>/Data Type Conversion' */
    LocalizationCore_B.DataTypeConversion[0] =
      LocalizationCore_B.sf_MATLABFunction5_n.y[0];
    LocalizationCore_B.DataTypeConversion[1] =
      LocalizationCore_B.sf_MATLABFunction5_n.y[1];
    LocalizationCore_B.DataTypeConversion[2] =
      LocalizationCore_B.sf_MATLABFunction5_n.y[2];

    /* MATLAB Function: '<S39>/MATLAB Function4' */
    LocalizationCor_MATLABFunction4(rtb_cov_out,
      &LocalizationCore_B.sf_MATLABFunction4_i);

    /* Update for UnitDelay: '<S39>/Unit Delay' */
    for (i = 0; i < 6; i++) {
      LocalizationCore_DW.UnitDelay_DSTATE_i[i] = rtb_states_out_l[i];
    }

    /* End of Update for UnitDelay: '<S39>/Unit Delay' */

    /* Update for UnitDelay: '<S39>/Unit Delay1' */
    memcpy(&LocalizationCore_DW.UnitDelay1_DSTATE[0], &rtb_cov_out[0], 36U *
           sizeof(real32_T));

    /* Update for UnitDelay: '<S39>/Unit Delay2' incorporates:
     *  MATLAB Function: '<S39>/KALMAN Filter'
     */
    LocalizationCore_DW.UnitDelay2_DSTATE = n_beacons;
  }

  /* End of RelationalOperator: '<S37>/Compare' */
  /* End of Outputs for SubSystem: '<S15>/Particle Filter' */

  /* MultiPortSwitch: '<S15>/Multiport Switch' */
  switch (rtb_DataTypeConversion2_h) {
   case 1:
    rtb_MultiportSwitch[0] = LocalizationCore_B.MultiportSwitch[0];
    rtb_MultiportSwitch[1] = LocalizationCore_B.MultiportSwitch[1];
    rtb_MultiportSwitch[2] = LocalizationCore_B.MultiportSwitch[2];
    break;

   case 2:
    rtb_MultiportSwitch[0] = LocalizationCore_B.DataTypeConversion_n[0];
    rtb_MultiportSwitch[1] = LocalizationCore_B.DataTypeConversion_n[1];
    rtb_MultiportSwitch[2] = LocalizationCore_B.DataTypeConversion_n[2];
    break;

   case 3:
    rtb_MultiportSwitch[0] = LocalizationCore_B.DataTypeConversion[0];
    rtb_MultiportSwitch[1] = LocalizationCore_B.DataTypeConversion[1];
    rtb_MultiportSwitch[2] = LocalizationCore_B.DataTypeConversion[2];
    break;

   default:
    rtb_MultiportSwitch[0] = LocalizationCore_B.MultiportSwitch_c[0];
    rtb_MultiportSwitch[1] = LocalizationCore_B.MultiportSwitch_c[1];
    rtb_MultiportSwitch[2] = LocalizationCore_B.MultiportSwitch_c[2];
    break;
  }

  /* End of MultiPortSwitch: '<S15>/Multiport Switch' */

  /* Outport: '<Root>/Location' */
  LocalizationCore_Y.Location[0] = rtb_MultiportSwitch[0];
  LocalizationCore_Y.Location[1] = rtb_MultiportSwitch[1];
  LocalizationCore_Y.Location[2] = rtb_MultiportSwitch[2];

  /* Outputs for Enabled SubSystem: '<S10>/Geolocation Data' incorporates:
   *  EnablePort: '<S27>/Enable'
   */
  /* Constant: '<S10>/Constant' */
  if (LocalizationCore_P.Constant_Value_h > 0.0) {
    /* Constant: '<S27>/A_matrix' */
    LocalizationCore_B.A_matrix[0] = LocalizationCore_P.A_matrix_Value[0];
    LocalizationCore_B.A_matrix[1] = LocalizationCore_P.A_matrix_Value[1];
    LocalizationCore_B.A_matrix[2] = LocalizationCore_P.A_matrix_Value[2];
    LocalizationCore_B.A_matrix[3] = LocalizationCore_P.A_matrix_Value[3];

    /* Constant: '<S27>/B_Vector' */
    LocalizationCore_B.B_Vector[0] = LocalizationCore_P.B_Vector_Value[0];
    LocalizationCore_B.B_Vector[1] = LocalizationCore_P.B_Vector_Value[1];
  }

  /* End of Constant: '<S10>/Constant' */
  /* End of Outputs for SubSystem: '<S10>/Geolocation Data' */

  /* Outport: '<Root>/GeoLocation' incorporates:
   *  MATLAB Function: '<S10>/MATLAB Function1'
   */
  /* MATLAB Function 'LocalizationCore/Core/Geolocation Calculator/MATLAB Function1': '<S28>:1' */
  /* '<S28>:1:2' */
  /* '<S28>:1:3' */
  /* '<S28>:1:4' */
  LocalizationCore_Y.GeoLocation[0] = (LocalizationCore_B.A_matrix[0] *
    (real32_T)rtb_MultiportSwitch[0] + LocalizationCore_B.A_matrix[2] *
    (real32_T)rtb_MultiportSwitch[1]) + LocalizationCore_B.B_Vector[0];
  LocalizationCore_Y.GeoLocation[1] = (LocalizationCore_B.A_matrix[1] *
    (real32_T)rtb_MultiportSwitch[0] + LocalizationCore_B.A_matrix[3] *
    (real32_T)rtb_MultiportSwitch[1]) + LocalizationCore_B.B_Vector[1];
  LocalizationCore_Y.GeoLocation[2] = (real32_T)rtb_MultiportSwitch[2];

  /* MultiPortSwitch: '<S15>/Multiport Switch1' */
  switch (rtb_DataTypeConversion2_h) {
   case 1:
    /* Outport: '<Root>/Accuracy' */
    LocalizationCore_Y.Accuracy = LocalizationCore_B.MultiportSwitch1;

    /* Outport: '<Root>/Speed' incorporates:
     *  Constant: '<S15>/Constant'
     *  MultiPortSwitch: '<S15>/Multiport Switch2'
     *  SignalConversion: '<S15>/ConcatBufferAtMatrix ConcatenateIn2'
     */
    LocalizationCore_Y.Speed[0] = LocalizationCore_P.Constant_Value_jk[0];
    LocalizationCore_Y.Speed[1] = LocalizationCore_P.Constant_Value_jk[1];
    LocalizationCore_Y.Speed[2] = rtb_Gain_e;
    break;

   case 2:
    /* Outport: '<Root>/Accuracy' */
    LocalizationCore_Y.Accuracy = LocalizationCore_B.sf_MATLABFunction4.y;

    /* Outport: '<Root>/Speed' incorporates:
     *  MultiPortSwitch: '<S15>/Multiport Switch2'
     */
    LocalizationCore_Y.Speed[0] = LocalizationCore_B.MultiportSelector13_o2_a[0];
    LocalizationCore_Y.Speed[1] = LocalizationCore_B.MultiportSelector13_o2_a[1];
    LocalizationCore_Y.Speed[2] = LocalizationCore_B.MultiportSelector13_o2_a[2];
    break;

   case 3:
    /* Outport: '<Root>/Accuracy' */
    LocalizationCore_Y.Accuracy = LocalizationCore_B.sf_MATLABFunction4_i.y;

    /* Outport: '<Root>/Speed' incorporates:
     *  MultiPortSwitch: '<S15>/Multiport Switch2'
     */
    LocalizationCore_Y.Speed[0] = LocalizationCore_B.MultiportSelector13_o2[0];
    LocalizationCore_Y.Speed[1] = LocalizationCore_B.MultiportSelector13_o2[1];
    LocalizationCore_Y.Speed[2] = LocalizationCore_B.MultiportSelector13_o2[2];
    break;

   default:
    /* Outport: '<Root>/Accuracy' */
    LocalizationCore_Y.Accuracy = LocalizationCore_B.MultiportSwitch1_j;

    /* Outport: '<Root>/Speed' incorporates:
     *  Constant: '<S15>/Constant'
     *  MultiPortSwitch: '<S15>/Multiport Switch2'
     *  SignalConversion: '<S15>/ConcatBufferAtMatrix ConcatenateIn2'
     */
    LocalizationCore_Y.Speed[0] = LocalizationCore_P.Constant_Value_jk[0];
    LocalizationCore_Y.Speed[1] = LocalizationCore_P.Constant_Value_jk[1];
    LocalizationCore_Y.Speed[2] = rtb_Gain_e;
    break;
  }

  /* End of MultiPortSwitch: '<S15>/Multiport Switch1' */

  /* Outport: '<Root>/Steps' */
  LocalizationCore_Y.Steps = rtb_Gain;

  /* S-Function (sdspmultiportsel): '<S14>/Select Z-Axis' */
  LocalizationCore_B.SelectZAxis = x0_n[LocalizationCore_ConstP.pooled4];

  /* S-Function (sdspfilter2): '<S14>/Digital Filter' */
  rtb_MultiportSwitch1 = LocalizationCore_B.SelectZAxis *
    LocalizationCore_P.DigitalFilter_RTP1COEFF_f[0];
  ar = 1;
  for (b_ia = LocalizationCore_DW.DigitalFilter_CIRCBUFFIDX_n; b_ia < 4; b_ia++)
  {
    rtb_MultiportSwitch1 += LocalizationCore_DW.DigitalFilter_FILT_STATES_b[b_ia]
      * LocalizationCore_P.DigitalFilter_RTP1COEFF_f[ar];
    ar++;
  }

  for (b_ia = 0; b_ia < LocalizationCore_DW.DigitalFilter_CIRCBUFFIDX_n; b_ia++)
  {
    rtb_MultiportSwitch1 += LocalizationCore_DW.DigitalFilter_FILT_STATES_b[b_ia]
      * LocalizationCore_P.DigitalFilter_RTP1COEFF_f[ar];
    ar++;
  }

  i = LocalizationCore_DW.DigitalFilter_CIRCBUFFIDX_n - 1;
  if (i < 0) {
    i = 3;
  }

  LocalizationCore_DW.DigitalFilter_FILT_STATES_b[i] =
    LocalizationCore_B.SelectZAxis;
  LocalizationCore_B.DigitalFilter = rtb_MultiportSwitch1;
  LocalizationCore_DW.DigitalFilter_CIRCBUFFIDX_n = i;

  /* End of S-Function (sdspfilter2): '<S14>/Digital Filter' */

  /* MATLAB Function: '<S14>/MATLAB Function2' */
  /* MATLAB Function 'LocalizationCore/Core/Step Counter/MATLAB Function2': '<S33>:1' */
  if (fabsf(LocalizationCore_B.DigitalFilter) < 1.4F) {
    /* '<S33>:1:2' */
    /* '<S33>:1:3' */
    rtb_MultiportSwitch1 = 0.0F;
  } else {
    /* '<S33>:1:5' */
    rtb_MultiportSwitch1 = LocalizationCore_B.DigitalFilter;
  }

  /* End of MATLAB Function: '<S14>/MATLAB Function2' */

  /* MATLAB Function: '<S14>/MATLAB Function3' incorporates:
   *  UnitDelay: '<S14>/Unit Delay'
   */
  /* MATLAB Function 'LocalizationCore/Core/Step Counter/MATLAB Function3': '<S34>:1' */
  /* '<S34>:1:2' */
  rtb_MultiportSelector1_o1 = 0.0F;
  if ((rtb_MultiportSwitch1 > 0.0F) && (LocalizationCore_DW.UnitDelay_DSTATE <
       0.0F)) {
    /* '<S34>:1:3' */
    /* '<S34>:1:4' */
    rtb_MultiportSelector1_o1 = 1.0F;
  }

  if ((rtb_MultiportSwitch1 < 0.0F) && (LocalizationCore_DW.UnitDelay_DSTATE >
       0.0F)) {
    /* '<S34>:1:6' */
    /* '<S34>:1:7' */
    rtb_MultiportSelector1_o1 = 1.0F;
  }

  if (rtb_MultiportSwitch1 > 0.0F) {
    /* Update for UnitDelay: '<S14>/Unit Delay' */
    /* '<S34>:1:10' */
    /* '<S34>:1:11' */
    LocalizationCore_DW.UnitDelay_DSTATE = 1.0F;
  } else if (rtb_MultiportSwitch1 < 0.0F) {
    /* Update for UnitDelay: '<S14>/Unit Delay' */
    /* '<S34>:1:12' */
    /* '<S34>:1:13' */
    LocalizationCore_DW.UnitDelay_DSTATE = -1.0F;
  } else {
    /* '<S34>:1:15' */
  }

  /* End of MATLAB Function: '<S14>/MATLAB Function3' */

  /* Update for DiscreteIntegrator: '<S14>/Discrete-Time Integrator' */
  LocalizationCore_DW.DiscreteTimeIntegrator_DSTATE +=
    LocalizationCore_P.DiscreteTimeIntegrator_gainval *
    rtb_MultiportSelector1_o1;
}

/* Model initialize function */
void LocalizationCore_initialize(void)
{
  /* Registration code */

  /* initialize error status */
  rtmSetErrorStatus(LocalizationCore_M, (NULL));

  /* block I/O */
  (void) memset(((void *) &LocalizationCore_B), 0,
                sizeof(B_LocalizationCore_T));

  /* states (dwork) */
  (void) memset((void *)&LocalizationCore_DW, 0,
                sizeof(DW_LocalizationCore_T));

  /* external inputs */
  (void) memset((void *)&LocalizationCore_U, 0,
                sizeof(ExtU_LocalizationCore_T));

  /* external outputs */
  (void) memset((void *)&LocalizationCore_Y, 0,
                sizeof(ExtY_LocalizationCore_T));

  {
    int32_T i;

    /* Start for Enabled SubSystem: '<S2>/Wifi Localizer' */
    /* Start for Enabled SubSystem: '<S16>/RadioMap' */
    /* Start for Constant: '<S49>/AP_LIST' */
    memcpy(&LocalizationCore_B.AP_LIST[0], &LocalizationCore_P.AP_LIST_Value[0],
           34816U * sizeof(uint8_T));

    /* Start for Constant: '<S49>/MEAN_RSS' */
    memcpy(&LocalizationCore_B.MEAN_RSS[0], &LocalizationCore_P.MEAN_RSS_Value[0],
           sizeof(real32_T) << 22U);

    /* Start for Constant: '<S49>/N_AP' */
    LocalizationCore_B.N_AP = LocalizationCore_P.N_AP_Value;

    /* Start for Constant: '<S49>/N_POINTS' */
    LocalizationCore_B.N_POINTS = LocalizationCore_P.N_POINTS_Value;

    /* Start for Constant: '<S49>/POINT_LIST' */
    memcpy(&LocalizationCore_B.POINT_LIST[0],
           &LocalizationCore_P.POINT_LIST_Value[0], 6144U * sizeof(real32_T));

    /* End of Start for SubSystem: '<S16>/RadioMap' */
    /* End of Start for SubSystem: '<S2>/Wifi Localizer' */

    /* InitializeConditions for Enabled SubSystem: '<S15>/KALMAN Filter' */
    /* InitializeConditions for UnitDelay: '<S38>/Unit Delay' */
    for (i = 0; i < 6; i++) {
      LocalizationCore_DW.UnitDelay_DSTATE_f[i] =
        LocalizationCore_P.UnitDelay_InitialCondition[i];
    }

    /* End of InitializeConditions for UnitDelay: '<S38>/Unit Delay' */

    /* InitializeConditions for UnitDelay: '<S38>/Unit Delay1' */
    memcpy(&LocalizationCore_DW.UnitDelay1_DSTATE_i[0],
           &LocalizationCore_P.UnitDelay1_InitialCondition[0], 36U * sizeof
           (real32_T));

    /* InitializeConditions for UnitDelay: '<S38>/Unit Delay2' */
    LocalizationCore_DW.UnitDelay2_DSTATE_i =
      LocalizationCore_P.UnitDelay2_InitialCondition;

    /* End of InitializeConditions for SubSystem: '<S15>/KALMAN Filter' */

    /* InitializeConditions for Enabled SubSystem: '<S15>/Particle Filter' */
    /* InitializeConditions for UnitDelay: '<S39>/Unit Delay' */
    for (i = 0; i < 6; i++) {
      LocalizationCore_DW.UnitDelay_DSTATE_i[i] =
        LocalizationCore_P.UnitDelay_InitialCondition_i[i];
    }

    /* End of InitializeConditions for UnitDelay: '<S39>/Unit Delay' */

    /* InitializeConditions for UnitDelay: '<S39>/Unit Delay1' */
    memcpy(&LocalizationCore_DW.UnitDelay1_DSTATE[0],
           &LocalizationCore_P.UnitDelay1_InitialCondition_h[0], 36U * sizeof
           (real32_T));

    /* InitializeConditions for UnitDelay: '<S39>/Unit Delay2' */
    LocalizationCore_DW.UnitDelay2_DSTATE =
      LocalizationCore_P.UnitDelay2_InitialCondition_k;

    /* End of InitializeConditions for SubSystem: '<S15>/Particle Filter' */

    /* Start for Enabled SubSystem: '<S10>/Geolocation Data' */
    /* Start for Constant: '<S27>/A_matrix' */
    LocalizationCore_B.A_matrix[0] = LocalizationCore_P.A_matrix_Value[0];
    LocalizationCore_B.A_matrix[1] = LocalizationCore_P.A_matrix_Value[1];
    LocalizationCore_B.A_matrix[2] = LocalizationCore_P.A_matrix_Value[2];
    LocalizationCore_B.A_matrix[3] = LocalizationCore_P.A_matrix_Value[3];

    /* Start for Constant: '<S27>/B_Vector' */
    LocalizationCore_B.B_Vector[0] = LocalizationCore_P.B_Vector_Value[0];
    LocalizationCore_B.B_Vector[1] = LocalizationCore_P.B_Vector_Value[1];

    /* End of Start for SubSystem: '<S10>/Geolocation Data' */
  }

  /* InitializeConditions for DiscreteIntegrator: '<S14>/Discrete-Time Integrator' */
  LocalizationCore_DW.DiscreteTimeIntegrator_DSTATE =
    LocalizationCore_P.DiscreteTimeIntegrator_IC;

  /* InitializeConditions for UnitDelay: '<S14>/Unit Delay' */
  LocalizationCore_DW.UnitDelay_DSTATE =
    LocalizationCore_P.UnitDelay_InitialCondition_g;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
