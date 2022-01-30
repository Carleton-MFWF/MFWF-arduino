/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: MFWF_control.h
 *
 * Code generated for Simulink model 'MFWF_control'.
 *
 * Model version                  : 8.23
 * Simulink Coder version         : 9.5 (R2021a) 14-Nov-2020
 * C/C++ source code generated on : Wed Jan 26 21:55:59 2022
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: 32-bit Generic
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_MFWF_control_h_
#define RTW_HEADER_MFWF_control_h_
#ifndef MFWF_control_COMMON_INCLUDES_
#define MFWF_control_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* MFWF_control_COMMON_INCLUDES_ */

#include "MFWF_control_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block signals (default storage) */
typedef struct {
  real_T FilterCoefficient;            /* '<S38>/Filter Coefficient' */
  real_T IntegralGain;                 /* '<S81>/Integral Gain' */
  real_T FilterCoefficient_b;          /* '<S87>/Filter Coefficient' */
} BlockIO_MFWF_control;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T Filter_DSTATE;                /* '<S30>/Filter' */
  real_T Filter_DSTATE_d;              /* '<S79>/Filter' */
  real_T Integrator_DSTATE;            /* '<S84>/Integrator' */
} D_Work_MFWF_control;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T C_in_sensor_z;                /* '<Root>/B_-1_-1' */
  real_T C_in_sensor_y;                /* '<Root>/B_-1_-1' */
  real_T C_in_sensor_phi;              /* '<Root>/B_-1_-1' */
  real_T C_in_control_z;               /* '<Root>/B_-1_-1' */
  real_T C_in_control_phi;             /* '<Root>/B_-1_-1' */
} ExternalInputs_MFWF_control;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T C_out_flapping_freq;          /* '<Root>/C_out_flapping_freq' */
  real_T C_out_pitch;                  /* '<Root>/C_out_pitch' */
  real_T C_out_roll;                   /* '<Root>/C_out_roll' */
  real_T C_out_yaw;                    /* '<Root>/C_out_yaw' */
} ExternalOutputs_MFWF_control;

/* Real-time Model Data Structure */
struct tag_RTM_MFWF_control {
  const char_T * volatile errorStatus;
};

/* Block signals (default storage) */
extern BlockIO_MFWF_control MFWF_control_B;

/* Block states (default storage) */
extern D_Work_MFWF_control MFWF_control_DWork;

/* External inputs (root inport signals with default storage) */
extern ExternalInputs_MFWF_control MFWF_control_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExternalOutputs_MFWF_control MFWF_control_Y;

/* Model entry point functions */
extern void MFWF_control_initialize(void);
extern void MFWF_control_output(void);
extern void MFWF_control_update(void);
extern void MFWF_control_terminate(void);

/* Real-time Model object */
extern RT_MODEL_MFWF_control *const MFWF_control_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'MFWF_control'
 * '<S1>'   : 'MFWF_control/Inner Loop'
 * '<S2>'   : 'MFWF_control/Outer Loop'
 * '<S3>'   : 'MFWF_control/Inner Loop/PID Controller'
 * '<S4>'   : 'MFWF_control/Inner Loop/PID Controller/Anti-windup'
 * '<S5>'   : 'MFWF_control/Inner Loop/PID Controller/D Gain'
 * '<S6>'   : 'MFWF_control/Inner Loop/PID Controller/Filter'
 * '<S7>'   : 'MFWF_control/Inner Loop/PID Controller/Filter ICs'
 * '<S8>'   : 'MFWF_control/Inner Loop/PID Controller/I Gain'
 * '<S9>'   : 'MFWF_control/Inner Loop/PID Controller/Ideal P Gain'
 * '<S10>'  : 'MFWF_control/Inner Loop/PID Controller/Ideal P Gain Fdbk'
 * '<S11>'  : 'MFWF_control/Inner Loop/PID Controller/Integrator'
 * '<S12>'  : 'MFWF_control/Inner Loop/PID Controller/Integrator ICs'
 * '<S13>'  : 'MFWF_control/Inner Loop/PID Controller/N Copy'
 * '<S14>'  : 'MFWF_control/Inner Loop/PID Controller/N Gain'
 * '<S15>'  : 'MFWF_control/Inner Loop/PID Controller/P Copy'
 * '<S16>'  : 'MFWF_control/Inner Loop/PID Controller/Parallel P Gain'
 * '<S17>'  : 'MFWF_control/Inner Loop/PID Controller/Reset Signal'
 * '<S18>'  : 'MFWF_control/Inner Loop/PID Controller/Saturation'
 * '<S19>'  : 'MFWF_control/Inner Loop/PID Controller/Saturation Fdbk'
 * '<S20>'  : 'MFWF_control/Inner Loop/PID Controller/Sum'
 * '<S21>'  : 'MFWF_control/Inner Loop/PID Controller/Sum Fdbk'
 * '<S22>'  : 'MFWF_control/Inner Loop/PID Controller/Tracking Mode'
 * '<S23>'  : 'MFWF_control/Inner Loop/PID Controller/Tracking Mode Sum'
 * '<S24>'  : 'MFWF_control/Inner Loop/PID Controller/Tsamp - Integral'
 * '<S25>'  : 'MFWF_control/Inner Loop/PID Controller/Tsamp - Ngain'
 * '<S26>'  : 'MFWF_control/Inner Loop/PID Controller/postSat Signal'
 * '<S27>'  : 'MFWF_control/Inner Loop/PID Controller/preSat Signal'
 * '<S28>'  : 'MFWF_control/Inner Loop/PID Controller/Anti-windup/Disabled'
 * '<S29>'  : 'MFWF_control/Inner Loop/PID Controller/D Gain/Internal Parameters'
 * '<S30>'  : 'MFWF_control/Inner Loop/PID Controller/Filter/Disc. Forward Euler Filter'
 * '<S31>'  : 'MFWF_control/Inner Loop/PID Controller/Filter ICs/Internal IC - Filter'
 * '<S32>'  : 'MFWF_control/Inner Loop/PID Controller/I Gain/Disabled'
 * '<S33>'  : 'MFWF_control/Inner Loop/PID Controller/Ideal P Gain/Passthrough'
 * '<S34>'  : 'MFWF_control/Inner Loop/PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S35>'  : 'MFWF_control/Inner Loop/PID Controller/Integrator/Disabled'
 * '<S36>'  : 'MFWF_control/Inner Loop/PID Controller/Integrator ICs/Disabled'
 * '<S37>'  : 'MFWF_control/Inner Loop/PID Controller/N Copy/Disabled'
 * '<S38>'  : 'MFWF_control/Inner Loop/PID Controller/N Gain/Internal Parameters'
 * '<S39>'  : 'MFWF_control/Inner Loop/PID Controller/P Copy/Disabled'
 * '<S40>'  : 'MFWF_control/Inner Loop/PID Controller/Parallel P Gain/Internal Parameters'
 * '<S41>'  : 'MFWF_control/Inner Loop/PID Controller/Reset Signal/Disabled'
 * '<S42>'  : 'MFWF_control/Inner Loop/PID Controller/Saturation/Enabled'
 * '<S43>'  : 'MFWF_control/Inner Loop/PID Controller/Saturation Fdbk/Disabled'
 * '<S44>'  : 'MFWF_control/Inner Loop/PID Controller/Sum/Sum_PD'
 * '<S45>'  : 'MFWF_control/Inner Loop/PID Controller/Sum Fdbk/Disabled'
 * '<S46>'  : 'MFWF_control/Inner Loop/PID Controller/Tracking Mode/Disabled'
 * '<S47>'  : 'MFWF_control/Inner Loop/PID Controller/Tracking Mode Sum/Passthrough'
 * '<S48>'  : 'MFWF_control/Inner Loop/PID Controller/Tsamp - Integral/Disabled wSignal Specification'
 * '<S49>'  : 'MFWF_control/Inner Loop/PID Controller/Tsamp - Ngain/Passthrough'
 * '<S50>'  : 'MFWF_control/Inner Loop/PID Controller/postSat Signal/Forward_Path'
 * '<S51>'  : 'MFWF_control/Inner Loop/PID Controller/preSat Signal/Forward_Path'
 * '<S52>'  : 'MFWF_control/Outer Loop/Altitude Controller'
 * '<S53>'  : 'MFWF_control/Outer Loop/Altitude Controller/Anti-windup'
 * '<S54>'  : 'MFWF_control/Outer Loop/Altitude Controller/D Gain'
 * '<S55>'  : 'MFWF_control/Outer Loop/Altitude Controller/Filter'
 * '<S56>'  : 'MFWF_control/Outer Loop/Altitude Controller/Filter ICs'
 * '<S57>'  : 'MFWF_control/Outer Loop/Altitude Controller/I Gain'
 * '<S58>'  : 'MFWF_control/Outer Loop/Altitude Controller/Ideal P Gain'
 * '<S59>'  : 'MFWF_control/Outer Loop/Altitude Controller/Ideal P Gain Fdbk'
 * '<S60>'  : 'MFWF_control/Outer Loop/Altitude Controller/Integrator'
 * '<S61>'  : 'MFWF_control/Outer Loop/Altitude Controller/Integrator ICs'
 * '<S62>'  : 'MFWF_control/Outer Loop/Altitude Controller/N Copy'
 * '<S63>'  : 'MFWF_control/Outer Loop/Altitude Controller/N Gain'
 * '<S64>'  : 'MFWF_control/Outer Loop/Altitude Controller/P Copy'
 * '<S65>'  : 'MFWF_control/Outer Loop/Altitude Controller/Parallel P Gain'
 * '<S66>'  : 'MFWF_control/Outer Loop/Altitude Controller/Reset Signal'
 * '<S67>'  : 'MFWF_control/Outer Loop/Altitude Controller/Saturation'
 * '<S68>'  : 'MFWF_control/Outer Loop/Altitude Controller/Saturation Fdbk'
 * '<S69>'  : 'MFWF_control/Outer Loop/Altitude Controller/Sum'
 * '<S70>'  : 'MFWF_control/Outer Loop/Altitude Controller/Sum Fdbk'
 * '<S71>'  : 'MFWF_control/Outer Loop/Altitude Controller/Tracking Mode'
 * '<S72>'  : 'MFWF_control/Outer Loop/Altitude Controller/Tracking Mode Sum'
 * '<S73>'  : 'MFWF_control/Outer Loop/Altitude Controller/Tsamp - Integral'
 * '<S74>'  : 'MFWF_control/Outer Loop/Altitude Controller/Tsamp - Ngain'
 * '<S75>'  : 'MFWF_control/Outer Loop/Altitude Controller/postSat Signal'
 * '<S76>'  : 'MFWF_control/Outer Loop/Altitude Controller/preSat Signal'
 * '<S77>'  : 'MFWF_control/Outer Loop/Altitude Controller/Anti-windup/Passthrough'
 * '<S78>'  : 'MFWF_control/Outer Loop/Altitude Controller/D Gain/Internal Parameters'
 * '<S79>'  : 'MFWF_control/Outer Loop/Altitude Controller/Filter/Disc. Forward Euler Filter'
 * '<S80>'  : 'MFWF_control/Outer Loop/Altitude Controller/Filter ICs/Internal IC - Filter'
 * '<S81>'  : 'MFWF_control/Outer Loop/Altitude Controller/I Gain/Internal Parameters'
 * '<S82>'  : 'MFWF_control/Outer Loop/Altitude Controller/Ideal P Gain/Passthrough'
 * '<S83>'  : 'MFWF_control/Outer Loop/Altitude Controller/Ideal P Gain Fdbk/Disabled'
 * '<S84>'  : 'MFWF_control/Outer Loop/Altitude Controller/Integrator/Discrete'
 * '<S85>'  : 'MFWF_control/Outer Loop/Altitude Controller/Integrator ICs/Internal IC'
 * '<S86>'  : 'MFWF_control/Outer Loop/Altitude Controller/N Copy/Disabled'
 * '<S87>'  : 'MFWF_control/Outer Loop/Altitude Controller/N Gain/Internal Parameters'
 * '<S88>'  : 'MFWF_control/Outer Loop/Altitude Controller/P Copy/Disabled'
 * '<S89>'  : 'MFWF_control/Outer Loop/Altitude Controller/Parallel P Gain/Internal Parameters'
 * '<S90>'  : 'MFWF_control/Outer Loop/Altitude Controller/Reset Signal/Disabled'
 * '<S91>'  : 'MFWF_control/Outer Loop/Altitude Controller/Saturation/Passthrough'
 * '<S92>'  : 'MFWF_control/Outer Loop/Altitude Controller/Saturation Fdbk/Disabled'
 * '<S93>'  : 'MFWF_control/Outer Loop/Altitude Controller/Sum/Sum_PID'
 * '<S94>'  : 'MFWF_control/Outer Loop/Altitude Controller/Sum Fdbk/Disabled'
 * '<S95>'  : 'MFWF_control/Outer Loop/Altitude Controller/Tracking Mode/Disabled'
 * '<S96>'  : 'MFWF_control/Outer Loop/Altitude Controller/Tracking Mode Sum/Passthrough'
 * '<S97>'  : 'MFWF_control/Outer Loop/Altitude Controller/Tsamp - Integral/Passthrough'
 * '<S98>'  : 'MFWF_control/Outer Loop/Altitude Controller/Tsamp - Ngain/Passthrough'
 * '<S99>'  : 'MFWF_control/Outer Loop/Altitude Controller/postSat Signal/Forward_Path'
 * '<S100>' : 'MFWF_control/Outer Loop/Altitude Controller/preSat Signal/Forward_Path'
 */
#endif                                 /* RTW_HEADER_MFWF_control_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
