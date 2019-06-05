/*
 * File: Echo_cancel.h
 *
 * Code generated for Simulink model 'Echo_cancel'.
 *
 * Model version                  : 1.40
 * Simulink Coder version         : 9.1 (R2019a) 23-Nov-2018
 * C/C++ source code generated on : Sun Jun  2 18:48:27 2019
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#ifndef RTW_HEADER_Echo_cancel_h_
#define RTW_HEADER_Echo_cancel_h_
#ifndef Echo_cancel_COMMON_INCLUDES_
# define Echo_cancel_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* Echo_cancel_COMMON_INCLUDES_ */

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

/* Forward declaration for rtModel */
typedef struct tag_RTMeC RT_MODELeC;

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  real32_T LMSFilter1_IN_BUFFER_DWORKeC[3];/* '<Root>/LMS Filter1' */
  real32_T LMSFilter1_WGT_IC_DWORKeC[3];/* '<Root>/LMS Filter1' */
  uint32_T LMSFilter1_BUFF_IDX_DWORKeC;/* '<Root>/LMS Filter1' */
} D_WorkeC;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real32_T BLE_receiveeC;              /* '<Root>/BLE_receive' */
  real32_T MIC_dataeC;                 /* '<Root>/MIC_data' */
  boolean_T EnableeC;                  /* '<Root>/Enable' */
  boolean_T ReseteC;                   /* '<Root>/Reset' */
} ExternalInputseC;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real32_T OutputeC;                   /* '<Root>/Output' */
  real32_T debug_erroreC;              /* '<Root>/debug_error' */
  real32_T debug_coeffseC[3];          /* '<Root>/debug_coeffs' */
} ExternalOutputseC;

/* Real-time Model Data Structure */
struct tag_RTMeC {
  const char_T * volatile errorStatus;
};

/* Block signals and states (default storage) */
extern D_WorkeC rtDWorkeC;

/* External inputs (root inport signals with default storage) */
extern ExternalInputseC rtUeC;

/* External outputs (root outports fed by signals with default storage) */
extern ExternalOutputseC rtYeC;

/* Model entry point functions */
extern void Echo_cancel_initialize(void);
extern void Echo_cancel_step(void);

/* Real-time Model object */
extern RT_MODELeC *const rtMeC;

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
 * '<Root>' : 'Echo_cancel'
 */
#endif                                 /* RTW_HEADER_Echo_cancel_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
