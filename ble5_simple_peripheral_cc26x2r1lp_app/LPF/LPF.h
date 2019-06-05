/*
 * File: LPF.h
 *
 * Code generated for Simulink model 'LPF'.
 *
 * Model version                  : 1.97
 * Simulink Coder version         : 9.1 (R2019a) 23-Nov-2018
 * C/C++ source code generated on : Sun Jun  2 18:54:10 2019
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives:
 *    1. RAM efficiency
 *    2. Execution efficiency
 * Validation result: Not run
 */

#ifndef RTW_HEADER_LPF_h_
#define RTW_HEADER_LPF_h_
#include <math.h>
#ifndef LPF_COMMON_INCLUDES_
# define LPF_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* LPF_COMMON_INCLUDES_ */

/* Macros for accessing real-time model data structure */

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  real32_T Filter_states[15];          /* '<Root>/Filter' */
  int32_T Filter_circBuf;              /* '<Root>/Filter' */
} DW;

/* Constant parameters (default storage) */
typedef struct {
  /* Computed Parameter: Filter_Coefficients
   * Referenced by: '<Root>/Filter'
   */
  real32_T Filter_Coefficients[16];
} ConstP;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real32_T In1;                        /* '<Root>/In1' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  int16_T Out1;                        /* '<Root>/Out1' */
} ExtY;

/* Block signals and states (default storage) */
extern DW rtDW;

/* External inputs (root inport signals with default storage) */
extern ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY rtY;

/* Constant parameters (default storage) */
extern const ConstP rtConstP;

/* Model entry point functions */
extern void LPF_initialize(void);
extern void LPF_step(void);

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
 * '<Root>' : 'LPF'
 */
#endif                                 /* RTW_HEADER_LPF_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
