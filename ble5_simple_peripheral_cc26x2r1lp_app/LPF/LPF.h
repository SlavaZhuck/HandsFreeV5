/*
 * File: LPF.h
 *
 * Code generated for Simulink model 'LPF'.
 *
 * Model version                  : 1.100
 * Simulink Coder version         : 9.1 (R2019a) 23-Nov-2018
 * C/C++ source code generated on : Sat Aug 17 17:31:55 2019
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
  real32_T DigitalFilter_states[9];    /* '<S1>/Digital Filter' */
  real32_T DigitalFilter_states_g[20]; /* '<S2>/Digital Filter' */
  real32_T DigitalFilter_states_n[20]; /* '<S3>/Digital Filter' */
  real32_T DigitalFilter_states_i[20]; /* '<S4>/Digital Filter' */
} DW;

/* Constant parameters (default storage) */
typedef struct {
  /* Computed Parameter: DigitalFilter_Coefficients
   * Referenced by: '<S1>/Digital Filter'
   */
  real32_T DigitalFilter_Coefficients[10];

  /* Computed Parameter: DigitalFilter_Coefficients_f
   * Referenced by: '<S2>/Digital Filter'
   */
  real32_T DigitalFilter_Coefficients_f[21];

  /* Computed Parameter: DigitalFilter_Coefficients_e
   * Referenced by: '<S3>/Digital Filter'
   */
  real32_T DigitalFilter_Coefficients_e[21];

  /* Computed Parameter: DigitalFilter_Coefficients_d
   * Referenced by: '<S4>/Digital Filter'
   */
  real32_T DigitalFilter_Coefficients_d[21];
} ConstP;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real32_T In1;                        /* '<Root>/In1' */
  uint8_T switch_a;                   /* '<Root>/switch' */
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
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S5>/Check Signal Attributes' : Unused code path elimination
 * Block '<S6>/Check Signal Attributes' : Unused code path elimination
 * Block '<S7>/Check Signal Attributes' : Unused code path elimination
 * Block '<S8>/Check Signal Attributes' : Unused code path elimination
 */

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
 * '<S1>'   : 'LPF/Digital Filter Design'
 * '<S2>'   : 'LPF/Digital Filter Design1'
 * '<S3>'   : 'LPF/Digital Filter Design2'
 * '<S4>'   : 'LPF/Digital Filter Design3'
 * '<S5>'   : 'LPF/Digital Filter Design/Check Signal Attributes'
 * '<S6>'   : 'LPF/Digital Filter Design1/Check Signal Attributes'
 * '<S7>'   : 'LPF/Digital Filter Design2/Check Signal Attributes'
 * '<S8>'   : 'LPF/Digital Filter Design3/Check Signal Attributes'
 */
#endif                                 /* RTW_HEADER_LPF_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
