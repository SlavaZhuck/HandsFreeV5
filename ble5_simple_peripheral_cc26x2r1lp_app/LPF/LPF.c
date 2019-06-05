/*
 * File: LPF.c
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

#include "LPF.h"

/* Block signals and states (default storage) */
DW rtDW;

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Model step function */
void LPF_step(void)
{
  int32_T dataIdx;
  int32_T cff;
  int32_T idxN;
  real32_T tapsum;
  real32_T acc;
  int32_T j;

  /* DiscreteFir: '<Root>/Filter' incorporates:
   *  Inport: '<Root>/In1'
   */
  idxN = rtDW.Filter_circBuf - 1;
  dataIdx = idxN + 15;
  if (idxN + 15 >= 15) {
    dataIdx = idxN;
  }

  acc = (rtU.In1 + rtDW.Filter_states[dataIdx]) * 0.0103798052F;
  cff = 1;
  for (j = 0; j < 7; j++) {
    dataIdx = (j + idxN) + 1;
    if (dataIdx >= 15) {
      dataIdx -= 15;
    }

    tapsum = rtDW.Filter_states[dataIdx];
    dataIdx = (dataIdx - ((j + 1) << 1)) + 15;
    if (dataIdx >= 15) {
      dataIdx -= 15;
    }

    tapsum += rtDW.Filter_states[dataIdx];
    acc += tapsum * rtConstP.Filter_Coefficients[cff];
    cff++;
  }

  if (idxN < 0) {
    idxN = 14;
  }

  rtDW.Filter_states[idxN] = rtU.In1;
  rtDW.Filter_circBuf = idxN;

  /* End of DiscreteFir: '<Root>/Filter' */

  /* Outport: '<Root>/Out1' incorporates:
   *  DataTypeConversion: '<Root>/Data Type Conversion'
   */
  rtY.Out1 = (int16_T)floorf(acc);
}

/* Model initialize function */
void LPF_initialize(void)
{
  /* (no initialization code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
