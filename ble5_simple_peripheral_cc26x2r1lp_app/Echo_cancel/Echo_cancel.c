/*
 * File: Echo_cancel.c
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

#include "Echo_cancel.h"

/* Block signals and states (default storage) */
D_WorkeC rtDWorkeC;

/* External inputs (root inport signals with default storage) */
ExternalInputseC rtUeC;

/* External outputs (root outports fed by signals with default storage) */
ExternalOutputseC rtYeC;

/* Real-time model */
RT_MODELeC rtMeC_;
RT_MODELeC *const rtMeC = &rtMeC_;
extern void MWSPCGlmsnb_S(const real32_T xeC[], const real32_T deC[], real32_T
  mueC, uint32_T *startIdxeC, real32_T xBufeC[], real32_T wBufeC[], int32_T
  wLeneC, real32_T leakFaceC, int32_T xLeneC, real32_T yeC[], real32_T eYeC[],
  real32_T wYeC[], boolean_T NeedAdapteC);
void MWSPCGlmsnb_S(const real32_T xeC[], const real32_T deC[], real32_T mueC,
                   uint32_T *startIdxeC, real32_T xBufeC[], real32_T wBufeC[],
                   int32_T wLeneC, real32_T leakFaceC, int32_T xLeneC, real32_T
                   yeC[], real32_T eYeC[], real32_T wYeC[], boolean_T
                   NeedAdapteC)
{
  int32_T ieC;
  int32_T jeC;
  real32_T bufEnergyeC;
  int32_T j1eC;

  /* S-Function (sdsplms): '<Root>/LMS Filter1' */
  for (ieC = 0; ieC < xLeneC; ieC++) {
    yeC[ieC] = 0.0F;
  }

  for (ieC = 0; ieC < xLeneC; ieC++) {
    bufEnergyeC = 0.0F;

    /* Copy the current sample at the END of the circular buffer and update BuffStartIdx
     */
    xBufeC[*startIdxeC] = xeC[ieC];
    (*startIdxeC)++;
    if (*startIdxeC == (uint32_T)wLeneC) {
      *startIdxeC = 0U;
    }

    /* Multiply wgtBuff_vector (not yet updated) and inBuff_vector
     */
    /* Get the energy of the signal in updated buffer
     */
    j1eC = 0;
    for (jeC = (int32_T)*startIdxeC; jeC < wLeneC; jeC++) {
      yeC[ieC] += wBufeC[j1eC] * xBufeC[jeC];
      bufEnergyeC += xBufeC[jeC] * xBufeC[jeC];
      j1eC++;
    }

    for (jeC = 0; jeC < (int32_T)*startIdxeC; jeC++) {
      yeC[ieC] += wBufeC[j1eC] * xBufeC[jeC];
      bufEnergyeC += xBufeC[jeC] * xBufeC[jeC];
      j1eC++;
    }

    /* Ger error for the current sample
     */
    eYeC[ieC] = deC[ieC] - yeC[ieC];

    /* Update weight-vector for next input sample
     */
    if (NeedAdapteC) {
      j1eC = 0;
      for (jeC = (int32_T)*startIdxeC; jeC < wLeneC; jeC++) {
        wBufeC[j1eC] = xBufeC[jeC] / (bufEnergyeC + 1.1920929E-7F) * eYeC[ieC] *
          mueC + leakFaceC * wBufeC[j1eC];
        j1eC++;
      }

      for (jeC = 0; jeC < (int32_T)*startIdxeC; jeC++) {
        wBufeC[j1eC] = xBufeC[jeC] / (bufEnergyeC + 1.1920929E-7F) * eYeC[ieC] *
          mueC + leakFaceC * wBufeC[j1eC];
        j1eC++;
      }
    }
  }

  j1eC = wLeneC;
  for (jeC = 0; jeC < wLeneC; jeC++) {
    wYeC[jeC] = wBufeC[j1eC - 1];
    j1eC--;
  }

  /* End of S-Function (sdsplms): '<Root>/LMS Filter1' */
}

/* Model step function */
void Echo_cancel_step(void)
{
  boolean_T needAdapteC;

  /* S-Function (sdsplms): '<Root>/LMS Filter1' incorporates:
   *  Inport: '<Root>/Enable'
   *  Inport: '<Root>/Reset'
   */
  needAdapteC = false;
  if (rtUeC.ReseteC) {
    rtDWorkeC.LMSFilter1_WGT_IC_DWORKeC[0] = 0.0F;
    rtDWorkeC.LMSFilter1_WGT_IC_DWORKeC[1] = 0.0F;
    rtDWorkeC.LMSFilter1_WGT_IC_DWORKeC[2] = 0.0F;
  }

  if (rtUeC.EnableeC) {
    needAdapteC = true;
  }

  /* Outport: '<Root>/debug_error' incorporates:
   *  Inport: '<Root>/BLE_receive'
   *  Inport: '<Root>/MIC_data'
   *  Outport: '<Root>/Output'
   *  Outport: '<Root>/debug_coeffs'
   *  S-Function (sdsplms): '<Root>/LMS Filter1'
   */
  MWSPCGlmsnb_S(&rtUeC.BLE_receiveeC, &rtUeC.MIC_dataeC, 0.02F,
                &rtDWorkeC.LMSFilter1_BUFF_IDX_DWORKeC,
                &rtDWorkeC.LMSFilter1_IN_BUFFER_DWORKeC[0U],
                &rtDWorkeC.LMSFilter1_WGT_IC_DWORKeC[0U], 3, 1.0F, 1,
                &rtYeC.debug_erroreC, &rtYeC.OutputeC, &rtYeC.debug_coeffseC[0U],
                needAdapteC);
}

/* Model initialize function */
void Echo_cancel_initialize(void)
{
  /* (no initialization code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
