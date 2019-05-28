/*
 * File: Echo_cancel.c
 *
 * Code generated for Simulink model 'Echo_cancel'.
 *
 * Model version                  : 1.16
 * Simulink Coder version         : 9.1 (R2019a) 23-Nov-2018
 * C/C++ source code generated on : Tue May 28 21:32:19 2019
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "Echo_cancel.h"
#ifndef UCHAR_MAX
#include <limits.h>
#endif

#if ( UCHAR_MAX != (0xFFU) ) || ( SCHAR_MAX != (0x7F) )
#error Code was generated for compiler with different sized uchar/char. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

#if ( USHRT_MAX != (0xFFFFU) ) || ( SHRT_MAX != (0x7FFF) )
#error Code was generated for compiler with different sized ushort/short. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

#if ( UINT_MAX != (0xFFFFFFFFU) ) || ( INT_MAX != (0x7FFFFFFF) )
#error Code was generated for compiler with different sized uint/int. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

#if ( ULONG_MAX != (0xFFFFFFFFU) ) || ( LONG_MAX != (0x7FFFFFFF) )
#error Code was generated for compiler with different sized ulong/long. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

/* Skipping ulong_long/long_long check: insufficient preprocessor integer range. */

/* Block signals and states (default storage) */
D_WorkeC rtDWorkeC;

/* External inputs (root inport signals with default storage) */
ExternalInputseC rtUeC;

/* External outputs (root outports fed by signals with default storage) */
ExternalOutputseC rtYeC;

/* Real-time model */
RT_MODELeC rtMeC_;
RT_MODELeC *const rtMeC = &rtMeC_;
extern void MWSPCGlmsna10101010101_LElFahUI(const int16_T xeC[], const int16_T
  deC[], int16_T mueC, uint32_T *startIdxeC, int16_T xBufeC[], int16_T wBufeC[],
  int32_T wLeneC, int16_T leakFaceC, int32_T xLeneC, int16_T yeC[], int16_T
  eYeC[], boolean_T NeedAdapteC);
static int32_T div_nzp_s32_round(int32_T numeratoreC, int32_T denominatoreC);
static int32_T div_nzp_s32_round(int32_T numeratoreC, int32_T denominatoreC)
{
  uint32_T absNumeratoreC;
  uint32_T absDenominatoreC;
  uint32_T tempAbsQuotienteC;
  absNumeratoreC = numeratoreC < 0 ? ~(uint32_T)numeratoreC + 1U : (uint32_T)
    numeratoreC;
  absDenominatoreC = denominatoreC < 0 ? ~(uint32_T)denominatoreC + 1U :
    (uint32_T)denominatoreC;
  tempAbsQuotienteC = absNumeratoreC / absDenominatoreC;
  absNumeratoreC %= absDenominatoreC;
  absNumeratoreC <<= 1U;
  if (absNumeratoreC >= absDenominatoreC) {
    tempAbsQuotienteC++;
  }

  return (numeratoreC < 0) != (denominatoreC < 0) ? -(int32_T)tempAbsQuotienteC :
    (int32_T)tempAbsQuotienteC;
}

void MWSPCGlmsna10101010101_LElFahUI(const int16_T xeC[], const int16_T deC[],
  int16_T mueC, uint32_T *startIdxeC, int16_T xBufeC[], int16_T wBufeC[],
  int32_T wLeneC, int16_T leakFaceC, int32_T xLeneC, int16_T yeC[], int16_T
  eYeC[], boolean_T NeedAdapteC)
{
  int32_T ieC;
  int32_T jeC;
  int16_T accWtUeC;
  int16_T accUtUeC;
  int32_T j1eC;
  int32_T tmpeC;
  int32_T tmp_0eC;
  int32_T tmp_1eC;

  /* S-Function (sdsplms): '<Root>/LMS Filter1' */
  for (ieC = 0; ieC < xLeneC; ieC++) {
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
    accWtUeC = 0;
    accUtUeC = 0;
    j1eC = 0;
    for (jeC = (int32_T)*startIdxeC; jeC < wLeneC; jeC++) {
      tmpeC = wBufeC[j1eC] * xBufeC[jeC];
      if (tmpeC > 32767) {
        tmpeC = 32767;
      } else {
        if (tmpeC < -32768) {
          tmpeC = -32768;
        }
      }

      tmpeC += accWtUeC;
      if (tmpeC > 32767) {
        tmpeC = 32767;
      } else {
        if (tmpeC < -32768) {
          tmpeC = -32768;
        }
      }

      accWtUeC = (int16_T)tmpeC;
      tmpeC = xBufeC[jeC] * xBufeC[jeC];
      if (tmpeC > 32767) {
        tmpeC = 32767;
      } else {
        if (tmpeC < -32768) {
          tmpeC = -32768;
        }
      }

      tmpeC += accUtUeC;
      if (tmpeC > 32767) {
        tmpeC = 32767;
      } else {
        if (tmpeC < -32768) {
          tmpeC = -32768;
        }
      }

      accUtUeC = (int16_T)tmpeC;
      j1eC++;
    }

    for (jeC = 0; jeC < (int32_T)*startIdxeC; jeC++) {
      tmpeC = wBufeC[j1eC] * xBufeC[jeC];
      if (tmpeC > 32767) {
        tmpeC = 32767;
      } else {
        if (tmpeC < -32768) {
          tmpeC = -32768;
        }
      }

      tmpeC += accWtUeC;
      if (tmpeC > 32767) {
        tmpeC = 32767;
      } else {
        if (tmpeC < -32768) {
          tmpeC = -32768;
        }
      }

      accWtUeC = (int16_T)tmpeC;
      tmpeC = xBufeC[jeC] * xBufeC[jeC];
      if (tmpeC > 32767) {
        tmpeC = 32767;
      } else {
        if (tmpeC < -32768) {
          tmpeC = -32768;
        }
      }

      tmpeC += accUtUeC;
      if (tmpeC > 32767) {
        tmpeC = 32767;
      } else {
        if (tmpeC < -32768) {
          tmpeC = -32768;
        }
      }

      accUtUeC = (int16_T)tmpeC;
      j1eC++;
    }

    yeC[ieC] = accWtUeC;

    /* Get error for the current sample
     */
    eYeC[ieC] = deC[ieC];
    tmpeC = eYeC[ieC] - yeC[ieC];
    if (tmpeC > 32767) {
      tmpeC = 32767;
    } else {
      if (tmpeC < -32768) {
        tmpeC = -32768;
      }
    }

    eYeC[ieC] = (int16_T)tmpeC;
    tmpeC = mueC * eYeC[ieC];
    tmpeC = div_nzp_s32_round((int16_T)((((tmpeC & 65536U) != 0U) && (((tmpeC &
      65535U) != 0U) || (tmpeC > 0))) + (tmpeC >> 17)), accUtUeC);
    if (tmpeC > 32767) {
      tmpeC = 32767;
    } else {
      if (tmpeC < -32768) {
        tmpeC = -32768;
      }
    }

    /* Update weight-vector for next input sample
     */
    if (NeedAdapteC) {
      j1eC = 0;
      for (jeC = (int32_T)*startIdxeC; jeC < wLeneC; jeC++) {
        tmp_1eC = xBufeC[jeC] * (int16_T)tmpeC;
        if (tmp_1eC > 32767) {
          tmp_1eC = 32767;
        } else {
          if (tmp_1eC < -32768) {
            tmp_1eC = -32768;
          }
        }

        tmp_0eC = leakFaceC * wBufeC[j1eC];
        tmp_0eC = (((tmp_0eC & 8192U) != 0U) && (((tmp_0eC & 8191U) != 0U) ||
                    (tmp_0eC > 0))) + (tmp_0eC >> 14);
        if (tmp_0eC > 32767) {
          tmp_0eC = 32767;
        } else {
          if (tmp_0eC < -32768) {
            tmp_0eC = -32768;
          }
        }

        tmp_1eC = (int16_T)tmp_1eC + tmp_0eC;
        if (tmp_1eC > 32767) {
          tmp_1eC = 32767;
        } else {
          if (tmp_1eC < -32768) {
            tmp_1eC = -32768;
          }
        }

        wBufeC[j1eC] = (int16_T)tmp_1eC;
        j1eC++;
      }

      for (jeC = 0; jeC < (int32_T)*startIdxeC; jeC++) {
        tmp_1eC = xBufeC[jeC] * (int16_T)tmpeC;
        if (tmp_1eC > 32767) {
          tmp_1eC = 32767;
        } else {
          if (tmp_1eC < -32768) {
            tmp_1eC = -32768;
          }
        }

        tmp_0eC = leakFaceC * wBufeC[j1eC];
        tmp_0eC = (((tmp_0eC & 8192U) != 0U) && (((tmp_0eC & 8191U) != 0U) ||
                    (tmp_0eC > 0))) + (tmp_0eC >> 14);
        if (tmp_0eC > 32767) {
          tmp_0eC = 32767;
        } else {
          if (tmp_0eC < -32768) {
            tmp_0eC = -32768;
          }
        }

        tmp_1eC = (int16_T)tmp_1eC + tmp_0eC;
        if (tmp_1eC > 32767) {
          tmp_1eC = 32767;
        } else {
          if (tmp_1eC < -32768) {
            tmp_1eC = -32768;
          }
        }

        wBufeC[j1eC] = (int16_T)tmp_1eC;
        j1eC++;
      }
    }
  }

  /* End of S-Function (sdsplms): '<Root>/LMS Filter1' */
}

/* Model step function */
void Echo_cancel_step(void)
{
  boolean_T needAdapteC;
  int32_T ieC;
  int16_T rtb_LMSFilter1_o1eC;

  /* S-Function (sdsplms): '<Root>/LMS Filter1' incorporates:
   *  Inport: '<Root>/Enable'
   *  Inport: '<Root>/Reset'
   */
  needAdapteC = false;
  if (rtUeC.ReseteC) {
    for (ieC = 0; ieC < 5; ieC++) {
      rtDWorkeC.LMSFilter1_WGT_IC_DWORKeC[ieC] = 0;
    }
  }

  if (rtUeC.EnableeC) {
    needAdapteC = true;
  }

  /* Outport: '<Root>/Output' incorporates:
   *  Inport: '<Root>/BLE_receive'
   *  Inport: '<Root>/MIC_data'
   *  S-Function (sdsplms): '<Root>/LMS Filter1'
   */
  MWSPCGlmsna10101010101_LElFahUI(&rtUeC.BLE_receiveeC, &rtUeC.MIC_dataeC, 26214,
    &rtDWorkeC.LMSFilter1_BUFF_IDX_DWORKeC,
    &rtDWorkeC.LMSFilter1_IN_BUFFER_DWORKeC[0U],
    &rtDWorkeC.LMSFilter1_WGT_IC_DWORKeC[0U], 5, 16384, 1, &rtb_LMSFilter1_o1eC,
    &rtYeC.OutputeC, needAdapteC);
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
