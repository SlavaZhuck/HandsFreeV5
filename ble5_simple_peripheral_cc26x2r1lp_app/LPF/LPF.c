/*
 * File: LPF.c
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
    int32_T j;
    real32_T rtb_DigitalFilter_e;
    real32_T rtb_DigitalFilter_h;
    real32_T rtb_DigitalFilter;
    real32_T rtb_DigitalFilter_o;

    /* MultiPortSwitch: '<Root>/Index Vector' incorporates:
     *  Inport: '<Root>/switch'
     */
    switch ((int32_T) rtU.switch_a)
    {
    case 0:
        /* DiscreteFir: '<S1>/Digital Filter' incorporates:
         *  Inport: '<Root>/In1'
         */
        /* Consume delay line and beginning of input samples */
        rtb_DigitalFilter = rtU.In1 * rtConstP.DigitalFilter_Coefficients[0];
        for (j = 0; j < 9; j++)
        {
            rtb_DigitalFilter += rtConstP.DigitalFilter_Coefficients[1 + j]
                    * rtDW.DigitalFilter_states[j];
        }

        /* Update delay line for next frame */
        for (j = 7; j >= 0; j--)
        {
            rtDW.DigitalFilter_states[1 + j] = rtDW.DigitalFilter_states[j];
        }

        rtDW.DigitalFilter_states[0] = rtU.In1;

        /* End of DiscreteFir: '<S1>/Digital Filter' */
        /* Outport: '<Root>/Out1' incorporates:
         *  DataTypeConversion: '<Root>/Data Type Conversion'
         */
        rtY.Out1 = (int16_T) floorf(rtb_DigitalFilter);
        break;

    case 1:
        /* DiscreteFir: '<S2>/Digital Filter' incorporates:
         *  Inport: '<Root>/In1'
         */
        /* Consume delay line and beginning of input samples */
        rtb_DigitalFilter_h = rtU.In1
                * rtConstP.DigitalFilter_Coefficients_f[0];
        for (j = 0; j < 20; j++)
        {
            rtb_DigitalFilter_h += rtConstP.DigitalFilter_Coefficients_f[1 + j]
                    * rtDW.DigitalFilter_states_g[j];
        }

        /* Update delay line for next frame */
        for (j = 18; j >= 0; j--)
        {
            rtDW.DigitalFilter_states_g[1 + j] = rtDW.DigitalFilter_states_g[j];
        }

        rtDW.DigitalFilter_states_g[0] = rtU.In1;

        /* End of DiscreteFir: '<S2>/Digital Filter' */

        /* Outport: '<Root>/Out1' incorporates:
         *  DataTypeConversion: '<Root>/Data Type Conversion1'
         */
        rtY.Out1 = (int16_T) floorf(rtb_DigitalFilter_h);
        break;

    case 2:
        /* DiscreteFir: '<S3>/Digital Filter' incorporates:
         *  Inport: '<Root>/In1'
         */
        /* Consume delay line and beginning of input samples */
        rtb_DigitalFilter_e = rtU.In1
                * rtConstP.DigitalFilter_Coefficients_e[0];
        for (j = 0; j < 20; j++)
        {
            rtb_DigitalFilter_e += rtConstP.DigitalFilter_Coefficients_e[1 + j]
                    * rtDW.DigitalFilter_states_n[j];
        }

        /* Update delay line for next frame */
        for (j = 18; j >= 0; j--)
        {
            rtDW.DigitalFilter_states_n[1 + j] = rtDW.DigitalFilter_states_n[j];
        }

        rtDW.DigitalFilter_states_n[0] = rtU.In1;

        /* End of DiscreteFir: '<S3>/Digital Filter' */

        /* Outport: '<Root>/Out1' incorporates:
         *  DataTypeConversion: '<Root>/Data Type Conversion2'
         */
        rtY.Out1 = (int16_T) floorf(rtb_DigitalFilter_e);
        break;

    case 3:
        /* DiscreteFir: '<S4>/Digital Filter' incorporates:
         *  Inport: '<Root>/In1'
         */
        /* Consume delay line and beginning of input samples */
        rtb_DigitalFilter_o = rtU.In1
                * rtConstP.DigitalFilter_Coefficients_d[0];
        for (j = 0; j < 20; j++)
        {
            rtb_DigitalFilter_o += rtConstP.DigitalFilter_Coefficients_d[1 + j]
                    * rtDW.DigitalFilter_states_i[j];
        }

        /* Update delay line for next frame */
        for (j = 18; j >= 0; j--)
        {
            rtDW.DigitalFilter_states_i[1 + j] = rtDW.DigitalFilter_states_i[j];
        }

        rtDW.DigitalFilter_states_i[0] = rtU.In1;

        /* End of DiscreteFir: '<S4>/Digital Filter' */
        /* Outport: '<Root>/Out1' incorporates:
         *  DataTypeConversion: '<Root>/Data Type Conversion3'
         */
        rtY.Out1 = (int16_T) floorf(rtb_DigitalFilter_o);
        break;

    default:
        rtY.Out1 = (int16_T) 0;
        break;
    }
    /* End of MultiPortSwitch: '<Root>/Index Vector' */
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
