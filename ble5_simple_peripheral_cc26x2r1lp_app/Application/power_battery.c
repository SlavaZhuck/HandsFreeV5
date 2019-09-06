/*
 * power_battery.c
 *
 *  Created on: 19 апр. 2019 г.
 *      Author: CIT_007
 */
#include "HandsFree.h"
#include "power_battery.h"
#include "max9860_i2c.h"
#include <ti/drivers/ADCBuf.h>
#include "driverlib/aon_batmon.h"
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <stdbool.h>
#include <util.h>

static Clock_Struct ADC_ChannelSwitchClock;
Clock_Handle ADC_ChannelSwitchClockHandle;
static PIN_Handle powerPinHandle;
static PIN_State powerPinState;
static bool power_state  = FALSE;
int16_t power_button_voltage[ADCBUFSIZE];
int16_t power_button_counter = 0;
ADCBuf_Handle adc_hdl;
static ADCBuf_Params adc_params;
ADCBuf_Conversion adc_conversion;
int16_t batt_voltage[ADCBUFSIZE];
static int16_t samp_buf1[ADCBUFSIZE];
static int16_t samp_buf2[ADCBUFSIZE];
bool power_button_check = TRUE;
bool enable_blink = TRUE;

PIN_Config powerPinTable[] = {
    CC26X2R1_LAUNCHXL_PIN_POWER  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};

void adc_callback(ADCBuf_Handle handle, ADCBuf_Conversion *conversion, void *completedADCBuffer, uint32_t completedChannel);
static void ADC_ChannelSwitchSwiFxn(UArg buttonId);
uint32_t get_bat_voltage(void);

void power_battery_init (void)
{
    ADC_ChannelSwitchClockHandle = Util_constructClock(&ADC_ChannelSwitchClock,
                                                       ADC_ChannelSwitchSwiFxn, ADC_SWITCH_TIMEOUT,
                                                       0,
                                                       0,
                                                       BUTTON_VOLUME_LOW);
    AONBatMonEnable();
    /* ADC init */
    ADCBuf_init();

    ADCBuf_Params_init(&adc_params);
    adc_params.callbackFxn = adc_callback;
    adc_params.recurrenceMode = ADCBuf_RECURRENCE_MODE_ONE_SHOT;
    adc_params.returnMode = ADCBuf_RETURN_MODE_CALLBACK;
    adc_params.samplingFrequency = SAMPFREQ;
//
    adc_hdl = ADCBuf_open(Board_ADCBUF0, &adc_params);//Board_ADCBUF0
//
    adc_conversion.arg = NULL;
    adc_conversion.adcChannel = ADC_POWER_BUTTON_PIN;

    adc_conversion.sampleBuffer = samp_buf1;
    adc_conversion.sampleBufferTwo = samp_buf2;
    adc_conversion.samplesRequestedCount = ADCBUFSIZE;
    if (!adc_hdl)
    {
        while(1);
    }
    powerPinHandle = PIN_open(&powerPinState, powerPinTable);
    if(!powerPinHandle)
    {
       Task_exit();
    }
    PIN_setOutputValue(powerPinHandle, CC26X2R1_LAUNCHXL_PIN_POWER, POWER_ON);
}

void adc_callback(ADCBuf_Handle handle, ADCBuf_Conversion *conversion, void *completedADCBuffer, uint32_t completedChannel)
{
    int16_t *buf_ptr = (int16_t*)completedADCBuffer;
    /* handle receive data */

    if(power_button_check)/* uart battery read is turned OFF*/
    {
        power_button_voltage[0] = buf_ptr[0] ;
        if (power_button_voltage[0] < ADC_POWER_BUTTON_THRESHOLD)
        {
            power_button_counter++;
        }
        else
        {
            power_button_counter--;
        }
        if(power_button_counter > 3)
        {
            power_button_counter = 3;
            if (power_state == FALSE)
            {
                PIN_setOutputValue(powerPinHandle, CC26X2R1_LAUNCHXL_PIN_POWER, POWER_ON); //turn ON power
            }
            else
            {
                PIN_setOutputValue(powerPinHandle, CC26X2R1_LAUNCHXL_PIN_POWER, POWER_OFF); //turn OFF power
                enable_blink = FALSE; // disable blinking after power OFF
            }

        }
        else if (power_button_counter <= 0)
        {
            power_button_counter = 0;
            if (PIN_getOutputValue(CC26X2R1_LAUNCHXL_PIN_POWER))
            {
                power_state = TRUE;
            }
            else
            {
                power_state = FALSE;
            }

        }

    }
    else
    {
        batt_voltage[0] = buf_ptr[0] ;
        power_button_check = TRUE;
    }

   // ADCBuf_convertCancel(adc_hdl);
}

static void ADC_ChannelSwitchSwiFxn(UArg buttonId)
{
    if(power_button_check)
    {
        adc_conversion.adcChannel = ADC_POWER_BUTTON_PIN;
        if (ADCBuf_convert(adc_hdl, &adc_conversion, 1) != ADCBuf_STATUS_SUCCESS) {
            while(1);
        }
    }
}

uint32_t get_bat_voltage(void)
{
    return ((AONBatMonBatteryVoltageGet() * 125) >> 5);
}


void power_off (void)
{
    PIN_setOutputValue(powerPinHandle, CC26X2R1_LAUNCHXL_PIN_POWER, POWER_OFF);
}
