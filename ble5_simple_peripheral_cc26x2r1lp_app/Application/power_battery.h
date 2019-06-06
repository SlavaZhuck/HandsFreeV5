/*
 * power_battery.h
 *
 *  Created on: 19 апр. 2019 г.
 *      Author: CIT_007
 */

#ifndef POWER_BATTERY_H_
#define POWER_BATTERY_H_
    /******ADC Start ******/
#define ADC_SWITCH_TIMEOUT      30 //ms
#define ADCBUFSIZE      1
#define SAMPFREQ        10000
#define ADC_VOLTAGE_MEASURE_PIN   CC26X2R1_LAUNCHXL_ADCBUF0CHANNEL5
#define ADC_POWER_BUTTON_PIN      CC26X2R1_LAUNCHXL_ADCBUF0CHANNEL4
#define POWER_ON 1
#define POWER_OFF 0
#define ADC_POWER_BUTTON_THRESHOLD 100 // ~15 pressed button, ~305 not pressed
#define BAT_LOW_VOLTAGE     1780 //3000
    /******ADC End ******/


void power_battery_init (void);
void power_off (void);
uint32_t get_bat_voltage(void);
#endif /* POWER_BATTERY_H_ */
