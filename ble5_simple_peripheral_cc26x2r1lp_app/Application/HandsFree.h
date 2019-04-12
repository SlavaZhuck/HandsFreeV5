/*
 * HandsFree.h
 *
 *  Created on: 24 џэт. 2019 у.
 *      Author: CIT_007
 */

#ifndef APPLICATION_HANDSFREE_H_
#define APPLICATION_HANDSFREE_H_

#include "project_zero.h"
#include "./services/data_service.h"
#include "max9860_i2c.h"
#include <ti/drivers/timer/GPTimerCC26XX.h>
#include <ti/sysbios/BIOS.h>
#include <GeneralDef.h>


#define I2S_SAMP_PER_FRAME                320
#define TRANSMIT_DATA_LENGTH              167  //bytes

#define TMR_PERIOD                          ((48000000UL))
#define LOW_STATE_TIME                      ((TMR_PERIOD / 10) * 9)
#define HIGH_STATE_TIME                     (TMR_PERIOD - LOW_STATE_TIME)

#define SAMP_PERIOD                       (20.0f)  //ms


#define SAMP_TIME                         (TMR_PERIOD * (SAMP_PERIOD / 1000.0f) - 1)
//#define SAMP_TIME                          (4799999/4)
#define V_STREAM_OUTPUT_SOUND_LEN                (TRANSMIT_DATA_LENGTH - 7)// - 7


#define MAILBOX_DEPTH       10
/******I2S Start ******/
//#define I2S_MEM_BASE                        (GPRAM_BASE + FlashSectorSizeGet())



//#define NUM_OF_CHANNELS                     2
//#define I2S_BUF                             sizeof(int16_t) * (I2S_SAMP_PER_FRAME *   \
//                                            I2SCC26XX_QUEUE_SIZE * NUM_OF_CHANNELS)
#define NUM_CHAN                        2

/*
 * Configure for a 10ms frame @ 16kHz sample rate.
 * Note that the frame size variable is limited to a max size of 255.
 * It is an 8bit field in hardware (AIFDMACFG).
 */
//#define FRAME_SIZE                      I2S_SAMP_PER_FRAME

#define FRAME_SIZE                      160

#define I2S_TOTAL_QUEUE_MEM_SZ         (I2S_BLOCK_OVERHEAD_IN_BYTES *           \
                                        I2SCC26XX_QUEUE_SIZE *                  \
                                        NUM_CHAN)

#define I2S_SAMPLE_MEMORY_SZ           (FRAME_SIZE *                            \
                                        I2SCC26XX_QUEUE_SIZE *                  \
                                        NUM_CHAN)

/******I2S End ******/


void start_voice_handle(void);
void stop_voice_handle(void);

void HandsFree_init (void);



void blink_timer_callback(GPTimerCC26XX_Handle handle, GPTimerCC26XX_IntMask interruptMask);
void samp_timer_callback(GPTimerCC26XX_Handle handle, GPTimerCC26XX_IntMask interruptMask);


#endif /* APPLICATION_HANDSFREE_H_ */
