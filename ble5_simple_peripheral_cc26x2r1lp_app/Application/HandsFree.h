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
#include <ti/drivers/AESCBC.h>
#include <ti/drivers/cryptoutils/cryptokey/CryptoKeyPlaintext.h>
#include <ti/sysbios/BIOS.h>
#include <GeneralDef.h>


#define I2S_SAMP_PER_FRAME                320u
#define TRANSMIT_DATA_LENGTH              167u  //bytes
#define SID_LENGTH                        20u   //bytes
#define MAC_SIZE                          6u   //bytes

#define TMR_PERIOD                          ((48000000UL))
#define LOW_STATE_TIME                      ((TMR_PERIOD / 10) * 9)
#define HIGH_STATE_TIME                     ((TMR_PERIOD) - (LOW_STATE_TIME))

#define SAMP_PERIOD                       (20.0f)  //ms


#define SAMP_TIME                         ((TMR_PERIOD) * (SAMP_PERIOD / 1000.0f) - 1)
//#define SAMP_TIME                          (4799999/4)
#define PACKET_CODEC_META_DATA                   (3u)
#define PACKET_PACKET_NUMBER_LENGHT              (4u)
#define V_STREAM_OUTPUT_SOUND_LEN                (TRANSMIT_DATA_LENGTH - PACKET_CODEC_META_DATA - PACKET_PACKET_NUMBER_LENGHT )// - 7


#define MAILBOX_DEPTH       20
#define RESEND_DELAY        (SAMP_PERIOD/2.0f)
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

#define PACKET_RECEIVED_MESSAGE_TYPE 0u
#define PACKET_SENT_MESSAGE_TYPE     1u
#define PACKET_SENT_ERROR_TYPE       2u
#define RECEIVE_BUFFER_STATUS        3u

void start_voice_handle(void);
void stop_voice_handle(void);

void HandsFree_init (void);



void blink_timer_callback(GPTimerCC26XX_Handle handle, GPTimerCC26XX_IntMask interruptMask);
void samp_timer_callback(GPTimerCC26XX_Handle handle, GPTimerCC26XX_IntMask interruptMask);


typedef uint8_t mac_dataType[MAC_SIZE];
typedef uint8_t sid_dataType[SID_LENGTH];


extern uint8_t read_aes_key(uint8_t *key);
extern uint8_t write_aes_key(uint8_t *key);

struct event_indicator_struct_BLE {
    uint8_t message_type;
    mac_dataType MAC_addr;
    sid_dataType SID;
    uint32_t timestamp;
    uint32_t packet_number;
    uint8_t null_terminator ;
}__attribute__((packed));

struct event_indicator_struct_BUF_status {
    uint8_t message_type;
    mac_dataType MAC_addr;
    sid_dataType SID;
    uint32_t timestamp;
    uint8_t buff_status;
    uint8_t null_terminator;
}__attribute__((packed));


#endif /* APPLICATION_HANDSFREE_H_ */
