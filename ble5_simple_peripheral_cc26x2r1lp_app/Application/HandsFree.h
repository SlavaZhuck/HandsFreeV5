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


#define TRANSMIT_DATA_LENGTH              167u  /* size of 1 BLE packet*/
#define SID_LENGTH                        20u   /* size of session identifier, used in LOGGING*/
#define MAC_SIZE                          6u    /* size of mac address, used in LOGGING*/

#define TMR_PERIOD                          ((48000000UL))                      /* MCU frequency, 48 MHz*/
#define LOW_STATE_TIME                      ((TMR_PERIOD / 10) * 9)             /* time for LED state is OFF, used in LED blinking*/
#define HIGH_STATE_TIME                     ((TMR_PERIOD) - (LOW_STATE_TIME))   /* time for LED state is ON, used in LED blinking*/

#define SAMP_PERIOD                       (20.0f)  /* in ms. Period of BLE transmissions and read/write to I2S driver*/


#define SAMP_TIME                         ((TMR_PERIOD) * (SAMP_PERIOD / 1000.0f) - 1)                                   /* SAMP_PERIOD recalculated to number of system timer ticks */
#define PACKET_CODEC_META_DATA            (3u)                                                                           /* 3 bytes for sound codec meta data: previous amplitude and size of step*/
#define PACKET_PACKET_NUMBER_LENGHT       (4u)                                                                           /* size of packet numer field, uint32_t = 4 bytes*/
#define V_STREAM_OUTPUT_SOUND_LEN         (TRANSMIT_DATA_LENGTH - PACKET_CODEC_META_DATA - PACKET_PACKET_NUMBER_LENGHT ) /* size of sound data */


#define MAILBOX_DEPTH       30                  /* size of BLE receiving FIFO */
#define RESEND_DELAY        (SAMP_PERIOD/2.0f)  /* if BLE packet wasn't sent - after this time new attemp will be made */

/******I2S Start ************************************************************************************/
#define I2S_SAMP_PER_FRAME              320u    /* number of frames per one SAMP_PERIOD*/
#define NUM_CHAN                        2u      /* number if I2S channels: 1 for read and 1 for write*/


#define FRAME_SIZE                      160u    /* I2S number of samples in one driver read and write operation */

/* memory buffer for I2S driver */
#define I2S_TOTAL_QUEUE_MEM_SZ         (I2S_BLOCK_OVERHEAD_IN_BYTES *           \
                                        I2SCC26XX_QUEUE_SIZE *                  \
                                        NUM_CHAN)
/* memory buffer for I2S driver */
#define I2S_SAMPLE_MEMORY_SZ           (FRAME_SIZE *                            \
                                        I2SCC26XX_QUEUE_SIZE *                  \
                                        NUM_CHAN)
/******I2S End ************************************************************************************/

/* Type of messages for LOGGING*/
#define PACKET_RECEIVED_MESSAGE_TYPE 0u
#define PACKET_SENT_MESSAGE_TYPE     1u
#define PACKET_SENT_ERROR_TYPE       2u
#define RECEIVE_BUFFER_STATUS        3u


void HandsFree_init (void);

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
}__attribute__((packed));

struct event_indicator_struct_BUF_status {
    uint8_t message_type;
    mac_dataType MAC_addr;
    sid_dataType SID;
    uint32_t timestamp;
    uint8_t buff_status;
}__attribute__((packed));


#endif /* APPLICATION_HANDSFREE_H_ */
