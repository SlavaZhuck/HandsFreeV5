/*
 * logging.c
 *
 *  Created on: 13 ���. 2019 �.
 *      Author: CIT_007
 */
#include "logging.h"
#include "GeneralDef.h"
#include "HandsFree.h"
#include <ti/sysbios/knl/Mailbox.h>

#include <ti/sysbios/BIOS.h>

#ifdef LOGGING
extern bool UART_ready;
extern uint8_t macAddress[MAC_SIZE];
extern uint32_t measure_tim_value;
extern UART_Handle uart;
extern uint32_t counter_packet_received;
extern volatile uint32_t counter_packet_send;
extern Mailbox_Handle mailbox;

static uint8_t received_SID[SID_LENGTH] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19};
struct event_indicator_struct_BUF_status  event_BUF_status_message;
struct event_indicator_struct_BLE         event_BLE_message;
struct connection_status                  event_connection_status_message;
static uint8_t UART_BUF_status_message               [BASE64_SIZE(sizeof(event_BUF_status_message))];
static uint8_t logging_buffer_BLE_message_receive    [BASE64_SIZE(sizeof(event_BLE_message))];
static uint8_t logging_buffer_BLE_message_send       [BASE64_SIZE(sizeof(event_BLE_message))];
static uint8_t logging_buffer_BLE_message_send_error [BASE64_SIZE(sizeof(event_BLE_message))];
static uint8_t UART_buffer_connection_status         [BASE64_SIZE(sizeof(event_connection_status_message))];

static Clock_Struct Uart_Connection_Status;
static Clock_Handle Uart_Connection_Status_ClockHandle;
static void UART_Send_Connection_Status_SwiFxn(UArg temp);

static Clock_Struct LogUART_send_ChannelSwitchClock;
static Clock_Handle LogUART_send_ClockHandle;
static void LogUART_send_SwiFxn(UArg temp);

static Mailbox_Handle mailbox_for_UART;
static uint8_t mailpost_for_UART_usage;
uint8_t UART_logging_message   [(sizeof(logging_buffer_BLE_message_receive))];

#define CONNECTION_STATUS_DELAY (10u) /* delay in ms between status messages sending, when BLE connection is established */
#define CONNECTION_STATUS_MESSAGE_NUM (2u) /* number of periodically sent messages */

static uint32_t sended_messages = 0u;

char *base64_encode(char *out, int out_size, const uint8_t *in, int in_size)
{
    static const char b64[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    char *ret, *dst;
    unsigned i_bits = 0;
    int i_shift = 0;
    int bytes_remaining = in_size;

    if (in_size >= UINT_MAX / 4 ||
        out_size < BASE64_SIZE(in_size))
        return NULL;
    ret = dst = out;
    while (bytes_remaining) {
        i_bits = (i_bits << 8) + *in++;
        bytes_remaining--;
        i_shift += 8;

        do {
            *dst++ = b64[(i_bits << 6 >> i_shift) & 0x3f];
            i_shift -= 6;
        } while (i_shift > 6 || (bytes_remaining == 0 && i_shift > 0));
    }
    while ((dst - ret) & 3)
        *dst++ = '=';
    *dst = '\0';

    return ret;
}

void logging_init(void)
{
    mailbox_for_UART = Mailbox_create(sizeof(logging_buffer_BLE_message_receive), MAILBOX_DEPTH, NULL, NULL);
    LogUART_send_ClockHandle = Util_constructClock(&LogUART_send_ChannelSwitchClock,
                                                   LogUART_send_SwiFxn, CONNECTION_STATUS_DELAY,
                                                       1, //1ms
                                                       false, //not start without ClockStart
                                                       0);

    Uart_Connection_Status_ClockHandle = Util_constructClock(&Uart_Connection_Status,
                                                             UART_Send_Connection_Status_SwiFxn, CONNECTION_STATUS_DELAY,
                                                       0,
                                                       0,
                                                       0);
    for(uint16_t i = 0 ; i < sizeof(event_connection_status_message.SID); i++)
    {
        event_connection_status_message.SID[i] = received_SID[i];
    }
    for(uint16_t i = 0 ; i < sizeof(event_connection_status_message.MAC_addr) ; i++)
    {
        event_connection_status_message.MAC_addr[i] = macAddress[i];
    }
    event_connection_status_message.message_type = CONNECTION_STATUS;
    event_BUF_status_message.message_type = RECEIVE_BUFFER_STATUS;
    swap_endian((uint8_t*)&event_connection_status_message.MAC_addr, sizeof(event_connection_status_message.MAC_addr));

}


void send_log_message_to_UART_mailbox (uint8_t message_type, uint32_t counter_packet_received_uart)
{
    //uint32_t timestamp = GPTimerCC26XX_getValue(measure_tim_hdl);
    uint32_t timestamp = measure_tim_value;
    event_BLE_message.message_type = message_type;
    if(PACKET_RECEIVED_MESSAGE_TYPE == message_type)
    {
        event_BLE_message.packet_number = counter_packet_received_uart;
    }
    else if ((PACKET_SENT_MESSAGE_TYPE == message_type) || (PACKET_SENT_ERROR_TYPE == message_type))
    {
        event_BLE_message.packet_number = counter_packet_send;
    }

    event_BLE_message.timestamp = timestamp;

    event_BUF_status_message.buff_status = Mailbox_getNumPendingMsgs(mailbox);
    event_BUF_status_message.timestamp = timestamp;

    swap_endian((uint8_t*)&event_BLE_message.packet_number, sizeof(event_BLE_message.packet_number));
    swap_endian((uint8_t*)&event_BLE_message.timestamp, sizeof(event_BLE_message.timestamp));

    swap_endian((uint8_t*)&event_BUF_status_message.timestamp, sizeof(event_BUF_status_message.timestamp));

    /* depending on message type - add message to Mailbox*/
    /* different UART buffers were created, because if previous buffer was not sent,
     * this buffer would be overwritten by new one
     * */
    if( PACKET_RECEIVED_MESSAGE_TYPE== event_BLE_message.message_type)
    {
        base64_encode((char *)logging_buffer_BLE_message_receive, (int32_t)(sizeof(logging_buffer_BLE_message_receive)), (const uint8_t *)&event_BLE_message, sizeof(event_BLE_message));
        logging_buffer_BLE_message_receive[sizeof(logging_buffer_BLE_message_receive) - 1] = 0x0A;
        Mailbox_post(mailbox_for_UART, logging_buffer_BLE_message_receive, BIOS_NO_WAIT);
    }
    else if( PACKET_SENT_MESSAGE_TYPE== event_BLE_message.message_type)
    {
        base64_encode((char *)logging_buffer_BLE_message_send, (int32_t)(sizeof(logging_buffer_BLE_message_send)), (const uint8_t *)&event_BLE_message, sizeof(event_BLE_message));
        logging_buffer_BLE_message_send[sizeof(logging_buffer_BLE_message_send) - 1] = 0x0A;
        Mailbox_post(mailbox_for_UART, logging_buffer_BLE_message_send, BIOS_NO_WAIT);

    }
    else if( PACKET_SENT_ERROR_TYPE== event_BLE_message.message_type)
    {
        base64_encode((char *)logging_buffer_BLE_message_send_error, (int32_t)(sizeof(logging_buffer_BLE_message_send_error)), (const uint8_t *)&event_BLE_message, sizeof(event_BLE_message));
        logging_buffer_BLE_message_send_error[sizeof(logging_buffer_BLE_message_send_error) - 1] = 0x0A;
        Mailbox_post(mailbox_for_UART, logging_buffer_BLE_message_send_error, BIOS_NO_WAIT);

    }
}

void swap_endian (uint8_t* pointer, uint32_t size)
{
    uint8_t temp_array[size];

    for (uint32_t x = 0; x < size; x++)
    {
        temp_array[x] = *(&pointer[x]);
    }

    for (uint32_t x = 0; x < size; x++) {
        pointer[x] = temp_array[size - 1 - x];
    }
}

void start_logging_clock (void)
{
    Util_startClock((Clock_Struct *)LogUART_send_ClockHandle);
}

void stop_logging_clock (void)
{
    Util_stopClock((Clock_Struct *)LogUART_send_ClockHandle);
}


static void UART_Send_Connection_Status_SwiFxn(UArg temp)
{
    while(sended_messages < CONNECTION_STATUS_MESSAGE_NUM) /* while not all status messages were sent */
    {
        ProjectZero_enqueueMsg(PZ_APP_MSG_Send_message_CONN_Status, NULL);
        sended_messages++;
        Util_startClock((Clock_Struct *)Uart_Connection_Status_ClockHandle);
    }
}

static void LogUART_send_SwiFxn(UArg temp)
{
    if(UART_ready)
    {
        mailpost_for_UART_usage = Mailbox_getNumPendingMsgs(mailbox_for_UART);
        if(mailpost_for_UART_usage > 0)
        {
            Mailbox_pend(mailbox_for_UART, UART_logging_message, BIOS_NO_WAIT);
            UART_ready = false;
            UART_write(uart, UART_logging_message, sizeof(UART_logging_message));
        }
    }
}


void update_SID (pzCharacteristicData_t *pCharData)
{
        memset(received_SID, 0, SID_LENGTH);
        memcpy(received_SID, pCharData->data, SID_LENGTH);
        for(uint16_t i = 0 ; i < sizeof(event_connection_status_message.SID); i++)
        {
            event_connection_status_message.SID[i] = received_SID[i];
        }
        swap_endian((uint8_t*)&event_connection_status_message.SID, sizeof(event_connection_status_message.SID));

        sended_messages = 0;/* update number of already sent messages if new SID was received */
        Util_startClock((Clock_Struct *)Uart_Connection_Status_ClockHandle); /* start to periodically send status messages to UART */
}


void Send_message_Buf_Status(void)
{
    base64_encode((char *)UART_BUF_status_message, (int32_t)(sizeof(UART_BUF_status_message)), (const uint8_t *)&event_BUF_status_message, sizeof(event_BUF_status_message));
    UART_BUF_status_message[sizeof(UART_BUF_status_message) - 1] = 0x0A;
    //while(!UART_ready)
    {
        ; /* wait when previous UART message is sent*/
    }
    UART_ready = false;
    UART_write(uart, UART_BUF_status_message, sizeof(UART_BUF_status_message));
//            UART_write(uart, &event_BUF_status_message, sizeof(event_BUF_status_message));
}


void Send_message_CONN_Status (void)
{
    base64_encode((char *)UART_buffer_connection_status, (int32_t)(sizeof(UART_buffer_connection_status)), (const uint8_t *)&event_connection_status_message, sizeof(event_connection_status_message));
    UART_buffer_connection_status[sizeof(UART_buffer_connection_status) - 1] = 0x0A;
    while(!UART_ready)
    {
        ; /* wait when previous UART message is sent*/
    }
    UART_ready = false;
    UART_write(uart, UART_buffer_connection_status, sizeof(UART_buffer_connection_status));
//            UART_write(uart, &event_connection_status_message, sizeof(event_connection_status_message));
}

#endif
