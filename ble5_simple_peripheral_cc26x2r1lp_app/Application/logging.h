/*
 * logging.h
 *
 *  Created on: 13 рту. 2019 у.
 *      Author: CIT_007
 */

#ifndef APPLICATION_LOGGING_H_
#define APPLICATION_LOGGING_H_

#include <stdint.h>
#include "project_zero.h"
#include "HandsFree.h"

#ifdef  LOGGING
    #define BASE64_SIZE(x)  (((x)+2) / 3 * 4 + 1)
#endif

void logging_init(void);
void send_log_message_to_UART_mailbox (uint8_t message_type, uint32_t counter_packet_received_uart);
void swap_endian (uint8_t* pointer, uint32_t size);
void start_logging_clock (void);
void stop_logging_clock (void);
void update_SID (pzCharacteristicData_t *pCharData);
void Send_message_CONN_Status (void);

typedef uint8_t mac_dataType[MAC_SIZE];
typedef uint8_t sid_dataType[SID_LENGTH];

struct event_indicator_struct_BLE {
    uint8_t message_type;
    uint32_t timestamp;
    uint32_t packet_number;
}__attribute__((packed));

struct event_indicator_struct_BUF_status {
    uint8_t message_type;
    uint32_t timestamp;
    uint8_t buff_status;
}__attribute__((packed));


struct connection_status {
    uint8_t message_type;
    mac_dataType MAC_addr;
    sid_dataType SID;
}__attribute__((packed));

#endif /* APPLICATION_LOGGING_H_ */
