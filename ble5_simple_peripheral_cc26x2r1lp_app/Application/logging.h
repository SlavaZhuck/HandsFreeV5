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

#ifdef  LOGGING
    #define BASE64_SIZE(x)  (((x)+2) / 3 * 4 + 1)
#endif

void logging_init(void);
void send_log_message_to_UART_mailbox (uint8_t message_type);
void swap_endian (uint8_t* pointer, uint32_t size);
void start_logging_clock (void);
void stop_logging_clock (void);
void update_SID (pzCharacteristicData_t *pCharData);
void Send_message_Buf_Status(void);
void Send_message_CONN_Status (void);

#endif /* APPLICATION_LOGGING_H_ */
