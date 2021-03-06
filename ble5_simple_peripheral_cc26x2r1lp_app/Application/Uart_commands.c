/*
 * Uart_commands.c
 *
 *  Created on: 7 ����. 2018 �.
 *      Author: CIT_007
 */
#include <string.h>
#include "Uart_commands.h"
#include "project_zero.h"
#include "Uart_Parser.h"
#include <ti/drivers/UART.h>
#include "icall_ble_api.h"

#define KEY_SIZE                            16

 unsigned short calculated_CRC_TX;
extern Serial_Data_Packet Tx_Data;
extern Serial_Data_Packet Rx_Data;
extern UART_Handle uart;
extern uint8_t macAddress[6];

void calcCRC_andSend(void);
void clear_Tx_packet(void);

void calcCRC_andSend(void){
    calculated_CRC_TX = Crc16((unsigned char*)(&Tx_Data)+1,(unsigned short)(Tx_Data.data_lenght)+3);
    Tx_Data.CRC = calculated_CRC_TX;
    UART_writeCancel(uart);
    UART_write(uart, &Tx_Data, Tx_Data.data_lenght+4);
    UART_writeCancel(uart);
    UART_write(uart, &Tx_Data.CRC, 2);
}

void send_answer_for_command(uint8_t request){
    clear_Tx_packet();
    Tx_Data.data_lenght = 0;
    Tx_Data.command = request;
    calcCRC_andSend();
}


uint8_t send_data(void){//here we receive data from host

    //copying data somewhere
    return 1;
}




void send_fh_cr_tp(void){   //TODO
    clear_Tx_packet();
    if(1){
        send_answer_for_command(REC_OK);
    }else{
        send_answer_for_command(REC_ERROR) ;
    }
}
extern int16_t batt_voltage[];

void get_fh_param(void){
    //readMacfunction TODO
    //hciStatus_t HCI_ReadBDADDRCmd

    clear_Tx_packet();
    Tx_Data.data_lenght = 8;//MAClenght + battery
    for(uint8_t i = 0 ;i < 6;i++){
        Tx_Data.data[i]=macAddress[i];
    }


    uint16_t voltage = batt_voltage[0];
    Tx_Data.data[6]= voltage>>8;
    Tx_Data.data[7]= voltage & 0x00ff;

    Tx_Data.command = SEND_FH_PARAM ;
    calcCRC_andSend();
}

extern uint8_t global_key[];

void send_fh_key (void){
    clear_Tx_packet();

    if(!write_aes_key((uint8_t *)&Rx_Data.data))
    {
        //read_aes_key(global_key);//read key to make it work at time of write action
        send_answer_for_command(REC_OK);
    }
    else
    {
        send_answer_for_command(REC_ERROR) ;
    }
}

void get_fh_key(void){

    clear_Tx_packet();

    if(!read_aes_key(&Tx_Data.data[0]))
    {
        Tx_Data.data_lenght = KEY_SIZE;
        Tx_Data.command = SEND_FH_KEY ;
        calcCRC_andSend();
    }
    else
    {
        Tx_Data.command = REC_ERROR ;
        calcCRC_andSend();
    }
}

void get_fh_cr_tp(void){
    clear_Tx_packet();
    char type[7]= {'A', 'e', 's', '1', '2', '8' , '\0'};
    Tx_Data.data_lenght = 7;

    for(uint8_t i = 0 ;i<Tx_Data.data_lenght;i++)
    {
        Tx_Data.data[i]=type[i];
    }
    Tx_Data.command = SEND_FH_CR_TP;
    calcCRC_andSend();
}

void no_command (void){
    clear_Tx_packet();
    Tx_Data.data_lenght = 0;
    Tx_Data.command = NO_COMAND;
    calcCRC_andSend();
}

void bad_crc(void){
    clear_Tx_packet();
    Tx_Data.data_lenght = 0;
    Tx_Data.command = BAD_CRC;
    calcCRC_andSend();
}

void clear_Tx_packet(void){
    memset(&Tx_Data.data_lenght,0,sizeof(Tx_Data.data_lenght)+sizeof(Tx_Data.command)+sizeof(Tx_Data.data)+sizeof(Tx_Data.CRC));
}
