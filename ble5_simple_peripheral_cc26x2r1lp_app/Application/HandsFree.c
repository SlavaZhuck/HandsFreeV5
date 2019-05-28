/*
 * HandsFree.c
 *
 *  Created on: 24 янв. 2019 г.
 *      Author: CIT_007
 */
#include <stdint.h>
#include <stdlib.h>
#include "HandsFree.h"
#include "Noise_TRSH.h"
#include "Uart_commands.h"
#include "max9860_i2c.h"
#include "I2S/I2SCC26XX.h"
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include "ima_codec/ima.h"
#include <ti/sysbios/knl/Mailbox.h>
#include "Uart_Parser.h"
#include "Uart_commands.h"
#include "GeneralDef.h"
#include <osal_snv.h>
#include <ti/drivers/ADCBuf.h>
#include <osal/src/inc/osal_snv.h>
#include "driverlib/aon_batmon.h"
#include "buttons.h"
#include "power_battery.h"
#include "Noise_TRSH.h"
#include "../LPF/LPF.h"                       /* Model's header file */
#include "../LPF/rtwtypes.h"
#include "../Echo_cancel/Echo_cancel.h"

#ifdef  LOGGING
#define BASE64_SIZE(x)  (((x)+2) / 3 * 4 + 1)
#endif
/* Timer variables ***************************************************/
GPTimerCC26XX_Params tim_params;
GPTimerCC26XX_Handle blink_tim_hdl = NULL;
GPTimerCC26XX_Handle measure_tim_hdl = NULL;
GPTimerCC26XX_Handle samp_tim_hdl = NULL;
GPTimerCC26XX_Value load_val[2] = {LOW_STATE_TIME, HIGH_STATE_TIME};
/*********************************************************************/
/* CryptoKey storage */
CryptoKey           cryptoKey;
/* AESCBC variables */
AESCBC_Operation    operationOneStepEncrypt;
AESCBC_Operation    operationOneStepDecrypt;
AESCBC_Handle       AESCBCHandle;
uint8_t temp_crypto[V_STREAM_OUTPUT_SOUND_LEN + 3];

/* Debug time variables ***************************************************/
GPTimerCC26XX_Value timestamp_encode_start;
GPTimerCC26XX_Value timestamp_encode_stop;
GPTimerCC26XX_Value timestamp_encode_dif;
GPTimerCC26XX_Value timestamp_decode_start;
GPTimerCC26XX_Value timestamp_decode_stop;
GPTimerCC26XX_Value timestamp_decode_dif;

/***********LPF****************************************************/
GPTimerCC26XX_Value timestamp_LPF_start;
GPTimerCC26XX_Value timestamp_LPF_stop;
GPTimerCC26XX_Value timestamp_LPF_dif;
bool enable_LPF = false;

/*****************NOISE GATE***********************************/
GPTimerCC26XX_Value timestamp_NG_start;
GPTimerCC26XX_Value timestamp_NG_stop;
GPTimerCC26XX_Value timestamp_NG_dif;
struct power_struct in_power;
bool enable_NoiseGate = false;
/***********Echo compensation****************************************************/
GPTimerCC26XX_Value timestamp_EC_start;
GPTimerCC26XX_Value timestamp_EC_stop;
GPTimerCC26XX_Value timestamp_EC_dif;
bool enable_EC = false;
/******Crypto key Start ******/
#define KEY_SNV_ID                          BLE_NVID_CUST_START
#define KEY_SIZE                            16
#define INIT_VOL_ADDR                       BLE_NVID_CUST_START+1
/******Crypto key End ******/

extern ADCBuf_Conversion adc_conversion;
extern ADCBuf_Handle adc_hdl;
/******Uart Start ******/

#define UART_BAUD_RATE 921600
#define MAX_NUM_RX_BYTES    100   // Maximum RX bytes to receive in one go
#define MAX_NUM_TX_BYTES    100   // Maximum TX bytes to send in one go
#define WANTED_RX_BYTES     1     // Maximum TX bytes to send in one go

UART_Handle uart;
static UART_Params uartParams;

uint8_t macAddress[MAC_SIZE];
static uint32_t wantedRxBytes = WANTED_RX_BYTES;            // Number of bytes received so far
static uint8_t rxBuf[MAX_NUM_RX_BYTES];   // Receive buffer
bool enable_UART_DEBUG = false;

//int16_t uart_data_send[I2S_SAMP_PER_FRAME+1];
int16_t uart_data_send[I2S_SAMP_PER_FRAME * 2 + 1 + 8]; // 2 buffers, start bytes, 2 ima_coder states

/******Uart End ******/
static void readCallback(UART_Handle handle, void *rxBuf, size_t size);
static void writeCallback(UART_Handle handle_uart, void *rxBuf, size_t size);

static void bufRdy_callback(I2SCC26XX_Handle handle, I2SCC26XX_StreamNotification *pStreamNotification);
static void AudioDuplex_enableCache();
static void AudioDuplex_disableCache();
static void encrypt_packet(uint8_t *packet);
static void decrypt_packet(uint8_t *packet);


#ifdef LOGGING
static uint8_t received_SID[SID_LENGTH] = {0};
struct event_indicator_struct_BUF_status event_BUF_status_message;
struct event_indicator_struct_BLE event_BLE_message;
uint8_t BUF_status_message_UART_buffer[BASE64_SIZE(sizeof(event_BUF_status_message))];
uint8_t BLE_message_UART_buffer[BASE64_SIZE(sizeof(event_BLE_message))];

static void update_UART_Messages (uint8_t message_type);
#endif

/* I2C variables *****************************************************/
static uint16_t i2c_read_delay;
/*********************************************************************/
uint32_t battery_voltage;
int stream_on = 0;
extern bool enable_blink;

extern PIN_Handle ledPinHandle;

uint32_t counter_packet_send = 0;
uint32_t counter_packet_received = 0;
float packet_lost = 0;
uint32_t error_counter_packet_send = 0;
uint8_t send_status = 0;

uint8_t send_array[DS_STREAM_OUTPUT_LEN];
uint8_t current_volume = INIT_GAIN;

/* I2S variables START*/

static I2SCC26XX_StreamNotification i2sStream;
static I2SCC26XX_BufferRelease bufferRelease;
static I2SCC26XX_StreamNotification i2sStream;
static I2SCC26XX_BufferRequest bufferRequest;
static I2SCC26XX_Handle i2sHandle = (I2SCC26XX_Handle)&(I2SCC26XX_config);
static uint8_t  i2sQueueMemory[I2S_TOTAL_QUEUE_MEM_SZ];
static uint16_t i2sSampleBuffer[I2S_SAMPLE_MEMORY_SZ];

static I2SCC26XX_Params i2sParams =
{
    .requestMode            = I2SCC26XX_CALLBACK_MODE,
    .ui32requestTimeout     = BIOS_WAIT_FOREVER,
    .callbackFxn            = bufRdy_callback,
    .blockSize              = FRAME_SIZE,//I2S_SAMP_PER_FRAME,
    .pvContBuffer           = (void *)i2sSampleBuffer,
    .ui32conBufTotalSize    = (sizeof(int16_t) * I2S_SAMPLE_MEMORY_SZ),
    .pvContMgtBuffer        = (void *)i2sQueueMemory,
    .ui32conMgtBufTotalSize = I2S_TOTAL_QUEUE_MEM_SZ,
    .currentStream          = &i2sStream
};

uint8_t packet_data[TRANSMIT_DATA_LENGTH];
int16_t raw_data_received[I2S_SAMP_PER_FRAME];
int16_t mic_data_1ch[I2S_SAMP_PER_FRAME];
int16_t filtered_data[I2S_SAMP_PER_FRAME];

bool gotBufferIn = false;
bool gotBufferInOut = false;
bool gotBufferOut = false;

extern bool button_check;
uint8_t global_key[KEY_SIZE] = {0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6,
                        0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C};




/* I2S variables END*/
static Mailbox_Handle mailbox;
static uint8_t mailpost_usage;
/* codec */
extern pzConnRec_t connList[MAX_NUM_BLE_CONNS];
extern bool connection_occured;
extern List_List paramUpdateList;


static ima_state ima_Encode_state;
static ima_state ima_Decode_state;
static uint32_t previous_receive_counter = 0;
/* codec end */

extern Clock_Handle ADC_ChannelSwitchClockHandle;

#ifdef LOGGING
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
#endif

void rt_OneStep(void);

void rt_OneStep(void)
  {
    static boolean_T OverrunFlag = false;

    /* Disable interrupts here */

    /* Check for overrun */
    if (OverrunFlag) {
      return;
    }

    OverrunFlag = true;

    /* Save FPU context here (if necessary) */
    /* Re-enable timer or interrupt here */
    /* Set model inputs here */

    /* Step the model */
    LPF_step();

    /* Get model outputs here */

    /* Indicate task complete */
    OverrunFlag = false;

    /* Disable interrupts here */
    /* Restore FPU context here (if necessary) */
    /* Enable interrupts here */
  }

void start_voice_handle(void)
{
    max9860_I2C_Shutdown_state(0);//disable shutdown_mode
    //ProjectZero_enqueueMsg(PZ_APP_MSG_Load_vol, NULL);// read global vol level
    osal_snv_read(INIT_VOL_ADDR, 1, &current_volume);
    PIN_setOutputValue(ledPinHandle, Board_PIN_GLED, 1);
    GPTimerCC26XX_setLoadValue(samp_tim_hdl, (GPTimerCC26XX_Value)SAMP_TIME);
    GPTimerCC26XX_start(samp_tim_hdl);
    GPTimerCC26XX_start(measure_tim_hdl);
    I2SCC26XX_startStream(i2sHandle);
    stream_on = 1;
}


void stop_voice_handle(void)
{
    GPTimerCC26XX_stop(samp_tim_hdl);
    GPTimerCC26XX_stop(measure_tim_hdl);
    max9860_I2C_Shutdown_state(1);//enable shutdown_mode
    if(stream_on)
    {
        if(!I2SCC26XX_stopStream(i2sHandle))
        {
           while(1);
        }
    }

    PIN_setOutputValue(ledPinHandle, Board_PIN_GLED, 0);
    stream_on = 0;
    while(mailpost_usage > 0)
    {
        Mailbox_pend(mailbox, packet_data, BIOS_NO_WAIT);
        mailpost_usage = Mailbox_getNumPendingMsgs(mailbox);
    }

    memset ( packet_data,   0, sizeof(packet_data) );
    memset ( raw_data_received, 0, sizeof(raw_data_received) );
    memset ( mic_data_1ch, 0, sizeof(mic_data_1ch));
    memset ( &rtDW, 0, sizeof(rtDW) );
    /* save current volume level */
    //ProjectZero_enqueueMsg(PZ_APP_MSG_Write_vol, NULL);
    osal_snv_write(INIT_VOL_ADDR, 1, &current_volume);
#ifdef LOGGING
    memset(received_SID, 0, SID_LENGTH);
    memset(&event_BLE_message, 0, sizeof(event_BLE_message)) ;
    memset(&event_BUF_status_message, 0, sizeof(event_BUF_status_message)) ;
#endif
}


void HandsFree_init (void)
{
    buttons_init();
    power_battery_init();
    LPF_initialize();
    Echo_cancel_initialize();
    max9860_I2C_Init();
    max9860_I2C_Read_Status();
    GPTimerCC26XX_Params_init(&tim_params);
    tim_params.width = GPT_CONFIG_32BIT;
    tim_params.mode = GPT_MODE_PERIODIC_UP;
    tim_params.debugStallMode = GPTimerCC26XX_DEBUG_STALL_ON;
    blink_tim_hdl = GPTimerCC26XX_open(Board_GPTIMER2A, &tim_params);

    /* init read of volume level */
    uint8 status;
    status = osal_snv_read(INIT_VOL_ADDR, 1, &current_volume);
    if(status != SUCCESS)
    {/* write value if first run*/
        current_volume = INIT_GAIN;
        status = osal_snv_write(INIT_VOL_ADDR, 1, &current_volume);
    }

    if (blink_tim_hdl == NULL) {
        while (1);
    }
    GPTimerCC26XX_setLoadValue(blink_tim_hdl, (GPTimerCC26XX_Value)LOW_STATE_TIME);
    GPTimerCC26XX_registerInterrupt(blink_tim_hdl, blink_timer_callback, GPT_INT_TIMEOUT);
    GPTimerCC26XX_start(blink_tim_hdl);

    samp_tim_hdl = GPTimerCC26XX_open(Board_GPTIMER3A, &tim_params);
    if (samp_tim_hdl == NULL) {
        while (1);
    }
    GPTimerCC26XX_setLoadValue(samp_tim_hdl, (GPTimerCC26XX_Value)SAMP_TIME);
    GPTimerCC26XX_registerInterrupt(samp_tim_hdl, samp_timer_callback, GPT_INT_TIMEOUT);

    measure_tim_hdl = GPTimerCC26XX_open(Board_GPTIMER1A, &tim_params);
    if (measure_tim_hdl == NULL) {
        while (1);
    }

    I2SCC26XX_init(i2sHandle);
    I2SCC26XX_Handle i2sHandleTmp = NULL;
    AudioDuplex_disableCache();

    // Reset I2S handle and attempt to open
    i2sHandle = (I2SCC26XX_Handle)&(I2SCC26XX_config);
    i2sHandleTmp = I2SCC26XX_open(i2sHandle, &i2sParams);

    if(!i2sHandleTmp)
    {
        while(1);
    }
    ProjectZero_enqueueMsg(PZ_I2C_Read_status_EVT, NULL);
    mailbox = Mailbox_create(sizeof(packet_data), MAILBOX_DEPTH, NULL, NULL);
    if (mailbox == NULL) {
        while (1);
    }

    UART_init();
    parser_init();

   // UartLog_init(UART_open(Board_UART0, NULL));
    /* Create a UART with data processing off. */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode    = UART_DATA_BINARY;
    uartParams.readDataMode     = UART_DATA_BINARY;
    uartParams.readMode         = UART_MODE_CALLBACK;
    uartParams.writeMode        = UART_MODE_CALLBACK;
    //uartParams.writeTimeout      = 0; //UART_WAIT_FOREVER
    uartParams.readCallback     = readCallback;
    uartParams.writeCallback    = writeCallback;
    uartParams.readReturnMode   = UART_RETURN_FULL;
    uartParams.readEcho         = UART_ECHO_OFF;
    uartParams.baudRate         = UART_BAUD_RATE;

    uart = UART_open(Board_UART0, &uartParams);
    if (uart == NULL) {
        /* UART_open() failed */
        while (1);
    }

    uint64_t temp = *((uint64_t *)(0x500012E8)) & 0xFFFFFFFFFFFFFF;
    for(uint8_t i = 0 ; i < 6 ; i++)
    {
        macAddress[i]=*(((uint8_t *)&temp)+i);
    }
    //macAddress = *((uint64_t *)(0x500012E8)) & 0xFFFFFFFFFFFFFF;

    UART_write(uart, macAddress, sizeof(macAddress));
    int rxBytes = UART_read(uart, rxBuf, wantedRxBytes);

   AESCBC_init();
    /* Open AESCCM_open */
    AESCBCHandle = AESCBC_open(0, NULL);

    if (!AESCBCHandle) {
        /* AESECM_open_open() failed */
        while(1);
    }
    /* Initialize the key structure */
    read_aes_key(global_key);
    CryptoKeyPlaintext_initKey(&cryptoKey, (uint8_t*) global_key, sizeof(global_key));
#ifdef LOGGING
    for(uint16_t i = 0 ; i < sizeof(event_BUF_status_message.MAC_addr) ; i++)
    {
        event_BUF_status_message.MAC_addr[i] = macAddress[i];
        event_BLE_message.MAC_addr[i] = macAddress[i];
    }
    event_BUF_status_message.null_terminator = 0x00;
    event_BLE_message.null_terminator = 0x00;
    event_BUF_status_message.message_type = RECEIVE_BUFFER_STATUS;
#endif
    /* BLE optimization */
    HCI_EXT_OverlappedProcessingCmd(HCI_EXT_ENABLE_OVERLAPPED_PROCESSING);
    HCI_EXT_HaltDuringRfCmd( HCI_EXT_HALT_DURING_RF_DISABLE ); //Enable CPU during RF events (scan included) - may increase power consumption
    HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_DISABLE_CLK_DIVIDE_ON_HALT ); //Set whether the system clock will be divided when the MCU is halted. - may increase power consumption
    HCI_EXT_SetFastTxResponseTimeCmd(HCI_EXT_ENABLE_FAST_TX_RESP_TIME); // configure the Link Layer fast transmit response time feature
}


void blink_timer_callback(GPTimerCC26XX_Handle handle, GPTimerCC26XX_IntMask interruptMask)
{
    static bool blink = false;
    static bool bat_low = false;

    if(button_check)
    {
        ProjectZero_enqueueMsg(PZ_APP_MSG_Read_ADC_Power_Button_Voltage, NULL);
    }
    battery_voltage = get_bat_voltage();
    if(blink)
    {
        blink = false;
        GPTimerCC26XX_setLoadValue(blink_tim_hdl, load_val[0]);

        if(!bat_low)
        {
            bat_low = battery_voltage < BAT_LOW_VOLTAGE;
        }
        else
        {
            PIN_setOutputValue(ledPinHandle, Board_PIN_RLED, 0);
        }

        if(!stream_on)
        {
            PIN_setOutputValue(ledPinHandle, Board_PIN_GLED, 0);
        }
    }
    else
    {
        blink = true;
        GPTimerCC26XX_setLoadValue(blink_tim_hdl, load_val[1]);

        if(bat_low)
        {
            PIN_setOutputValue(ledPinHandle, Board_PIN_RLED, 1);
        }

         if(!stream_on)
        {
            PIN_setOutputValue(ledPinHandle, Board_PIN_GLED, 1);
        }
    }

    if(!enable_blink)
    {
        PIN_setOutputValue(ledPinHandle, Board_PIN_RLED, 1);
        PIN_setOutputValue(ledPinHandle, Board_PIN_GLED, 0);
    }
}

void samp_timer_callback(GPTimerCC26XX_Handle handle, GPTimerCC26XX_IntMask interruptMask)
{
    if(stream_on)
    {
        ProjectZero_enqueueMsg(PZ_SEND_PACKET_EVT, NULL);
    }
}

void USER_task_Handler (pzMsg_t *pMsg)
{
    // Cast to pzCharacteristicData_t* here since it's a common message pdu type.
    pzCharacteristicData_t *pCharData = (pzCharacteristicData_t *)pMsg->pData;
    uint8_t status;

    switch(pMsg->event)
    {
        case PZ_SERVICE_WRITE_EVT: /* Message about received value write */
        {
            /* Call different handler per service */
            switch(pCharData->svcUUID)
            {
    //        case LED_SERVICE_SERV_UUID:
    //            ProjectZero_LedService_ValueChangeHandler(pCharData);
    //            break;
              case DATA_SERVICE_SERV_UUID:
                  ProjectZero_DataService_ValueChangeHandler(pCharData);
                  break;
            }
        }
        break;

        case PZ_SERVICE_CFG_EVT: /* Message about received CCCD write */
            /* Call different handler per service */
            switch(pCharData->svcUUID)
            {
    //            case BUTTON_SERVICE_SERV_UUID:
    //                ProjectZero_ButtonService_CfgChangeHandler(pCharData);
    //                break;
              case DATA_SERVICE_SERV_UUID:
                  ProjectZero_DataService_CfgChangeHandler(pCharData);
                  break;
            }
            break;
        case PZ_SEND_PACKET_EVT:
        {
            mailpost_usage = Mailbox_getNumPendingMsgs(mailbox);
            if(mailpost_usage>0)
            {
                if(previous_receive_counter != (counter_packet_received - 1))
                {
                   // memset(&ima_Decode_state, 0, sizeof(ima_Decode_state));
                }
                Mailbox_pend(mailbox, packet_data, BIOS_NO_WAIT);

                timestamp_decode_start =  GPTimerCC26XX_getValue(measure_tim_hdl);
                decrypt_packet(packet_data);
                int16_t temp_current = packet_data[V_STREAM_OUTPUT_SOUND_LEN + 1] | packet_data[V_STREAM_OUTPUT_SOUND_LEN] << 8;

                ima_Decode_state.current = (int32_t)temp_current;
                ima_Decode_state.stepindex = packet_data[V_STREAM_OUTPUT_SOUND_LEN + 2];
                ima_decode_mono(&ima_Decode_state, raw_data_received, packet_data, FRAME_SIZE);

                timestamp_decode_stop =  GPTimerCC26XX_getValue(measure_tim_hdl);
                timestamp_decode_dif = timestamp_decode_stop - timestamp_decode_start;

                previous_receive_counter = counter_packet_received;
                if(enable_LPF)//500000
                {
                    timestamp_LPF_start =  GPTimerCC26XX_getValue(measure_tim_hdl);
                    for(uint16_t i = 0 ; i< I2S_SAMP_PER_FRAME; i++)
                    {
                        rtU.In1 = raw_data_received[i];
                        rt_OneStep();
                        raw_data_received[i] = rtY.Out1;
                    }
                    timestamp_LPF_stop =  GPTimerCC26XX_getValue(measure_tim_hdl);
                    timestamp_LPF_dif = timestamp_LPF_stop - timestamp_LPF_start;
                }
            }else{
                memset ( packet_data,   0, sizeof(packet_data) );
                memset ( raw_data_received, 0, sizeof(raw_data_received));
                memset ( mic_data_1ch,  0, sizeof(mic_data_1ch));
            }

            bufferRequest.buffersRequested = I2SCC26XX_BUFFER_IN_AND_OUT;

            gotBufferInOut = I2SCC26XX_requestBuffer(i2sHandle, &bufferRequest);
            if (gotBufferInOut)
            {
                memcpy(bufferRequest.bufferOut, raw_data_received, sizeof(raw_data_received) / 2 );
                memcpy(mic_data_1ch, bufferRequest.bufferIn, sizeof(mic_data_1ch) / 2 );

                bufferRelease.bufferHandleOut = bufferRequest.bufferHandleOut;
                bufferRelease.bufferHandleIn = bufferRequest.bufferHandleIn;
                I2SCC26XX_releaseBuffer(i2sHandle, &bufferRelease);
                gotBufferInOut = 0;
                i2c_read_delay++;
            }

            bufferRequest.buffersRequested = I2SCC26XX_BUFFER_IN_AND_OUT;
            gotBufferInOut = I2SCC26XX_requestBuffer(i2sHandle, &bufferRequest);
            if (gotBufferInOut)
            {
                memcpy(bufferRequest.bufferOut, &raw_data_received[160], sizeof(raw_data_received) / 2 );
                memcpy(&mic_data_1ch[160], bufferRequest.bufferIn, sizeof(mic_data_1ch) / 2);
                timestamp_EC_start =  GPTimerCC26XX_getValue(measure_tim_hdl);
                if(enable_EC)
                {
                    for(uint16_t i = 0 ; i< I2S_SAMP_PER_FRAME; i++)
                    {
                        rtUeC.EnableeC = TRUE;
                        rtUeC.ReseteC = FALSE;
                        rtUeC.BLE_receiveeC = raw_data_received[i];
                        rtUeC.MIC_dataeC = mic_data_1ch[i];
                        Echo_cancel_step();
                        mic_data_1ch[i] = rtYeC.OutputeC;
                    }
                    timestamp_EC_stop =  GPTimerCC26XX_getValue(measure_tim_hdl);
                    timestamp_EC_dif = timestamp_EC_stop - timestamp_EC_start;
                }
                else
                {
                    rtUeC.ReseteC = TRUE;
                    Echo_cancel_step();
                }
                if(enable_NoiseGate)
                {
                    timestamp_NG_start =  GPTimerCC26XX_getValue(measure_tim_hdl);
                    in_power = power_calculation(mic_data_1ch, I2S_SAMP_PER_FRAME);//52000 ticks
                    amplify (mic_data_1ch, I2S_SAMP_PER_FRAME, (int16_t)in_power.power_log);

                    timestamp_NG_stop =  GPTimerCC26XX_getValue(measure_tim_hdl);
                    timestamp_NG_dif = timestamp_NG_stop - timestamp_NG_start;
                }
                bufferRelease.bufferHandleOut = bufferRequest.bufferHandleOut;
                bufferRelease.bufferHandleIn = bufferRequest.bufferHandleIn;
                I2SCC26XX_releaseBuffer(i2sHandle, &bufferRelease);
                gotBufferInOut = 0;
                i2c_read_delay++;
            }

            timestamp_encode_start =  GPTimerCC26XX_getValue(measure_tim_hdl);


            send_array[V_STREAM_OUTPUT_SOUND_LEN] =     (uint8_t)(ima_Encode_state.current >> 8);
            send_array[V_STREAM_OUTPUT_SOUND_LEN + 1] = (uint8_t)(ima_Encode_state.current & 0xFF);
            send_array[V_STREAM_OUTPUT_SOUND_LEN + 2] = (uint8_t)ima_Encode_state.stepindex;
            ima_encode_mono(&ima_Encode_state, send_array, mic_data_1ch, sizeof(mic_data_1ch));


            send_array[TRANSMIT_DATA_LENGTH - 4] = (uint8_t)(counter_packet_send >> 24);
            send_array[TRANSMIT_DATA_LENGTH - 3] = (uint8_t)(counter_packet_send >> 16);
            send_array[TRANSMIT_DATA_LENGTH - 2] = (uint8_t)(counter_packet_send >> 8);
            send_array[TRANSMIT_DATA_LENGTH - 1] = (uint8_t)(counter_packet_send);

            timestamp_encode_stop =  GPTimerCC26XX_getValue(measure_tim_hdl);
            timestamp_encode_dif = timestamp_encode_stop - timestamp_encode_start;

            encrypt_packet(send_array);
            send_status = DataService_SetParameter(DS_STREAM_OUTPUT_ID, DS_STREAM_OUTPUT_LEN, send_array);
            if((send_status != SUCCESS)/* || (send_status == 0x15)*/)
            {
                error_counter_packet_send++;
#ifdef LOGGING
                update_UART_Messages(PACKET_SENT_ERROR_TYPE);

                ProjectZero_enqueueMsg(PZ_APP_MSG_Send_message_BLE, NULL);
                ProjectZero_enqueueMsg(PZ_APP_MSG_Send_message_Buf_Status, NULL);
#endif
            }
            else
            {
#ifdef LOGGING
                update_UART_Messages(PACKET_SENT_MESSAGE_TYPE);

                ProjectZero_enqueueMsg(PZ_APP_MSG_Send_message_BLE, NULL);
                ProjectZero_enqueueMsg(PZ_APP_MSG_Send_message_Buf_Status, NULL);
#endif
            }


            counter_packet_send++;
            if(enable_UART_DEBUG)
            {
                memcpy(&uart_data_send[1],                            mic_data_1ch,      sizeof(mic_data_1ch));
                memcpy(&uart_data_send[I2S_SAMP_PER_FRAME + 1],       raw_data_received,     sizeof(raw_data_received));
                memcpy(&uart_data_send[I2S_SAMP_PER_FRAME*2 + 1],     &ima_Encode_state,     sizeof(ima_Encode_state));
                memcpy(&uart_data_send[I2S_SAMP_PER_FRAME*2 + 1 + 4], &ima_Decode_state,     sizeof(ima_Decode_state));
                //memcpy(&uart_data_send[I2S_SAMP_PER_FRAME * 2 + 1], raw_data_received, sizeof(raw_data_received));
                uart_data_send[0]= (40u << 8u) + 41u;   //start bytes for MATLAB ")("
                UART_write(uart, uart_data_send, sizeof(uart_data_send));
            }
        }
        break;

        case PZ_I2C_Read_status_EVT:
        {
            max9860_I2C_Read_Status();
        }
        break;

        case PZ_SEND_START_STREAM_EVT:
        {
            error_counter_packet_send = 0;
            counter_packet_send = 0;
            counter_packet_received = 0;
            start_voice_handle();
        }
        break;

        case PZ_SEND_STOP_STREAM_EVT:
        {
            stop_voice_handle();
        }
        break;

        case PZ_BUTTON_DEBOUNCED_EVT: /* Message from swi about pin change */
        {
            pzButtonState_t *pButtonState = (pzButtonState_t *)pMsg->pData;
            Handler_ButtonPress(pButtonState);
        }
        break;

        case PZ_APP_MSG_Read_ADC_Battery_Voltage:
        {
            adc_conversion.adcChannel = ADC_VOLTAGE_MEASURE_PIN;
            button_check = FALSE;
            if (ADCBuf_convert(adc_hdl, &adc_conversion, 1) != ADCBuf_STATUS_SUCCESS)
            {
                while(1);
            }
            /* return Power button monitor ADC channel after ADC_SWITCH_TIMEOUT*/
            Util_startClock((Clock_Struct *)ADC_ChannelSwitchClockHandle);
        }
        break;

        case PZ_APP_MSG_Read_ADC_Battery_Voltage_UART:
        {
            adc_conversion.adcChannel = ADC_VOLTAGE_MEASURE_PIN;
            button_check = FALSE;
            if (ADCBuf_convert(adc_hdl, &adc_conversion, 1) != ADCBuf_STATUS_SUCCESS)
            {
                while(1);
            }
            get_fh_param();
            /* return Power button monitor ADC channel after ADC_SWITCH_TIMEOUT*/
            Util_startClock((Clock_Struct *)ADC_ChannelSwitchClockHandle);
        }
        break;

        case PZ_APP_MSG_Read_ADC_Power_Button_Voltage:
        {
            adc_conversion.adcChannel = ADC_POWER_BUTTON_PIN;
            if (ADCBuf_convert(adc_hdl, &adc_conversion, 1) != ADCBuf_STATUS_SUCCESS)
            {
                while(1);
            }
        }
        break;

        case PZ_APP_MSG_Load_vol:
        {
            status = osal_snv_read(INIT_VOL_ADDR, 1, &current_volume);
            if(status != SUCCESS)
            {
                current_volume = INIT_GAIN;
            }
        }
        break;

        case PZ_APP_MSG_Write_vol:
        {
            osal_snv_write(INIT_VOL_ADDR, 1, &current_volume);
        }
        break;
#ifdef LOGGING
        case PZ_APP_MSG_Send_message_BLE:
        {
            base64_encode((char *)BLE_message_UART_buffer, (int32_t)BASE64_SIZE(sizeof(BLE_message_UART_buffer)), (const uint8_t *)&event_BLE_message, sizeof(event_BLE_message));
            UART_write(uart, BLE_message_UART_buffer, sizeof(BLE_message_UART_buffer));
        }
        break;

        case PZ_APP_MSG_Send_message_Buf_Status:
        {
            base64_encode((char *)BUF_status_message_UART_buffer, (int32_t)BASE64_SIZE(sizeof(BUF_status_message_UART_buffer)), (const uint8_t *)&event_BUF_status_message, sizeof(event_BUF_status_message));
            UART_write(uart, BUF_status_message_UART_buffer, sizeof(BUF_status_message_UART_buffer));
        }
        break;
#endif
        case PZ_APP_MSG_Read_key:
            get_fh_key();
        break;

        case PZ_APP_MSG_Write_key:
            send_fh_key();
        break;
        default:
        break;
    }
}


/*
 * @brief   Handle a CCCD (configuration change) write received from a peer
 *          device. This tells us whether the peer device wants us to send
 *          Notifications or Indications.
 *
 * @param   pCharData  pointer to malloc'd char write data
 *
 * @return  None.
 */
void ProjectZero_DataService_CfgChangeHandler(pzCharacteristicData_t *pCharData)
{
    // Cast received data to uint16, as that's the format for CCCD writes.
    uint16_t configValue = *(uint16_t *)pCharData->data;

    switch(pCharData->paramID)
    {
    case DS_STREAM_OUTPUT_ID:
        if(connection_occured)
        {
            if (configValue) // 0x0001 and 0x0002 both indicate turned on.
            {
                if(stream_on != 1)
                {
                    ProjectZero_enqueueMsg(PZ_SEND_START_STREAM_EVT, NULL);
                }
            }
            else
            {
                ProjectZero_enqueueMsg(PZ_SEND_STOP_STREAM_EVT, NULL);
            }
        }
        break;
    }
}

/*
 * @brief   Handle a write request sent from a peer device.
 *
 *          Invoked by the Task based on a message received from a callback.
 *
 *          When we get here, the request has already been accepted by the
 *          service and is valid from a BLE protocol perspective as well as
 *          having the correct length as defined in the service implementation.
 *
 * @param   pCharData  pointer to malloc'd char write data
 *
 * @return  None.
 */
void ProjectZero_DataService_ValueChangeHandler(
    pzCharacteristicData_t *pCharData)
{
    // Value to hold the received string for printing via Log, as Log printouts
    // happen in the Idle task, and so need to refer to a global/static variable.


    switch(pCharData->paramID)
    {
    case DS_STREAM_START_ID:
        // Do something useful with pCharData->data here
        // -------------------------
        // Copy received data to holder array, ensuring NULL termination.
#ifdef LOGGING
        memset(received_SID, 0, SID_LENGTH);
        memcpy(received_SID, pCharData->data, SID_LENGTH);
        for(uint16_t i = 0 ; i < sizeof(event_BUF_status_message.SID); i++)
        {
            event_BUF_status_message.SID[i] = received_SID[i];
            event_BLE_message.SID[i] = received_SID[i];
        }
#endif
        break;

    case DS_STREAM_INPUT_ID:
        Mailbox_post(mailbox, pCharData->data, BIOS_NO_WAIT);
        counter_packet_received++;
#ifdef LOGGING
        update_UART_Messages(PACKET_RECEIVED_MESSAGE_TYPE);

        ProjectZero_enqueueMsg(PZ_APP_MSG_Send_message_BLE, NULL);
        ProjectZero_enqueueMsg(PZ_APP_MSG_Send_message_Buf_Status, NULL);
#endif
        packet_lost = 100.0f * ((float)counter_packet_send - (float)counter_packet_received) / (float)counter_packet_send;
        // -------------------------
        // Do something useful with pCharData->data here
        break;

    default:
        return;
    }
}

#ifdef LOGGING
static void update_UART_Messages (uint8_t message_type)
{
    uint32_t timestamp = GPTimerCC26XX_getValue(measure_tim_hdl);
    event_BLE_message.message_type = message_type;
    event_BLE_message.packet_number = counter_packet_received;
    event_BLE_message.timestamp = timestamp;

    event_BUF_status_message.buff_status = Mailbox_getNumPendingMsgs(mailbox);
    event_BUF_status_message.timestamp = timestamp;
}
#endif

static void bufRdy_callback(I2SCC26XX_Handle handle, I2SCC26XX_StreamNotification *pStreamNotification)
{
    I2SCC26XX_Status streamStatus = pStreamNotification->status;

    if (streamStatus == I2SCC26XX_STREAM_BUFFER_READY || streamStatus == I2SCC26XX_STREAM_BUFFER_READY_BUT_NO_AVAILABLE_BUFFERS)
    {
//        gotBufferOut = true;
//        gotBufferIn = true;
//        gotBufferInOut = true;

    }
}

/*********************************************************************
 * @fn      AudioDuplex_disableCache
 *
 * @brief   Disables the instruction cache and sets power constaints
 *          This prevents the device from sleeping while streaming
 *
 * @param   None.
 *
 * @return  None.
 */
static void AudioDuplex_disableCache()
{
    uint_least16_t hwiKey = Hwi_disable();
    Power_setConstraint(PowerCC26XX_SB_VIMS_CACHE_RETAIN);
    Power_setConstraint(PowerCC26XX_NEED_FLASH_IN_IDLE);
    VIMSModeSafeSet(VIMS_BASE, VIMS_MODE_DISABLED, true);
    Hwi_restore(hwiKey);
}

/*********************************************************************
 * @fn      AudioDuplex_enableCache
 *
 * @brief   Enables the instruction cache and releases power constaints
 *          Allows device to sleep again
 *
 * @param   None.
 *
 * @return  None.
 */
static void AudioDuplex_enableCache()
{
    uint_least16_t hwiKey = Hwi_disable();
    Power_releaseConstraint(PowerCC26XX_SB_VIMS_CACHE_RETAIN);
    Power_releaseConstraint(PowerCC26XX_NEED_FLASH_IN_IDLE);
    VIMSModeSafeSet(VIMS_BASE, VIMS_MODE_ENABLED, true);
    Hwi_restore(hwiKey);
}

static void writeCallback(UART_Handle handle_uart, void *rxBuf, size_t size)
{
//SPPBLEServer_enqueueUARTMsg(SBC_UART_CHANGE_EVT,rxBuf,size);
}

static void readCallback(UART_Handle handle, void *rxBuf, size_t size)
{
 //   memset(&test_CRC,0,sizeof(test_CRC));

    OnRxByte(((unsigned char*)rxBuf)[0]);
    if(PackProcessing())
    {

    }

    wantedRxBytes = 1;
    UART_read(handle, rxBuf, wantedRxBytes);
}


static void encrypt_packet(uint8_t *packet)
{
    int_fast16_t status;
    uint8_t tmp_packet[V_STREAM_OUTPUT_SOUND_LEN + PACKET_CODEC_META_DATA];
    memset(tmp_packet, 0, V_STREAM_OUTPUT_SOUND_LEN + PACKET_CODEC_META_DATA);
    memcpy(tmp_packet, packet, V_STREAM_OUTPUT_SOUND_LEN + PACKET_CODEC_META_DATA);

    CryptoKeyPlaintext_initKey(&cryptoKey, (uint8_t*) global_key, sizeof(global_key));

    /* Perform a single step encrypt operation of the plain text */
    AESCBC_Operation_init(&operationOneStepEncrypt);
    operationOneStepEncrypt.key            = &cryptoKey;
    operationOneStepEncrypt.input          = packet;
    operationOneStepEncrypt.output         = tmp_packet;
    operationOneStepEncrypt.inputLength    = V_STREAM_OUTPUT_SOUND_LEN ;

    status = AESCBC_oneStepEncrypt(AESCBCHandle, &operationOneStepEncrypt);

    if(status != AESCBC_STATUS_SUCCESS)
    {
        while(1);
    }
    memcpy(packet, tmp_packet, V_STREAM_OUTPUT_SOUND_LEN);

    AESCBC_Operation_init(&operationOneStepEncrypt);
    operationOneStepEncrypt.key            = &cryptoKey;
    operationOneStepEncrypt.input          = &tmp_packet[147];
    operationOneStepEncrypt.output         = &packet[147];
    operationOneStepEncrypt.inputLength    = KEY_SIZE ;

    status = AESCBC_oneStepEncrypt(AESCBCHandle, &operationOneStepEncrypt);

    if(status != AESCBC_STATUS_SUCCESS)
    {
        while(1);
    }
}

static void decrypt_packet(uint8_t *packet)
{
    int_fast16_t status;
    uint8_t tmp_packet[V_STREAM_OUTPUT_SOUND_LEN + PACKET_CODEC_META_DATA];
    memcpy(tmp_packet, packet, V_STREAM_OUTPUT_SOUND_LEN + PACKET_CODEC_META_DATA);

    CryptoKeyPlaintext_initKey(&cryptoKey, (uint8_t*) global_key, sizeof(global_key));

    AESCBC_Operation_init(&operationOneStepDecrypt);
    operationOneStepDecrypt.key            = &cryptoKey;
    operationOneStepDecrypt.input          = &packet[147];
    operationOneStepDecrypt.output         = &tmp_packet[147];
    operationOneStepDecrypt.inputLength    = KEY_SIZE;

    status = AESCBC_oneStepDecrypt(AESCBCHandle, &operationOneStepDecrypt);

    if(status != AESCBC_STATUS_SUCCESS)
    {
        while(1);
    }
    memcpy(packet, tmp_packet, V_STREAM_OUTPUT_SOUND_LEN + PACKET_CODEC_META_DATA);

    AESCBC_Operation_init(&operationOneStepDecrypt);
    operationOneStepDecrypt.key            = &cryptoKey;
    operationOneStepDecrypt.input          = tmp_packet;
    operationOneStepDecrypt.output         = packet;
    operationOneStepDecrypt.inputLength    = V_STREAM_OUTPUT_SOUND_LEN;

    status = AESCBC_oneStepDecrypt(AESCBCHandle, &operationOneStepDecrypt);

    if(status != AESCBC_STATUS_SUCCESS)
    {
        while(1);
    }

}

uint8_t read_aes_key(uint8_t *key)
{
    uint8_t status;
    static uint8_t default_key[] =
        {0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6,
        0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C};

    status = osal_snv_read(KEY_SNV_ID, KEY_SIZE, key);
    if(status != SUCCESS)
    {
        memcpy(global_key, default_key, KEY_SIZE);
    }

    return status;
}

uint8_t write_aes_key(uint8_t *key)
{
    CryptoKeyPlaintext_initKey(&cryptoKey, (uint8_t*) key, sizeof(*key));
    return (osal_snv_write(KEY_SNV_ID, KEY_SIZE, key));
}
