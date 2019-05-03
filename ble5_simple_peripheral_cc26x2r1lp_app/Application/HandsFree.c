/*
 * HandsFree.c
 *
 *  Created on: 24 янв. 2019 г.
 *      Author: CIT_007
 */
#include "HandsFree.h"
#include "Noise_TRSH.h"
#include "Uart_commands.h"
#include "max9860_i2c.h"
#include "I2S/I2SCC26XX.h"
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include "codec/SitADPCM.h"
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
#ifdef  LPF
  #include "../LPF/LPF.h"                       /* Model's header file */
  #include "../LPF/rtwtypes.h"
#endif

#define ABS(a) (((a)<0)?-(a):a)
/* Timer variables ***************************************************/
GPTimerCC26XX_Params tim_params;
GPTimerCC26XX_Handle blink_tim_hdl = NULL;
GPTimerCC26XX_Handle measure_tim_hdl = NULL;
GPTimerCC26XX_Handle samp_tim_hdl = NULL;
GPTimerCC26XX_Value load_val[2] = {LOW_STATE_TIME, HIGH_STATE_TIME};
/*********************************************************************/

/* Debug time variables ***************************************************/
GPTimerCC26XX_Value timestamp_encode_start;
GPTimerCC26XX_Value timestamp_encode_stop;
GPTimerCC26XX_Value timestamp_encode_dif;
GPTimerCC26XX_Value timestamp_decode_start;
GPTimerCC26XX_Value timestamp_decode_stop;
GPTimerCC26XX_Value timestamp_decode_dif;
/*********************************************************************/
#ifdef  LPF
    GPTimerCC26XX_Value timestamp_LPF_start;
    GPTimerCC26XX_Value timestamp_LPF_stop;
    GPTimerCC26XX_Value timestamp_LPF_dif;
    GPTimerCC26XX_Value timestamp_DECIM_start;
    GPTimerCC26XX_Value timestamp_DECIM_stop;
    GPTimerCC26XX_Value timestamp_DECIM_dif;
#endif

/******Crypto key Start ******/
#define KEY_SNV_ID                          BLE_NVID_CUST_START
#define KEY_SIZE                            16
#define INIT_VOL_ADDR                       BLE_NVID_CUST_START+1
/******Crypto key End ******/

extern ADCBuf_Conversion adc_conversion;
extern ADCBuf_Handle adc_hdl;
/******Uart Start ******/
#ifdef UART_DEBUG
  #define UART_BAUD_RATE 921600
#endif
#ifndef UART_DEBUG
  #define UART_BAUD_RATE 115200
#endif
#define MAX_NUM_RX_BYTES    100   // Maximum RX bytes to receive in one go
#define MAX_NUM_TX_BYTES    100   // Maximum TX bytes to send in one go
#define WANTED_RX_BYTES     1     // Maximum TX bytes to send in one go

UART_Handle uart;
static UART_Params uartParams;

uint8_t macAddress[6];
static uint32_t wantedRxBytes = WANTED_RX_BYTES;            // Number of bytes received so far
static uint8_t rxBuf[MAX_NUM_RX_BYTES];   // Receive buffer

#ifdef UART_DEBUG
//int16_t uart_data_send[I2S_SAMP_PER_FRAME+1];
int16_t uart_data_send[I2S_SAMP_PER_FRAME * 2 + 1 + 8]; // 2 buffers, start bytes, 2 ima_coder states
#endif
/******Uart End ******/
void swap_array (int8_t *swap_arr);
static void readCallback(UART_Handle handle, void *rxBuf, size_t size);
static void writeCallback(UART_Handle handle_uart, void *rxBuf, size_t size);

static void bufRdy_callback(I2SCC26XX_Handle handle, I2SCC26XX_StreamNotification *pStreamNotification);
static void AudioDuplex_enableCache();
static void AudioDuplex_disableCache();


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




/* I2S variables END*/
static Mailbox_Handle mailbox;
static uint8_t mailpost_usage;
/* codec */
extern pzConnRec_t connList[MAX_NUM_BLE_CONNS];
extern bool connection_occured;
extern List_List paramUpdateList;
static struct ADPCMstate encoder_adpcm, decoder_adpcm;


static ima_state ima_Encode_state;
static ima_state ima_Decode_state;
static uint32_t previous_receive_counter = 0;
/* codec end */

extern Clock_Handle ADC_ChannelSwitchClockHandle;


#ifdef LPF
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
#endif


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
#ifdef LPF
    memset ( &rtDW, 0, sizeof(rtDW) );
#endif
    /* save current volume level */
    //ProjectZero_enqueueMsg(PZ_APP_MSG_Write_vol, NULL);
    osal_snv_write(INIT_VOL_ADDR, 1, &current_volume);
}


void HandsFree_init (void)
{
#ifdef LPF
    LPF_initialize();
#endif
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


    buttons_init();
    power_battery_init();
}


void blink_timer_callback(GPTimerCC26XX_Handle handle, GPTimerCC26XX_IntMask interruptMask)
{
    static bool blink = false;
    static bool bat_low = false;

    if(button_check)
    {
        ProjectZero_enqueueMsg(PZ_APP_MSG_Read_ADC_Power_Button_Voltage, NULL);
    }

    if(blink)
    {
        blink = false;
        GPTimerCC26XX_setLoadValue(blink_tim_hdl, load_val[0]);

        if(!bat_low)
        {
            battery_voltage = get_bat_voltage();
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
//                decoder_adpcm.prevsample = ((int16_t)(packet_data[V_STREAM_OUTPUT_SOUND_LEN]) << 8) |
//                        (int16_t)(packet_data[V_STREAM_OUTPUT_SOUND_LEN + 1]);
//
//                decoder_adpcm.previndex = ((int32_t)(packet_data[V_STREAM_OUTPUT_SOUND_LEN + 2]));
//                ADPCMDecoderBuf2((char*)(packet_data), raw_data_received, &decoder_adpcm);
//                ADPCMDecoderBuf2((char*)(&packet_data[TRANSMIT_DATA_LENGTH / 4]), &raw_data_received[I2S_SAMP_PER_FRAME / 4], &decoder_adpcm);
//                ADPCMDecoderBuf2((char*)(&packet_data[TRANSMIT_DATA_LENGTH / 2]), &raw_data_received[I2S_SAMP_PER_FRAME / 2], &decoder_adpcm);
//                ADPCMDecoderBuf2((char*)(&packet_data[TRANSMIT_DATA_LENGTH* 3 / 4]), &raw_data_received[I2S_SAMP_PER_FRAME * 3 / 4], &decoder_adpcm);


                int16_t temp_current = packet_data[V_STREAM_OUTPUT_SOUND_LEN + 1] | packet_data[V_STREAM_OUTPUT_SOUND_LEN] << 8;
                ima_Decode_state.current = (int32_t)temp_current;
                ima_Decode_state.stepindex = packet_data[V_STREAM_OUTPUT_SOUND_LEN + 2];
                ima_decode_mono(&ima_Decode_state, raw_data_received, packet_data, FRAME_SIZE);
                timestamp_decode_stop =  GPTimerCC26XX_getValue(measure_tim_hdl);
                timestamp_decode_dif = timestamp_decode_stop - timestamp_decode_start;
                previous_receive_counter = counter_packet_received;
            #ifdef  LPF
                timestamp_LPF_start =  GPTimerCC26XX_getValue(measure_tim_hdl);
                for(uint16_t i = 0 ; i< I2S_SAMP_PER_FRAME; i++)
                {
                    rtU.In1 = raw_data_received[i];
                    rt_OneStep();
                    raw_data_received[i] = rtY.Out1;
                }
                timestamp_LPF_stop =  GPTimerCC26XX_getValue(measure_tim_hdl);
                timestamp_LPF_dif = timestamp_LPF_stop - timestamp_LPF_start;
            #endif
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

                bufferRelease.bufferHandleOut = bufferRequest.bufferHandleOut;
                bufferRelease.bufferHandleIn = bufferRequest.bufferHandleIn;
                I2SCC26XX_releaseBuffer(i2sHandle, &bufferRelease);
                gotBufferInOut = 0;
                i2c_read_delay++;
            }

            timestamp_encode_start =  GPTimerCC26XX_getValue(measure_tim_hdl);


//            ADPCMEncoderBuf2(mic_data_1ch, (char*)(send_array), &encoder_adpcm);
//            ADPCMEncoderBuf2(&mic_data_1ch[I2S_SAMP_PER_FRAME / 4], (char*)(&send_array[TRANSMIT_DATA_LENGTH/4]), &encoder_adpcm);
//            ADPCMEncoderBuf2(&mic_data_1ch[I2S_SAMP_PER_FRAME / 2], (char*)(&send_array[TRANSMIT_DATA_LENGTH/2]), &encoder_adpcm);
//            ADPCMEncoderBuf2(&mic_data_1ch[I2S_SAMP_PER_FRAME * 3 / 4], (char*)(&send_array[TRANSMIT_DATA_LENGTH * 3 / 4]), &encoder_adpcm);
//
//            send_array[V_STREAM_OUTPUT_SOUND_LEN] = (uint8_t)(encoder_adpcm.prevsample >> 8);
//            send_array[V_STREAM_OUTPUT_SOUND_LEN + 1] = (uint8_t)encoder_adpcm.prevsample;
//            send_array[V_STREAM_OUTPUT_SOUND_LEN + 2] = (uint8_t)encoder_adpcm.previndex;
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

            status = DataService_SetParameter(DS_STREAM_OUTPUT_ID, DS_STREAM_OUTPUT_LEN, send_array);
            if((status != SUCCESS) || (status == 0x15))
            {
                error_counter_packet_send++;
            }
            counter_packet_send++;
            #ifdef  UART_DEBUG
                memcpy(&uart_data_send[1],                            mic_data_1ch,      sizeof(mic_data_1ch));
                memcpy(&uart_data_send[I2S_SAMP_PER_FRAME + 1],       raw_data_received,     sizeof(raw_data_received));
                memcpy(&uart_data_send[I2S_SAMP_PER_FRAME*2 + 1],     &ima_Encode_state,     sizeof(ima_Encode_state));
                memcpy(&uart_data_send[I2S_SAMP_PER_FRAME*2 + 1 + 4], &ima_Decode_state,     sizeof(ima_Decode_state));
                //memcpy(&uart_data_send[I2S_SAMP_PER_FRAME * 2 + 1], raw_data_received, sizeof(raw_data_received));
                uart_data_send[0]= (40u << 8u) + 41u;   //start bytes for MATLAB ")("
                UART_write(uart, uart_data_send, sizeof(uart_data_send));
            #endif
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
    static uint8_t received_string[DS_STREAM_START_LEN] = {0};

    switch(pCharData->paramID)
    {
    case DS_STREAM_START_ID:
        // Do something useful with pCharData->data here
        // -------------------------
        // Copy received data to holder array, ensuring NULL termination.
        memset(received_string, 0, DS_STREAM_START_LEN);
        memcpy(received_string, pCharData->data,
               MIN(pCharData->dataLen, DS_STREAM_START_LEN - 1));

        break;

    case DS_STREAM_INPUT_ID:
        Mailbox_post(mailbox, pCharData->data, BIOS_NO_WAIT);
        counter_packet_received++;

        packet_lost = 100.0f * ((float)counter_packet_send - (float)counter_packet_received) / (float)counter_packet_send;
        // -------------------------
        // Do something useful with pCharData->data here
        break;

    default:
        return;
    }
}

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


void swap_array (int8_t *swap_arr)
{
    int8_t temp;
    temp = swap_arr[163];
    swap_arr[163] = swap_arr[43];
    swap_arr[43] = temp;

    temp = swap_arr[164];
    swap_arr[164] = swap_arr[44];
    swap_arr[44] = temp;

    temp = swap_arr[165];
    swap_arr[165] = swap_arr[45];
    swap_arr[45] = temp;

    temp = swap_arr[166];
    swap_arr[166] = swap_arr[46];
    swap_arr[46] = temp;

}
