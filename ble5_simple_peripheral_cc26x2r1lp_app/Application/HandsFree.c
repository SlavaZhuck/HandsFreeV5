/*
 * HandsFree.c
 *
 *  Created on: 24 џэт. 2019 у.
 *      Author: CIT_007
 */
#include "HandsFree.h"
#include "Noise_TRSH.h"
#include "Uart_commands.h"
#include "max9860_i2c.h"
#include "I2S/I2SCC26XX.h"
#include <ti/sysbios/family/arm/m3/Hwi.h>

/* Timer variables ***************************************************/
GPTimerCC26XX_Params tim_params;
GPTimerCC26XX_Handle blink_tim_hdl = NULL;
GPTimerCC26XX_Handle samp_tim_hdl = NULL;
GPTimerCC26XX_Value load_val[2] = {LOW_STATE_TIME, HIGH_STATE_TIME};
/*********************************************************************/

/* I2C variables *****************************************************/
static uint16_t i2c_read_delay;
/*********************************************************************/
int stream_on = 0;

extern PIN_Handle buttonPinHandle;
extern PIN_Handle ledPinHandle;

uint16_t counter_packet_send = 0;
uint16_t counter_packet_received = 0;
float packet_lost = 0;
uint16_t error_counter_packet_send = 0;

uint8_t send_array[DS_STREAM_OUTPUT_LEN];

/* I2S variables START*/
static void bufRdy_callback(I2SCC26XX_Handle handle, I2SCC26XX_StreamNotification *pStreamNotification);

static int16_t *audio_decoded = NULL;
static uint8_t *i2sContMgtBuffer = NULL;

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
    .blockSize              = I2S_SAMP_PER_FRAME,
    .pvContBuffer           = (void *)i2sSampleBuffer,
    .ui32conBufTotalSize    = (sizeof(int16_t) * I2S_SAMPLE_MEMORY_SZ),
    .pvContMgtBuffer        = (void *)i2sQueueMemory,
    .ui32conMgtBufTotalSize = I2S_TOTAL_QUEUE_MEM_SZ,
    .currentStream          = &i2sStream
};

uint8_t packet_data[TRANSMIT_DATA_LENGTH];
int16_t raw_data_received[I2S_SAMP_PER_FRAME];
int16_t mic_data_1ch[I2S_SAMP_PER_FRAME];


bool gotBufferIn = false;
bool gotBufferInOut = false;
bool gotBufferOut = false;
static void AudioDuplex_enableCache();
static void AudioDuplex_disableCache();

/* I2S variables END*/


void start_voice_handle(void)
{
    max9860_I2C_Shutdown_state(0);//disable shutdown_mode
    I2SCC26XX_startStream(i2sHandle);
    stream_on = 1;
    PIN_setOutputValue(ledPinHandle, Board_PIN_GLED, 1);
    GPTimerCC26XX_start(samp_tim_hdl);
}


void stop_voice_handle(void)
{
    max9860_I2C_Shutdown_state(1);//enable shutdown_mode
    if(stream_on)
    {
        if(!I2SCC26XX_stopStream(i2sHandle))
        {
           while(1);
        }
    }
    GPTimerCC26XX_stop(samp_tim_hdl);
    PIN_setOutputValue(ledPinHandle, Board_PIN_GLED, 0);
    stream_on = 0;
}


void HandsFree_init (void)
{
    max9860_I2C_Init();
    max9860_I2C_Read_Status();
    GPTimerCC26XX_Params_init(&tim_params);
    tim_params.width = GPT_CONFIG_32BIT;
    tim_params.mode = GPT_MODE_PERIODIC_UP;
    tim_params.debugStallMode = GPTimerCC26XX_DEBUG_STALL_ON;
    blink_tim_hdl = GPTimerCC26XX_open(Board_GPTIMER2A, &tim_params);

    if (blink_tim_hdl == NULL) {
        while (1);
    }
    GPTimerCC26XX_setLoadValue(blink_tim_hdl, (GPTimerCC26XX_Value)LOW_STATE_TIME);
    GPTimerCC26XX_registerInterrupt(blink_tim_hdl, blink_timer_callback, GPT_INT_TIMEOUT);
    GPTimerCC26XX_start(blink_tim_hdl);

    tim_params.width = GPT_CONFIG_16BIT;
    samp_tim_hdl = GPTimerCC26XX_open(Board_GPTIMER3A, &tim_params);
    if (samp_tim_hdl == NULL) {
        while (1);
    }
    GPTimerCC26XX_setLoadValue(samp_tim_hdl, (GPTimerCC26XX_Value)SAMP_TIME);
    GPTimerCC26XX_registerInterrupt(samp_tim_hdl, samp_timer_callback, GPT_INT_TIMEOUT);
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

}

void blink_timer_callback(GPTimerCC26XX_Handle handle, GPTimerCC26XX_IntMask interruptMask)
{
    static bool blink = false;
    if(blink)
    {
        blink = false;
        GPTimerCC26XX_setLoadValue(blink_tim_hdl, load_val[0]);

        PIN_setOutputValue(ledPinHandle, Board_PIN_RLED, 0);

        if(!stream_on)
        {
            PIN_setOutputValue(ledPinHandle, Board_PIN_GLED, 0);
        }
    }
    else
    {
        blink = true;
        GPTimerCC26XX_setLoadValue(blink_tim_hdl, load_val[1]);

         if(!stream_on)
        {
            PIN_setOutputValue(ledPinHandle, Board_PIN_GLED, 1);
        }
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
            /* Call different handler per service */
            switch(pCharData->svcUUID)
            {
    //            case LED_SERVICE_SERV_UUID:
    //                ProjectZero_LedService_ValueChangeHandler(pCharData);
    //                break;
              case DATA_SERVICE_SERV_UUID:
                  ProjectZero_DataService_ValueChangeHandler(pCharData);
                  break;
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
            bufferRequest.buffersRequested = I2SCC26XX_BUFFER_IN_AND_OUT;

            gotBufferInOut = I2SCC26XX_requestBuffer(i2sHandle, &bufferRequest);

            if (gotBufferInOut)
            {
                memcpy(bufferRequest.bufferOut, raw_data_received, sizeof(raw_data_received));
                memcpy(mic_data_1ch, bufferRequest.bufferIn, sizeof(mic_data_1ch));

                bufferRelease.bufferHandleOut = bufferRequest.bufferHandleOut;
                bufferRelease.bufferHandleIn = bufferRequest.bufferHandleIn;
                I2SCC26XX_releaseBuffer(i2sHandle, &bufferRelease);
                gotBufferInOut = 0;
                i2c_read_delay++;
            }

            send_array[DS_STREAM_OUTPUT_LEN - 4] = counter_packet_send >> 24;
            send_array[DS_STREAM_OUTPUT_LEN - 3] = counter_packet_send >> 16;
            send_array[DS_STREAM_OUTPUT_LEN - 2] = counter_packet_send >> 8;
            send_array[DS_STREAM_OUTPUT_LEN - 1] = counter_packet_send;
            status = DataService_SetParameter(DS_STREAM_OUTPUT_ID, DS_STREAM_OUTPUT_LEN, send_array);
            if((status != SUCCESS) || (status == 0x15))
            {
                error_counter_packet_send++;
            }
            counter_packet_send++;
            break;
        case PZ_I2C_Read_status_EVT:
            max9860_I2C_Read_Status();
            break;
        case PZ_SEND_START_STREAM_EVT:
            error_counter_packet_send = 0;
            counter_packet_send = 0;
            counter_packet_received = 0;
            start_voice_handle();
            break;
        case PZ_SEND_STOP_STREAM_EVT:
            stop_voice_handle();
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

        if (configValue) // 0x0001 and 0x0002 both indicate turned on.
        {
            if(stream_on != 1)
            {
                //GAPRole_SendUpdateParam(8, 8, 0, TIMEOUT, GAPROLE_RESEND_PARAM_UPDATE);
                ProjectZero_enqueueMsg(PZ_SEND_START_STREAM_EVT, NULL);
            }
        }
        else
        {
            ProjectZero_enqueueMsg(PZ_SEND_STOP_STREAM_EVT, NULL);
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
