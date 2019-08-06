/*
 * HandsFree.c
 *
 *  Created on: 24 янв. 2019 г.
 *      Author: CIT_007
 */
#include <stdlib.h>
#include "HandsFree.h"
#include "Noise_TRSH.h"
#include "Uart_commands.h"
#include "max9860_i2c.h"
#include "I2S/I2SCC26XX.h"
#include <ti/sysbios/family/arm/m3/Hwi.h>
#include "codec/SitADPCM.h"
//#include "ima_codec/ima.h"
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
#include "../LPF/LPF.h"
#include "../LPF/rtwtypes.h"
#include "../Echo_cancel/Echo_cancel.h"
#include "project_zero.h"
#include "./services/data_service.h"
#include "max9860_i2c.h"
#include <ti/drivers/timer/GPTimerCC26XX.h>
#include <ti/drivers/AESCBC.h>
#include <ti/drivers/cryptoutils/cryptokey/CryptoKeyPlaintext.h>
#include <ti/sysbios/BIOS.h>
#include <math.h>

#ifdef  LOGGING
    #define BASE64_SIZE(x)  (((x)+2) / 3 * 4 + 1)
#endif

/******local functions START***************************************************/
static void readCallback(UART_Handle handle, void *rxBuf, size_t size);
static void writeCallback(UART_Handle handle_uart, void *rxBuf, size_t size);

static void bufRdy_callback(I2SCC26XX_Handle handle, I2SCC26XX_StreamNotification *pStreamNotification);
static void AudioDuplex_enableCache();
static void AudioDuplex_disableCache();
static void encrypt_packet(uint8_t *packet);
static void decrypt_packet(uint8_t *packet);
static void rt_OneStep(void);
static void LPF_partial_processing ( uint16_t start_index, uint16_t stop_index, bool enable, bool first_buffer);
static void EC_partial_processing ( uint16_t start_index, uint16_t stop_index, bool enable, bool first_buffer);
static void Resend_BLEpacket_SwiFxn(UArg temp);
static void GetSecondSoundBuf_SwiFxn(UArg temp);
static void start_voice_handle(void);
static void stop_voice_handle(void);
static void blink_timer_callback(GPTimerCC26XX_Handle handle, GPTimerCC26XX_IntMask interruptMask);
static void samp_timer_callback(GPTimerCC26XX_Handle handle, GPTimerCC26XX_IntMask interruptMask);
static void measure_timer_callback(GPTimerCC26XX_Handle handle, GPTimerCC26XX_IntMask interruptMask);
/******local functions END***************************************************/

/* Timer variables START***************************************************/
static GPTimerCC26XX_Params tim_params;
static GPTimerCC26XX_Handle blink_tim_hdl = NULL;
static GPTimerCC26XX_Handle measure_tim_hdl = NULL;
static uint32_t measure_tim_value = 0; /* in ms */
static GPTimerCC26XX_Handle samp_tim_hdl = NULL;
static GPTimerCC26XX_Value load_val[2] = {LOW_STATE_TIME, HIGH_STATE_TIME};

/* Timer variables END***************************************************/

/* Debug measure variables START***************************************************/
static GPTimerCC26XX_Value timestamp_encode_start  = 0;
static GPTimerCC26XX_Value timestamp_encode_stop   = 0;
volatile static GPTimerCC26XX_Value timestamp_encode_dif    = 0;
static GPTimerCC26XX_Value timestamp_decode_start  = 0;
static GPTimerCC26XX_Value timestamp_decode_stop   = 0;
volatile static GPTimerCC26XX_Value timestamp_decode_dif    = 0;

volatile static uint32_t counter_packet_send                = 0;
volatile static uint32_t counter_adc_data_read              = 0;
volatile static uint32_t save_counter_packet_send           = 0;
volatile static uint32_t i2s_buffer_error_1                 = 0;
volatile static uint32_t i2s_buffer_error_2                 = 0;
volatile static float packet_lost_percentage                = 0;
volatile static uint32_t error_counter_packet_send          = 0;
volatile static uint32_t skip_counter_packet_send           = 0;
volatile static uint32_t resend_error_counter_packet_send   = 0;
static GPTimerCC26XX_Value timestamp_TOTAL_start;
static GPTimerCC26XX_Value timestamp_TOTAL_stop;
volatile static GPTimerCC26XX_Value timestamp_TOTAL_dif;
/* Debug measure variables END***************************************************/

/***********LPF START****************************************************/
bool enable_LPF = false;
/***********LPF END****************************************************/

/*****************NOISE GATE START***********************************/
static GPTimerCC26XX_Value timestamp_NG_start;
static GPTimerCC26XX_Value timestamp_NG_stop;
static GPTimerCC26XX_Value timestamp_NG_dif;
static struct power_struct in_power;

bool enable_NoiseGate = false;
/*****************NOISE GATE END***********************************/

/***********Echo compensation START****************************************************/
#define EC_FILTER_SIZE          (3u)
bool enable_EC = false;

static float EC_data_debug[I2S_SAMP_PER_FRAME];
static float EC_filt_coeffs_debug[EC_FILTER_SIZE];
/***********Echo compensation END****************************************************/

/******Crypto Start ***************************************************/
#define KEY_SNV_ID                          (BLE_NVID_CUST_START)
#define KEY_SIZE                            (16)
#define INIT_VOL_ADDR                       (BLE_NVID_CUST_START+1)
    /* CryptoKey storage */
static CryptoKey           cryptoKey;
/* global key applied in the system*/
uint8_t global_key[KEY_SIZE] =
                                {0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6,
                                0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C};
/* default key, if there were no write key operations*/
static const uint8_t default_key[] =
                                {0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6,
                                0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C};
    /* AESCBC variables */
static AESCBC_Operation    operationOneStepEncrypt;
static AESCBC_Operation    operationOneStepDecrypt;
static AESCBC_Handle       AESCBCHandle;
/******Crypto End ***************************************************/

/******Uart Start ***************************************************/
#define UART_BAUD_RATE      921600
#define MAX_NUM_RX_BYTES    100   // Maximum RX bytes to receive in one go
#define MAX_NUM_TX_BYTES    100   // Maximum TX bytes to send in one go
#define WANTED_RX_BYTES     1     // number of waiting bytes in one go

UART_Handle uart; /* global, because it is also used in "Uart_commands.c"*/
static UART_Params uartParams;
static uint32_t wantedRxBytes = WANTED_RX_BYTES;            // Number of bytes received so far
static uint8_t rxBuf[MAX_NUM_RX_BYTES];   // Receive UART buffer
static int16_t uart_data_send[I2S_SAMP_PER_FRAME * 2 + 1 + 2 + 2]; // 2 I2S buffers, 2 start bytes, 4 bytes counter send, 4 bytes adc counter send
bool enable_UART_DEBUG = false;
bool UART_ready = false;
/******Uart End ***************************************************/



#ifdef LOGGING
    static uint8_t received_SID[SID_LENGTH] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19};
    struct event_indicator_struct_BUF_status  event_BUF_status_message;
    struct event_indicator_struct_BLE         event_BLE_message;
    struct connection_status                  event_connection_status_message;
    uint8_t UART_buffer_BUF_status_message    [BASE64_SIZE(sizeof(event_BUF_status_message))];
    uint8_t UART_buffer_BLE_message_receive   [BASE64_SIZE(sizeof(event_BLE_message))];
    uint8_t UART_buffer_BLE_message_send      [BASE64_SIZE(sizeof(event_BLE_message))];
    uint8_t UART_buffer_BLE_message_send_error[BASE64_SIZE(sizeof(event_BLE_message))];
    uint8_t UART_buffer_connection_status     [BASE64_SIZE(sizeof(event_connection_status_message))];

    static Clock_Struct Uart_Connection_Status;
    static Clock_Handle Uart_Connection_Status_ClockHandle;
    static void UART_Send_Connection_Status_SwiFxn(UArg temp);

#define CONNECTION_STATUS_DELAY (10u) /* delay in ms between status messages sending, when BLE connection is established */
#define CONNECTION_STATUS_MESSAGE_NUM (10u) /* number of periodically sent messages */

    static uint32_t sended_messages = 0u;

    static void update_UART_Messages (uint8_t message_type);
    static void swap_endian (uint8_t* pointer, uint32_t size);
#endif

/* I2C variables START*****************************************************/
static uint16_t i2c_read_delay;
/* I2C variables END*****************************************************/

/*********************************************************************/

/****** global variables START****************************************/
uint32_t battery_voltage;
uint8_t current_volume = INIT_GAIN;
uint8_t stream_on = 0;
uint8_t macAddress[MAC_SIZE];
/****** global variables END****************************************/


/****** BLE receive/transmit variables and helpers START****************************************/
static Clock_Struct Resend_BLEpacket_ChannelSwitchClock;
static Clock_Handle Resend_BLEpacket_ClockHandle;
static Clock_Struct GetSecondSoundBuf_ChannelSwitchClock;
static Clock_Handle GetSecondSoundBuf_ClockHandle;
static Mailbox_Handle mailbox;
static uint8_t mailpost_usage;
static uint8_t resend_counter = 0;

static uint8_t send_status = 0;
static uint32_t counter_packet_received = 0;
static uint32_t previous_receive_counter = 0;

static uint8_t send_array[DS_STREAM_OUTPUT_LEN];
static uint8_t packet_data[DS_STREAM_OUTPUT_LEN];
/****** BLE receive/transmit variables and helpers END****************************************/


/*************** I2S variables START ***************************************************/
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
    .ui32requestTimeout     = BIOS_NO_WAIT,//BIOS_NO_WAIT,
    .callbackFxn            = bufRdy_callback,
    .blockSize              = FRAME_SIZE,//I2S_SAMP_PER_FRAME,
    .pvContBuffer           = (void *)i2sSampleBuffer,
    .ui32conBufTotalSize    = (sizeof(int16_t) * I2S_SAMPLE_MEMORY_SZ),
    .pvContMgtBuffer        = (void *)i2sQueueMemory,
    .ui32conMgtBufTotalSize = I2S_TOTAL_QUEUE_MEM_SZ,
    .currentStream          = &i2sStream
};

static int16_t raw_data_received[I2S_SAMP_PER_FRAME];
static int16_t mic_data_1ch[I2S_SAMP_PER_FRAME];

#ifdef SECOND_MICROPHONE
static int16_t mic_data[I2S_SAMP_PER_FRAME*2];
static int16_t mic_data_2ch[I2S_SAMP_PER_FRAME];
#endif

static int16_t sinus[I2S_SAMP_PER_FRAME];
//int16_t raw_mic_data[I2S_SAMP_PER_FRAME];

static bool gotBufferInOut = FALSE;
/*************** I2S variables END ***************************************************/

/******** codec START ***************************************************/
//static ima_state ima_Encode_state;
//static ima_state ima_Decode_state;

static struct ADPCMstate encoder_adpcm, decoder_adpcm;
/******** codec END ***************************************************/

/******** extern variables START ***************************************************/
extern Clock_Handle ADC_ChannelSwitchClockHandle;
extern ADCBuf_Conversion adc_conversion;
extern ADCBuf_Handle adc_hdl;
extern pzConnRec_t connList[MAX_NUM_BLE_CONNS];
extern bool connection_occured;
extern List_List paramUpdateList;
extern bool button_check;
extern bool enable_blink;
extern PIN_Handle ledPinHandle;
/******** extern variables END ***************************************************/

#ifdef DEBUG_SINUS

#define M_PI        (3.14159265358979323846)
#define M_PI2  (2 * M_PI)

static float freq = 10.0f, phase = 0.0f, amplitude = 1024.0f;
static float local_phase = 0;
static float time = 0;
static GPTimerCC26XX_Value timestamp_sinus_debug_start;
static GPTimerCC26XX_Value timestamp_sinus_debug_stop;
static GPTimerCC26XX_Value timestamp_sinus_debug_dif;



void inline sinus_debug (void)
{
    timestamp_sinus_debug_start = GPTimerCC26XX_getValue(measure_tim_hdl);

    for (uint16_t sample = 0; sample < I2S_SAMP_PER_FRAME; sample++)
    {
        local_phase =   (M_PI / 180.0f * ((float)sample) * freq * 0.02265) + (phase * M_PI / 180.0f);

        while(local_phase >= M_PI2 )
        {
            local_phase -= M_PI2;
        }
        raw_data_received[sample] = (int16_t)(amplitude * sinf(local_phase));


        if(sample ==  I2S_SAMP_PER_FRAME - 1)
        {
//            local_phase =   (M_PI / 180.0f * (sample + 2 ) * freq) + (phase * M_PI / 180.0f);
            phase = local_phase * 180.0f / M_PI;
            while(phase >= 360 )
            {
                phase -= 360;
            }
        }

    }
    freq += 10;
    if(freq >= 7000)
    {
        freq = 10;
    }
    timestamp_sinus_debug_stop = GPTimerCC26XX_getValue(measure_tim_hdl);
    timestamp_sinus_debug_dif = timestamp_sinus_debug_stop-timestamp_sinus_debug_start;
}
#endif

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


static void LPF_partial_processing ( uint16_t start_index, uint16_t stop_index, bool enable, bool first_buffer)
{
    static uint32_t timestamp_LPF_start = 0;
    static uint32_t timestamp_LPF_stop  = 0;
    static uint32_t timestamp_LPF_dif   = 0;
    static uint32_t timestamp_LPF_prev  = 0;

    if(enable)
    {
        timestamp_LPF_start =  GPTimerCC26XX_getValue(measure_tim_hdl);
        if(first_buffer)
        {
            timestamp_LPF_prev = 0;
        }
        else
        {
            timestamp_LPF_dif = timestamp_LPF_prev;
        }

        for(uint16_t i = start_index ; i< stop_index; i++)
        {
            //raw_mic_data[i] = mic_data_1ch[i];
            rtU.In1 = (float)mic_data_1ch[i];
            rt_OneStep();
            mic_data_1ch[i] = (int16)rtY.Out1;
        }
        timestamp_LPF_stop =  GPTimerCC26XX_getValue(measure_tim_hdl);
        timestamp_LPF_dif += timestamp_LPF_stop - timestamp_LPF_start;
    }
}

static void EC_partial_processing ( uint16_t start_index, uint16_t stop_index, bool enable, bool first_buffer)
{
    static uint32_t timestamp_EC_start = 0;
    static uint32_t timestamp_EC_stop  = 0;
    static uint32_t timestamp_EC_dif   = 0;
    static uint32_t timestamp_EC_prev  = 0;

    if(enable)
    {
        timestamp_EC_start =  GPTimerCC26XX_getValue(measure_tim_hdl);
        if(first_buffer)
        {
            timestamp_EC_prev = 0;
        }
        else
        {
            timestamp_EC_dif = timestamp_EC_prev;
        }

        for(uint16_t i = start_index ; i< stop_index; i++)
        {
            rtUeC.EnableeC = TRUE;
            rtUeC.ReseteC = FALSE;
            rtUeC.BLE_receiveeC = (float)raw_data_received[i];
            rtUeC.MIC_dataeC = (float)mic_data_1ch[i];
            Echo_cancel_step();
            mic_data_1ch[i] = (int16_t)rtYeC.OutputeC;
            EC_data_debug[i] = rtYeC.debug_erroreC;
        }
        timestamp_EC_stop =  GPTimerCC26XX_getValue(measure_tim_hdl);
        timestamp_EC_dif += timestamp_EC_stop - timestamp_EC_start;
    }
    else
    {
        rtUeC.ReseteC = TRUE;
        Echo_cancel_step();
    }
}

void start_voice_handle(void)
{
    /* init debug variables and counters*/
#ifdef DEBUG_SINUS
    time = 0;
#endif
    measure_tim_value = 0;
    skip_counter_packet_send = 0;
    i2s_buffer_error_1 = 0;
    i2s_buffer_error_2 = 0;
    resend_error_counter_packet_send = 0;
    error_counter_packet_send = 0;
    counter_packet_send = 0;
    counter_adc_data_read = 0;
    counter_packet_received = 0;
#ifdef LOGGING
//    memset(received_SID, 0, SID_LENGTH);
//    memset(&event_BLE_message, 0, sizeof(event_BLE_message)) ;
//    memset(&event_BUF_status_message, 0, sizeof(event_BUF_status_message)) ;
#endif
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
    gotBufferInOut = FALSE ;

    /* release all rest sound buffer before stopping stream */
    bufferRequest.buffersRequested = I2SCC26XX_BUFFER_IN_AND_OUT;
    gotBufferInOut = I2SCC26XX_requestBuffer(i2sHandle, &bufferRequest);
    while (gotBufferInOut)
    {
        bufferRelease.bufferHandleOut = bufferRequest.bufferHandleOut;
        bufferRelease.bufferHandleIn = bufferRequest.bufferHandleIn;
        I2SCC26XX_releaseBuffer(i2sHandle, &bufferRelease);
        bufferRequest.buffersRequested = I2SCC26XX_BUFFER_IN_AND_OUT;
        gotBufferInOut = I2SCC26XX_requestBuffer(i2sHandle, &bufferRequest);
    }

    /* stop I2S stream */
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
    GPTimerCC26XX_setLoadValue(measure_tim_hdl, (GPTimerCC26XX_Value)MEASURE_TIME);
    GPTimerCC26XX_registerInterrupt(measure_tim_hdl, measure_timer_callback, GPT_INT_TIMEOUT);

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
#ifdef DEBUG_LOGGING
    uartParams.writeMode        = UART_MODE_BLOCKING;
    uartParams.writeTimeout      = UART_WAIT_FOREVER; //UART_WAIT_FOREVER
#endif
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
    UART_ready = true;
    uint64_t temp = *((uint64_t *)(0x500012E8)) & 0xFFFFFFFFFFFFFF;
    for(uint8_t i = 0 ; i < 6 ; i++)
    {
        macAddress[i]=*(((uint8_t *)&temp)+i);
    }
    //macAddress = *((uint64_t *)(0x500012E8)) & 0xFFFFFFFFFFFFFF;

    //UART_write(uart, macAddress, sizeof(macAddress));
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

#endif

    Resend_BLEpacket_ClockHandle = Util_constructClock(&Resend_BLEpacket_ChannelSwitchClock,
                                                       Resend_BLEpacket_SwiFxn, RESEND_DELAY,
                                                       0,
                                                       0,
                                                       0);
    GetSecondSoundBuf_ClockHandle = Util_constructClock(&GetSecondSoundBuf_ChannelSwitchClock,
                                                       GetSecondSoundBuf_SwiFxn, SECOND_BUFFER_AQUIRE_DELAY,
                                                       0,
                                                       0,
                                                       0);
    /* BLE optimization */
    HCI_EXT_OverlappedProcessingCmd(HCI_EXT_ENABLE_OVERLAPPED_PROCESSING);
    HCI_EXT_HaltDuringRfCmd( HCI_EXT_HALT_DURING_RF_DISABLE ); //Enable CPU during RF events (scan included) - may increase power consumption
    HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_DISABLE_CLK_DIVIDE_ON_HALT ); //Set whether the system clock will be divided when the MCU is halted. - may increase power consumption
    HCI_EXT_SetFastTxResponseTimeCmd(HCI_EXT_ENABLE_FAST_TX_RESP_TIME); // configure the Link Layer fast transmit response time feature

#ifdef DEBUG_LOGGING
   // start_voice_handle();
    GPTimerCC26XX_setLoadValue(samp_tim_hdl, (GPTimerCC26XX_Value)SAMP_TIME);
    GPTimerCC26XX_start(samp_tim_hdl);
    GPTimerCC26XX_start(measure_tim_hdl);
#endif

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
        ProjectZero_enqueueMsg(PZ_GET_FIRST_SOUND_FRAME, NULL);
    }
#ifdef DEBUG_LOGGING
    counter_packet_received++;
    update_UART_Messages(PACKET_SENT_MESSAGE_TYPE);
    ProjectZero_enqueueMsg(PZ_APP_MSG_Send_message_BLE, NULL);
    ProjectZero_enqueueMsg(PZ_APP_MSG_Send_message_Buf_Status, NULL);
    counter_packet_received++;
    update_UART_Messages(PACKET_SENT_MESSAGE_TYPE);
    ProjectZero_enqueueMsg(PZ_APP_MSG_Send_message_BLE, NULL);
    ProjectZero_enqueueMsg(PZ_APP_MSG_Send_message_Buf_Status, NULL);

#endif
}
void measure_timer_callback(GPTimerCC26XX_Handle handle, GPTimerCC26XX_IntMask interruptMask)
{
    measure_tim_value++;
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

        case PZ_GET_FIRST_SOUND_FRAME:
        {
            timestamp_TOTAL_start = measure_tim_value;
            mailpost_usage = Mailbox_getNumPendingMsgs(mailbox);
            if(mailpost_usage>0)
            {
                if(previous_receive_counter != (counter_packet_received - 1))
                {
                   // memset(&ima_Decode_state, 0, sizeof(ima_Decode_state));
                }
                Mailbox_pend(mailbox, packet_data, BIOS_NO_WAIT);

                timestamp_decode_start =  GPTimerCC26XX_getValue(measure_tim_hdl);
                //decrypt_packet(packet_data);
                decoder_adpcm.prevsample = ((int16_t)(packet_data[V_STREAM_OUTPUT_SOUND_LEN]) << 8) |
                        (int16_t)(packet_data[V_STREAM_OUTPUT_SOUND_LEN + 1]);

                decoder_adpcm.previndex = ((int32_t)(packet_data[V_STREAM_OUTPUT_SOUND_LEN + 2]));
                ADPCMDecoderBuf2((char*)(packet_data), raw_data_received, &decoder_adpcm);
                ADPCMDecoderBuf2((char*)(&packet_data[FRAME_SIZE / 4]), &raw_data_received[I2S_SAMP_PER_FRAME / 4], &decoder_adpcm);
                ADPCMDecoderBuf2((char*)(&packet_data[FRAME_SIZE / 2]), &raw_data_received[I2S_SAMP_PER_FRAME / 2], &decoder_adpcm);
                ADPCMDecoderBuf2((char*)(&packet_data[FRAME_SIZE* 3 / 4]), &raw_data_received[I2S_SAMP_PER_FRAME * 3 / 4], &decoder_adpcm);

                timestamp_decode_stop =  GPTimerCC26XX_getValue(measure_tim_hdl);
                timestamp_decode_dif = timestamp_decode_stop - timestamp_decode_start;

                previous_receive_counter = counter_packet_received;

            }else{
                memset ( packet_data,   0x00, sizeof(packet_data) );
//                memset ( raw_data_received, 0x00, sizeof(raw_data_received));
            }

            bufferRequest.buffersRequested = I2SCC26XX_BUFFER_IN_AND_OUT;
            gotBufferInOut = I2SCC26XX_requestBuffer(i2sHandle, &bufferRequest);

#ifdef DEBUG_SINUS
            sinus_debug();
//            memcpy(raw_data_received, sinus, sizeof(raw_data_received));
#endif

            if (gotBufferInOut)
            {
#ifdef SECOND_MICROPHONE
                memcpy(mic_data, bufferRequest.bufferIn, sizeof(mic_data) / 2 );
                for(uint16_t i = 0; i<I2S_SAMP_PER_FRAME/2;i++)
                {
                    mic_data_2ch[i] = mic_data[i*2];  //DA1
                    mic_data_1ch[i] = mic_data[i*2+1];//DA2
                }
#else
                memcpy(mic_data_1ch, bufferRequest.bufferIn, sizeof(mic_data_1ch) / 2 );
#endif
                memcpy(bufferRequest.bufferOut, raw_data_received, sizeof(raw_data_received) / 2 );

                bufferRelease.bufferHandleOut = bufferRequest.bufferHandleOut;
                bufferRelease.bufferHandleIn = bufferRequest.bufferHandleIn;
                I2SCC26XX_releaseBuffer(i2sHandle, &bufferRelease);
                gotBufferInOut = FALSE;
                i2c_read_delay++;
                /* process first part of sound buffer*/
                LPF_partial_processing ( 0 , I2S_SAMP_PER_FRAME / 2, enable_LPF, TRUE);
                EC_partial_processing  ( 0 , I2S_SAMP_PER_FRAME / 2, enable_EC, TRUE);
            }
            else
            {
                /* erase MIC buffer if no data was received from I2S */
                memset ( mic_data_1ch,  0x00, sizeof(mic_data_1ch));
                i2s_buffer_error_1++;
            }
            /* request second part of MIC buffer*/
            Util_startClock((Clock_Struct *)GetSecondSoundBuf_ClockHandle);
        }
        break;

        case PZ_GET_SECOND_SOUND_FRAME_SEND_PACKET_EVT:
        {
            bufferRequest.buffersRequested = I2SCC26XX_BUFFER_IN_AND_OUT;
            gotBufferInOut = I2SCC26XX_requestBuffer(i2sHandle, &bufferRequest);
            if (gotBufferInOut)
            {
#ifdef SECOND_MICROPHONE
                memcpy(&mic_data[I2S_SAMP_PER_FRAME], bufferRequest.bufferIn, sizeof(mic_data) / 2 );
                for(uint16_t i = I2S_SAMP_PER_FRAME/2; i<I2S_SAMP_PER_FRAME;i++)
                {
                    mic_data_2ch[i] = mic_data[i*2];  //DA1
                    mic_data_1ch[i] = mic_data[i*2+1];//DA2
                }
#else
                memcpy(&mic_data_1ch[160], bufferRequest.bufferIn, sizeof(mic_data_1ch) / 2 );
#endif
                memcpy(bufferRequest.bufferOut, &raw_data_received[160], sizeof(raw_data_received) / 2 );

                bufferRelease.bufferHandleOut = bufferRequest.bufferHandleOut;
                bufferRelease.bufferHandleIn = bufferRequest.bufferHandleIn;
                I2SCC26XX_releaseBuffer(i2sHandle, &bufferRelease);
                gotBufferInOut = FALSE;
                counter_adc_data_read++;
                /* process second part of sound buffer*/
                LPF_partial_processing ( I2S_SAMP_PER_FRAME / 2 , I2S_SAMP_PER_FRAME, enable_LPF, FALSE);
                EC_partial_processing  ( I2S_SAMP_PER_FRAME / 2 , I2S_SAMP_PER_FRAME, enable_EC, FALSE);

                if(enable_NoiseGate)
                {
                    timestamp_NG_start =  GPTimerCC26XX_getValue(measure_tim_hdl);
                    in_power = power_calculation(mic_data_1ch, I2S_SAMP_PER_FRAME);//52000 ticks
                    amplify (mic_data_1ch, I2S_SAMP_PER_FRAME, (int16_t)in_power.power_log);

                    timestamp_NG_stop =  GPTimerCC26XX_getValue(measure_tim_hdl);
                    timestamp_NG_dif = timestamp_NG_stop - timestamp_NG_start;
                }
                //i2c_read_delay++;
            }
            else
            {
                /* erase part of MIC buffer if no data was received from I2S */
                memset ( &mic_data_1ch[160],  0x00, sizeof(mic_data_1ch) / 2);
                i2s_buffer_error_2++;
            }

            timestamp_encode_start =  GPTimerCC26XX_getValue(measure_tim_hdl);
            send_array[V_STREAM_OUTPUT_SOUND_LEN] = (uint8_t)(encoder_adpcm.prevsample >> 8);
            send_array[V_STREAM_OUTPUT_SOUND_LEN + 1] = (uint8_t)encoder_adpcm.prevsample;
            send_array[V_STREAM_OUTPUT_SOUND_LEN + 2] = (uint8_t)encoder_adpcm.previndex;
            send_array[TRANSMIT_DATA_LENGTH - 4] = (uint8_t)(counter_packet_send >> 24);
            send_array[TRANSMIT_DATA_LENGTH - 3] = (uint8_t)(counter_packet_send >> 16);
            send_array[TRANSMIT_DATA_LENGTH - 2] = (uint8_t)(counter_packet_send >> 8);
            send_array[TRANSMIT_DATA_LENGTH - 1] = (uint8_t)(counter_packet_send);

            ADPCMEncoderBuf2(mic_data_1ch, (char*)(send_array), &encoder_adpcm);
            ADPCMEncoderBuf2(&mic_data_1ch[I2S_SAMP_PER_FRAME / 4], (char*)(&send_array[FRAME_SIZE/4]), &encoder_adpcm);
            ADPCMEncoderBuf2(&mic_data_1ch[I2S_SAMP_PER_FRAME / 2], (char*)(&send_array[FRAME_SIZE/2]), &encoder_adpcm);
            ADPCMEncoderBuf2(&mic_data_1ch[I2S_SAMP_PER_FRAME * 3 / 4], (char*)(&send_array[FRAME_SIZE * 3 / 4]), &encoder_adpcm);


            timestamp_encode_stop =  GPTimerCC26XX_getValue(measure_tim_hdl);
            timestamp_encode_dif = timestamp_encode_stop - timestamp_encode_start;

            //encrypt_packet(send_array);
            send_status = DataService_SetParameter(DS_STREAM_OUTPUT_ID, DS_STREAM_OUTPUT_LEN, send_array);
            if((send_status != SUCCESS)/* || (send_status == 0x15)*/) /* 0x15 bleNoResources*/
            {
                Util_startClock((Clock_Struct *)Resend_BLEpacket_ClockHandle);
                resend_error_counter_packet_send++;
                /* safe current packet number to check before repeat */
                save_counter_packet_send = counter_packet_send;
            }
            else
            {
                counter_packet_send++;
                resend_counter = 0;
#ifdef LOGGING
                update_UART_Messages(PACKET_SENT_MESSAGE_TYPE);

                ProjectZero_enqueueMsg(PZ_APP_MSG_Send_message_BLE, NULL);
                ProjectZero_enqueueMsg(PZ_APP_MSG_Send_message_Buf_Status, NULL);
#endif
            }

            if(enable_UART_DEBUG)
            {
                memcpy(&uart_data_send[1],                            mic_data_1ch,      sizeof(mic_data_1ch));

                memcpy(&uart_data_send[I2S_SAMP_PER_FRAME*1 + 1],       raw_data_received,     sizeof(raw_data_received));

                uart_data_send[0]= (40u << 8u) + 41u;   //start bytes for MATLAB ")("
                uart_data_send[I2S_SAMP_PER_FRAME*2 + 1] = counter_packet_send >>16;

                uart_data_send[I2S_SAMP_PER_FRAME*2 + 1 + 2 - 2] = (uint16_t)(counter_packet_send >> 16);
                uart_data_send[I2S_SAMP_PER_FRAME*2 + 1 + 2 - 1] = (uint16_t)(counter_packet_send);

                uart_data_send[I2S_SAMP_PER_FRAME*2 + 1 + 4 - 2] = (uint16_t)(counter_adc_data_read >> 16);
                uart_data_send[I2S_SAMP_PER_FRAME*2 + 1 + 4 - 1] = (uint16_t)(counter_adc_data_read);


                while(!UART_ready)
                {
                    UART_write(uart, uart_data_send, sizeof(uart_data_send));
                }
                UART_ready = false;
            }
            timestamp_TOTAL_stop = measure_tim_value;
            timestamp_TOTAL_dif = timestamp_TOTAL_stop - timestamp_TOTAL_start;
        }
        break;

        case PZ_I2C_Read_status_EVT:
        {
            max9860_I2C_Read_Status();
        }
        break;

        case PZ_START_STREAM_EVT:
        {
            start_voice_handle();
        }
        break;

        case PZ_STOP_STREAM_EVT:
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
//                while(1);
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
            /* different UART buffers were created, because if previous buffer was not sent,
             * this buffer would be overwritten by new one
             * */
            if( PACKET_RECEIVED_MESSAGE_TYPE== event_BLE_message.message_type)
            {
                base64_encode((char *)UART_buffer_BLE_message_receive, (int32_t)(sizeof(UART_buffer_BLE_message_receive)), (const uint8_t *)&event_BLE_message, sizeof(event_BLE_message));
                UART_buffer_BLE_message_receive[sizeof(UART_buffer_BLE_message_receive) - 1] = 0x0A;
                while(!UART_ready)
                {
                    UART_write(uart, UART_buffer_BLE_message_receive, sizeof(UART_buffer_BLE_message_receive));
                }
                UART_ready = false;
            }
            else if( PACKET_SENT_MESSAGE_TYPE== event_BLE_message.message_type)
            {
                base64_encode((char *)UART_buffer_BLE_message_send, (int32_t)(sizeof(UART_buffer_BLE_message_send)), (const uint8_t *)&event_BLE_message, sizeof(event_BLE_message));
                UART_buffer_BLE_message_send[sizeof(UART_buffer_BLE_message_send) - 1] = 0x0A;
                while(!UART_ready)
                {
                    UART_write(uart, UART_buffer_BLE_message_send, sizeof(UART_buffer_BLE_message_send));
                }
                UART_ready = false;
            }
            else if( PACKET_SENT_ERROR_TYPE== event_BLE_message.message_type)
            {
                base64_encode((char *)UART_buffer_BLE_message_send_error, (int32_t)(sizeof(UART_buffer_BLE_message_send_error)), (const uint8_t *)&event_BLE_message, sizeof(event_BLE_message));
                UART_buffer_BLE_message_send_error[sizeof(UART_buffer_BLE_message_send_error) - 1] = 0x0A;
                while(!UART_ready)
                {
                    UART_write(uart, UART_buffer_BLE_message_send_error, sizeof(UART_buffer_BLE_message_send_error));
                }
                UART_ready = false;
            }
        }
        break;

        case PZ_APP_MSG_Send_message_Buf_Status:
        {
            base64_encode((char *)UART_buffer_BUF_status_message, (int32_t)(sizeof(UART_buffer_BUF_status_message)), (const uint8_t *)&event_BUF_status_message, sizeof(event_BUF_status_message));
            UART_buffer_BUF_status_message[sizeof(UART_buffer_BUF_status_message) - 1] = 0x0A;
            while(!UART_ready)
            {
                UART_write(uart, UART_buffer_BUF_status_message, sizeof(UART_buffer_BUF_status_message));
            }
            UART_ready = false;

        }
        break;

        case PZ_APP_MSG_Send_message_CONN_Status:
        {
            base64_encode((char *)UART_buffer_connection_status, (int32_t)(sizeof(UART_buffer_connection_status)), (const uint8_t *)&event_connection_status_message, sizeof(event_connection_status_message));
            UART_buffer_connection_status[sizeof(UART_buffer_connection_status) - 1] = 0x0A;
            while(!UART_ready)
            {
                UART_write(uart, UART_buffer_connection_status, sizeof(UART_buffer_connection_status));
            }
            UART_ready = false;
        }
        break;
#endif
        case PZ_APP_MSG_Read_key:
            get_fh_key();
        break;

        case PZ_APP_MSG_Write_key:
            send_fh_key();
        break;

        case PZ_APP_MSG_Resend_Packet:
            if(counter_packet_send == save_counter_packet_send)
            {
                /* increment resend counter, resend no more than 3 times (4 is max available) */
                resend_counter++;
                send_status = DataService_SetParameter(DS_STREAM_OUTPUT_ID, DS_STREAM_OUTPUT_LEN, send_array);
                if((send_status != SUCCESS)/* || (send_status == 0x15)*/) /* 0x15 bleNoResources*/
                {
                    Util_startClock((Clock_Struct *)Resend_BLEpacket_ClockHandle);
                    error_counter_packet_send++;
                    //save_counter_packet_send = 0;
#ifdef LOGGING
                    update_UART_Messages(PACKET_SENT_ERROR_TYPE);
                    ProjectZero_enqueueMsg(PZ_APP_MSG_Send_message_BLE, NULL);
                    ProjectZero_enqueueMsg(PZ_APP_MSG_Send_message_Buf_Status, NULL);
#endif
                }
                else
                {
                    resend_counter = 0;
                    counter_packet_send++;
#ifdef LOGGING
                    update_UART_Messages(PACKET_SENT_MESSAGE_TYPE);
                    ProjectZero_enqueueMsg(PZ_APP_MSG_Send_message_BLE, NULL);
                    ProjectZero_enqueueMsg(PZ_APP_MSG_Send_message_Buf_Status, NULL);
#endif
                }
            }
            else
            {
                skip_counter_packet_send++;
                resend_counter = 0;
#ifdef LOGGING
                update_UART_Messages(PACKET_SENT_ERROR_TYPE);
                ProjectZero_enqueueMsg(PZ_APP_MSG_Send_message_BLE, NULL);
                ProjectZero_enqueueMsg(PZ_APP_MSG_Send_message_Buf_Status, NULL);
#endif

            }
        break;


        default:
        break;
    }
}

static void Resend_BLEpacket_SwiFxn(UArg temp)
{
    if(resend_counter <3)
    {
        ProjectZero_enqueueMsg(PZ_APP_MSG_Resend_Packet, NULL);
    }
}

static void GetSecondSoundBuf_SwiFxn(UArg temp)
{
    ProjectZero_enqueueMsg(PZ_GET_SECOND_SOUND_FRAME_SEND_PACKET_EVT, NULL);
}

#ifdef LOGGING

static void UART_Send_Connection_Status_SwiFxn(UArg temp)
{
    while(sended_messages < CONNECTION_STATUS_MESSAGE_NUM) /* while not all status messages were sent */
    {
        ProjectZero_enqueueMsg(PZ_APP_MSG_Send_message_CONN_Status, NULL);
        sended_messages++;
        Util_startClock((Clock_Struct *)Uart_Connection_Status_ClockHandle);
    }
}

#endif

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
                    ProjectZero_enqueueMsg(PZ_START_STREAM_EVT, NULL);
                }
            }
            else
            {
                ProjectZero_enqueueMsg(PZ_STOP_STREAM_EVT, NULL);
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
        for(uint16_t i = 0 ; i < sizeof(event_connection_status_message.SID); i++)
        {
            event_connection_status_message.SID[i] = received_SID[i];
        }
        swap_endian((uint8_t*)&event_connection_status_message.SID, sizeof(event_connection_status_message.SID));

        sended_messages = 0;/* update number of already sent messages if new SID was received */
        Util_startClock((Clock_Struct *)Uart_Connection_Status_ClockHandle); /* start to periodically send status messages to UART */
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
        packet_lost_percentage = 100.0f * ((float)counter_packet_send - (float)counter_packet_received) / (float)counter_packet_send;
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
    //uint32_t timestamp = GPTimerCC26XX_getValue(measure_tim_hdl);
    uint32_t timestamp = measure_tim_value;
    event_BLE_message.message_type = message_type;
    event_BLE_message.packet_number = counter_packet_received;
    event_BLE_message.timestamp = timestamp;

    event_BUF_status_message.buff_status = Mailbox_getNumPendingMsgs(mailbox);
    event_BUF_status_message.timestamp = timestamp;

    swap_endian((uint8_t*)&event_BLE_message.packet_number, sizeof(event_BLE_message.packet_number));
    swap_endian((uint8_t*)&event_BLE_message.timestamp, sizeof(event_BLE_message.timestamp));

    swap_endian((uint8_t*)&event_BUF_status_message.timestamp, sizeof(event_BUF_status_message.timestamp));

}

static void swap_endian (uint8_t* pointer, uint32_t size)
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


#endif

static void bufRdy_callback(I2SCC26XX_Handle handle, I2SCC26XX_StreamNotification *pStreamNotification)
{
    I2SCC26XX_Status streamStatus = pStreamNotification->status;

    if (streamStatus == I2SCC26XX_STREAM_BUFFER_READY || streamStatus == I2SCC26XX_STREAM_BUFFER_READY_BUT_NO_AVAILABLE_BUFFERS)
    {
        gotBufferInOut = true;
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
    UART_ready = true;
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


    status = osal_snv_read(KEY_SNV_ID, KEY_SIZE, key);
    if(status != SUCCESS)
    {
        memcpy(key, default_key, KEY_SIZE);
    }
    CryptoKeyPlaintext_initKey(&cryptoKey, (uint8_t*) key, sizeof(*key));

    return status;
}

uint8_t write_aes_key(uint8_t *key)
{
    return (osal_snv_write(KEY_SNV_ID, KEY_SIZE, key));
}
