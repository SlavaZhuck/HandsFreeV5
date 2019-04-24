/*
 * buttons.c
 *
 *  Created on: 19 апр. 2019 г.
 *      Author: CIT_007
 */
#include "HandsFree.h"
#include "buttons.h"
#include "max9860_i2c.h"


/* Button handling functions */
static void buttonDebounceSwiFxn(UArg buttonId);
static void buttonCallbackFxn(PIN_Handle handle,
                              PIN_Id pinId);


static PIN_Handle buttonPinHandle;
static PIN_State buttonPinState;

// State of the buttons
static uint8_t button0State = 0;
static uint8_t button1State = 0;
extern uint8_t current_volume;
/*
 * Application button pin configuration table:
 *   - Buttons interrupts are configured to trigger on falling edge.
 */
PIN_Config buttonPinTable[] = {
    BUTTON_VOLUME_HIGH | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    BUTTON_VOLUME_LOW | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_NEGEDGE,
    PIN_TERMINATE
};








// Clock objects for debouncing the buttons
static Clock_Struct button0DebounceClock;
static Clock_Struct button1DebounceClock;
static Clock_Handle button0DebounceClockHandle;
static Clock_Handle button1DebounceClockHandle;

void buttons_init (void)
{
    // Open button pins
    buttonPinHandle = PIN_open(&buttonPinState, buttonPinTable);
    if(!buttonPinHandle)
    {
        Task_exit();
    }

    // Setup callback for button pins
    if(PIN_registerIntCb(buttonPinHandle, &buttonCallbackFxn) != 0)
    {
        Task_exit();
    }

    // Create the debounce clock objects for Button 0 and Button 1
    button0DebounceClockHandle = Util_constructClock(&button0DebounceClock,
                                                     buttonDebounceSwiFxn, BUTTON_DEBOUNCE_TIME,
                                                     0,
                                                     0,
                                                     BUTTON_VOLUME_HIGH);
    button1DebounceClockHandle = Util_constructClock(&button1DebounceClock,
                                                     buttonDebounceSwiFxn, BUTTON_DEBOUNCE_TIME,
                                                     0,
                                                     0,
                                                     BUTTON_VOLUME_LOW);

}




/*
 * @brief   Handle a debounced button press or release in Task context.
 *          Invoked by the taskFxn based on a message received from a callback.
 *
 * @see     buttonDebounceSwiFxn
 * @see     buttonCallbackFxn
 *
 * @param   pState  pointer to pzButtonState_t message sent from debounce Swi.
 *
 * @return  None.
 */
void Handler_ButtonPress(pzButtonState_t *pState)
{

    // Update the service with the new value.
    // Will automatically send notification/indication if enabled.
    switch(pState->pinId)
    {
        case BUTTON_VOLUME_HIGH:
        {
            if(current_volume<=0)
            {
               current_volume = 0;
            }
            else
            {
               current_volume-=2;
            }
        }
        break;

        case BUTTON_VOLUME_LOW:
        {
            if(current_volume>=86) //188 - MUTE
            {
               current_volume = 86;
            }
            else
            {
               current_volume+=2;
            }
        }
        break;

        default:
        {

        }
        break;

    }
    max9860_I2C_Volume_update(current_volume);
}

/*********************************************************************
 * @fn     buttonCallbackFxn
 *
 * @brief  Callback from PIN driver on interrupt
 *
 *         Sets in motion the debouncing.
 *
 * @param  handle    The PIN_Handle instance this is about
 * @param  pinId     The pin that generated the interrupt
 */
static void buttonCallbackFxn(PIN_Handle handle, PIN_Id pinId)
{
    // Disable interrupt on that pin for now. Re-enabled after debounce.
    PIN_setConfig(handle, PIN_BM_IRQ, pinId | PIN_IRQ_DIS);

    // Start debounce timer
    switch(pinId)
    {
    case BUTTON_VOLUME_HIGH:
        Util_startClock((Clock_Struct *)button0DebounceClockHandle);
        break;
    case BUTTON_VOLUME_LOW:
        Util_startClock((Clock_Struct *)button1DebounceClockHandle);
        break;
    }
}

/*********************************************************************
 * @fn     buttonDebounceSwiFxn
 *
 * @brief  Callback from Clock module on timeout
 *
 *         Determines new state after debouncing
 *
 * @param  buttonId    The pin being debounced
 */
static void buttonDebounceSwiFxn(UArg buttonId)
{
    // Used to send message to app
    pzButtonState_t buttonMsg = { .pinId = buttonId };
    uint8_t sendMsg = FALSE;

    // Get current value of the button pin after the clock timeout
    uint8_t buttonPinVal = PIN_getInputValue(buttonId);

    // Set interrupt direction to opposite of debounced state
    // If button is now released (button is active low, so release is high)
    if(buttonPinVal)
    {
        // Enable negative edge interrupts to wait for press
        PIN_setConfig(buttonPinHandle, PIN_BM_IRQ, buttonId | PIN_IRQ_NEGEDGE);
    }
    else
    {
        // Enable positive edge interrupts to wait for relesae
        PIN_setConfig(buttonPinHandle, PIN_BM_IRQ, buttonId | PIN_IRQ_POSEDGE);
    }

    switch(buttonId)
    {
    case BUTTON_VOLUME_HIGH:
        // If button is now released (buttonPinVal is active low, so release is 1)
        // and button state was pressed (buttonstate is active high so press is 1)
        if(buttonPinVal && button0State)
        {
            // Button was released
            buttonMsg.state = button0State = 0;
            sendMsg = TRUE;
        }
        else if(!buttonPinVal && !button0State)
        {
            // Button was pressed
            buttonMsg.state = button0State = 1;
            sendMsg = TRUE;
        }
        break;

    case BUTTON_VOLUME_LOW:
        // If button is now released (buttonPinVal is active low, so release is 1)
        // and button state was pressed (buttonstate is active high so press is 1)
        if(buttonPinVal && button1State)
        {
            // Button was released
            buttonMsg.state = button1State = 0;
            sendMsg = TRUE;
        }
        else if(!buttonPinVal && !button1State)
        {
            // Button was pressed
            buttonMsg.state = button1State = 1;
            sendMsg = TRUE;
        }
        break;
    }

    if(sendMsg == TRUE)
    {
        pzButtonState_t *pButtonState = ICall_malloc(sizeof(pzButtonState_t));
        if(pButtonState != NULL)
        {
            *pButtonState = buttonMsg;
            if(ProjectZero_enqueueMsg(PZ_BUTTON_DEBOUNCED_EVT, pButtonState) != SUCCESS)
            {
              ICall_free(pButtonState);
            }
        }
    }
}
