/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include "app_public.h"
// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;
MSG_FORMAT msgFormat;
// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary local functions.
*/

void pushDataQ(unsigned char dataValue) {
    BaseType_t dataRecv = xQueueSendToBack(appData.data_q, &dataValue, portMAX_DELAY );
    if (dataRecv == pdFALSE) {
        stopEverything();
    }
}
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    step = 0;
    //stopEverything();
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    //Create the queue
    appData.local_q = xQueueCreate(10, sizeof(unsigned int));
    //Ensure queue was created. If not, do not continue and turn on LED
    if(appData.local_q == 0)
    {
        stopEverything();
    }
   
    appData.data_q = xQueueCreate(100, sizeof(unsigned char));
    if (appData.data_q == 0) {
        stopEverything();
    }
    //stopEverything()
    //Create the timer
    appData.local_timer = xTimerCreate( "50msTimer",
                50 / portTICK_PERIOD_MS,
                pdTRUE,
                0,
                vTimerCallback );
    
    //Ensure timer was created. If not, do not continue and turn on LED
    if(appData.local_timer == 0)
    {
        stopEverything();
    }
    BaseType_t started = xTimerStart(appData.local_timer, 0);
    
    //Ensure the timer started successfully. If not, do not continue and turn
    // on LED
    if(started == pdFAIL)
    {
        stopEverything();
    }   
    
    //Setup AD Driver
   /* SYS_INT_SourceEnable(INT_SOURCE_ADC_1);
    DRV_ADC_Initialize();
    DRV_ADC_Open();
    DRV_ADC_ChannelScanInputsAdd(ADC_INPUT_SCAN_AN0 | ADC_INPUT_SCAN_AN1|ADC_INPUT_SCAN_AN2);
    PLIB_ADC_MuxAInputScanEnable(ADC_ID_1);
    DRV_ADC_Start();
    */
    /* Initialization is done, allow the state machine to continue */
    appData.state = APP_STATE_OUTPUT;
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */
void APP_Tasks ( void )
{
   /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            break;
        }

        /* The running state. Get value from the queue and output character
         * if necessary */
        case APP_STATE_OUTPUT:
        {
            debugChar(start_app_state_output);
            //Receive Information from the Queue
#ifdef MACRO_DEBUG
debugChar(0x03);
#endif 

#ifdef MACRO_DEBUG
debugChar(0x04);
#endif
            unsigned char forw[10] = {0x81,'L',0x07,0x00,0x00,'F',0x00,0x00,0x00,0x88};
            unsigned char back[10] = {0x81,'L',0x07,0x00,0x00,'B',0x00,0x00,0x00,0x88};
            unsigned char stop[10] = {0x81,'L',0x07,0x00,0x00,'S',0x00,0x00,0x00,0x88};
          
            debugChar(data_q_before_recv);
            BaseType_t recvData = xQueueReceive(appData.data_q , &d, portMAX_DELAY);
            debugChar(data_q_after_recv);
            if (recvData == pdFALSE) {
                stopEverything();
            }
            
            debugChar(before_wifly_send);
            sendMsgToWIFLY(back);
            debugChar(after_wifly_send);
            //PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_ADC_1);
            
            break;
        }
        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }  
}


/*******************************************************************************
 End of File
 */
