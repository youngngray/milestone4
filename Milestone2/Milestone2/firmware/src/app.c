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
#define ALLOW_MOVEMENT
void forward(void)
{
#ifdef ALLOW_MOVEMENT
    DRV_OC1_Start();
    DRV_OC0_Start();
    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1, 0);
    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14, 0);
#endif
}

void reverse(void)
{
    #ifdef ALLOW_MOVEMENT
    DRV_OC1_Start();
    DRV_OC0_Start();
    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1, 1);
    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14, 1);
#endif
}

void right(void)
{
    #ifdef ALLOW_MOVEMENT
    DRV_OC1_Start();
    DRV_OC0_Start();
    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1, 0);
    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14, 1);
#endif
}

void left(void)
{
    #ifdef ALLOW_MOVEMENT
    DRV_OC1_Start();
    DRV_OC0_Start();
    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1, 1);
    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14, 0);  
#endif
}

void stop(void)
{
    DRV_OC1_Stop();
    DRV_OC0_Stop();
}

void stopleft(void)
{
    DRV_OC1_Stop();
}

void stopright(void)
{
    DRV_OC0_Stop();
}

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
    //stop();
   
    //stopEverything();
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    DRV_TMR0_Initialize();
    DRV_TMR0_Start();
    DRV_TMR1_Initialize();
    DRV_TMR1_Start();
    DRV_TMR2_Initialize();
    DRV_TMR2_Start();
    DRV_OC0_Initialize();
    DRV_OC1_Initialize();
   
    appData.data_q = xQueueCreate(100, sizeof(unsigned char));
    if (appData.data_q == 0) {
        stopEverything();
    }
    //stopEverything() 
    appData.num_commands = 0;
    
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
            
            /* OLD CODE: IGNORE
            unsigned char forw[10] = {0x81,'L',0x07,0x00,0x00,'F',0x00,0x00,0x00,0x88};
            unsigned char ba[10] = {0x81,'L',0x07,0x00,0x00,'B',0x00,0x00,0x00,0x88};
            unsigned char st[10] = {0x81,'L',0x07,0x00,0x00,'S',0x00,0x00,0x00,0x88};
             
            //unsigned char pickup_token[10] = {'1','M', '2', '3', '4', '5', '6', '7', '8', '9'};
            //sendMsgToWIFLY(pickup_token, 10);
            unsigned char message[10] = {0x81, 'L', 0x04, 0, 1, 'F', 'N', 'D', 'T', 0x88};
            //sendMsgToWIFLY(message, 10);
            //sendMsgToWIFLY(pickup_token, 10);
             */
            unsigned char command;
            BaseType_t received = xQueueReceive(appData.data_q,
                    &command, portMAX_DELAY);
            //If not received, stop and turn on LED.
            if(received == pdFALSE)
            {
                stopEverything();
            }
            
#ifdef ALLOW_MOVEMENT
            switch(command)
            {
                case 'F':
                {
                    forward();
                    debugChar('F');
                    break;
                }
                case 'S':
                {
                    stop();
                    debugChar('S');
                    break;
                }
                case 'L':
                {
                    left();
                    debugChar('L');
                    break;
                }
                case 'B':
                {
                    reverse();
                    debugChar('B');
                    break;
                }
                case 'R':
                {
                    right();
                    debugChar('R');
                    break;
                }
                default:
                {
                    break;
                }
            }
#endif
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
