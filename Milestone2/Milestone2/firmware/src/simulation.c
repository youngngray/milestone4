/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    simulation.c

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

#include "simulation.h"

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

SIMULATION_DATA simulationData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback funtions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************
//#define SIM1
//#define SIM2
//#define SIM3
#define NOSIM
/*******************************************************************************
  Function:
    void SIMULATION_Initialize ( void )

  Remarks:
    See prototype in simulation.h.
 */

void SIMULATION_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    simulationData.state = SIMULATION_STATE_INIT;
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
#ifdef SIM1
    simulationData.state = SIMULATION_STATE_BEGIN;
#endif
#ifdef SIM2
    simulationData.state = SIMULATION_STATE_THIRD;
#endif
#ifdef SIM3
    simulationData.state = SIMULATION_STATE_SECOND;
#endif
#ifdef NOSIM
    simulationData.state = SIMULATION_STATE_INIT;
#endif
}


/******************************************************************************
  Function:
    void SIMULATION_Tasks ( void )

  Remarks:
    See prototype in simulation.h.
 */

void SIMULATION_Tasks ( void )
{
    /* Check the application's current state. */
    switch ( simulationData.state )
    {
        /* Application's initial state. */
        case SIMULATION_STATE_INIT:
        {
            break;
        }
        case SIMULATION_STATE_BEGIN:
        {
            //pushLengthCommand((0xFF000000 + 0x6A)); //50
            //forward();
            unsigned char message[10] = {0x81, 'L', 0x21, 0, 1, 0, 0x10, 0, 0, 0x88};
            sendMsgToWIFLY(message, 10);
            simulationData.state = SIMULATION_STATE_INIT;
            break;
        }
        case SIMULATION_STATE_SECOND:
        {
            unsigned char message1[10] = {0x81, 'L', 0x23, 0, 1, 0, 0x5A, 0, 0, 0x88};
            sendMsgToWIFLY(message1, 10);
            vTaskDelay(5000);
            unsigned char message2[10] = {0x81, 'L', 0x23, 0, 2, 0, 0x5A, 0, 0, 0x88};
            sendMsgToWIFLY(message2, 10);
            vTaskDelay(5000);
            unsigned char message3[10] = {0x81, 'L', 0x23, 0, 3, 0, 0x5A, 0, 0, 0x88};
            sendMsgToWIFLY(message3, 10);
            vTaskDelay(5000);
            unsigned char message4[10] = {0x81, 'L', 0x23, 0, 4, 0, 0x5A, 0, 0, 0x88};
            sendMsgToWIFLY(message4, 10);
            vTaskDelay(5000);
            unsigned char message5[10] = {0x81, 'L', 0x23, 0, 5, 0, 0x5A, 0, 0, 0x88};
            sendMsgToWIFLY(message5, 10);
            vTaskDelay(5000);
            unsigned char message6[10] = {0x81, 'L', 0x23, 0, 6, 0, 0x5A, 0, 0, 0x88};
            sendMsgToWIFLY(message6, 10);
            vTaskDelay(5000);
            unsigned char message7[10] = {0x81, 'L', 0x23, 0, 7, 0, 0x5A, 0, 0, 0x88};
            sendMsgToWIFLY(message7, 10);
            vTaskDelay(5000);
            unsigned char message8[10] = {0x81, 'L', 0x23, 0, 8, 0, 0x5A, 0, 0, 0x88};
            sendMsgToWIFLY(message8, 10);
            vTaskDelay(5000);
            //unsigned char message[10] = {0x81, 'L', 0x20, 0, 1, 'S', 0, 0, 0, 0x88};
            //sendMsgToWIFLY(message, 10);
            simulationData.state = SIMULATION_STATE_INIT;
            break;
        }
        case SIMULATION_STATE_THIRD:
        {
            //pushLengthCommand((0xFF000000 + 0xC0)); //50
            //forward();
            unsigned char message[10] = {0x81, 'L', 0x21, 0, 1, 0, 0x1C, 0, 0, 0x88};
            sendMsgToWIFLY(message, 10);
            vTaskDelay(9000);
            //pushLengthCommand((0xFF000000 + 0x32));
            //right();
            unsigned char message1[10] = {0x81, 'L', 0x24, 0, 1, 0, 0x5A, 0, 0, 0x88};
            sendMsgToWIFLY(message1, 10);
            vTaskDelay(7000);
            //pushLengthCommand((0xFF000000 + 0x50));
            //forward();
            unsigned char message2[10] = {0x81, 'L', 0x21, 0, 2, 0, 0x0B, 0, 0, 0x88};
            sendMsgToWIFLY(message2, 10);
            vTaskDelay(7000);
            //pushLengthCommand((0xFF000000 + 0x32));
            //left();
            unsigned char message3[10] = {0x81, 'L', 0x23, 0, 1, 0, 0x5A, 0, 0, 0x88};
            sendMsgToWIFLY(message3, 10);
            vTaskDelay(7000);
            //pushLengthCommand((0xFF000000 + 0x60));
            //reverse();
            unsigned char message4[10] = {0x81, 'L', 0x22, 0, 1, 0, 0xFF, 0, 0, 0x88};
            sendMsgToWIFLY(message4, 10);
            simulationData.state = SIMULATION_STATE_INIT;
            break;
        }
        case SIMULATION_STATE_FOURTH:
        {
            //vTaskDelay(8000);
            //unsigned char message[10] = {0x81, 'L', 0x22, 0, 1, 'B', 0, 0, 0, 0x88};
            //sendMsgToWIFLY(message, 10);
            //simulationData.state = SIMULATION_STATE_FIFTH;
            unsigned int read = 0;
            read = PLIB_PORTS_Read(PORTS_ID_0, PORT_CHANNEL_D);
            if(read & 0x04)
            {
                unsigned char message4[10] = {0x81, 'L', 0x20, 0, 1, 0, 0, 0, 0, 0x88};
                sendMsgToWIFLY(message4, 10);
            }
            break;
        }
        case SIMULATION_STATE_FIFTH:
        {
            vTaskDelay(8000);
            unsigned char message[10] = {0x81, 'L', 0x24, 0, 1, 'R', 0, 0, 0, 0x88};
            sendMsgToWIFLY(message, 10);
            simulationData.state = SIMULATION_STATE_INIT;
            break;
        }

        /* TODO: implement your application state machine.*/

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
