/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    sensor4.c

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

#include "sensor4.h"

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

SENSOR4_DATA sensor4Data;

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
//Puts an unsigned int in milliseconds into the queue
//Returns 0 if successful and 1 if failed
int sensor4SendTimerValToMsgQ(unsigned int millisecondsElapsed)
{
    BaseType_t err_code;
    err_code = xQueueSendToBack( sensor4Data.local_q, &millisecondsElapsed,
                                   portMAX_DELAY );
    if(err_code == pdTRUE)
        return 0;
    else if(err_code == errQUEUE_FULL)
        return 1;
}

void sensor4SendSensorValToSensorQ(unsigned char sensorValue)
{
#ifdef MACRO_DEBUG
    debugChar(0x01);
#endif 
    //debugChar(sensorValue);
    xQueueSendFromISR( sensor4Data.sensor4_q, &sensorValue,
                                   NULL );
#ifdef MACRO_DEBUG
    debugChar(0x02);
#endif
}
unsigned char sensor4ReceiveVal()
{
    unsigned char sensorRead;
    BaseType_t sensorReceived;
    sensorReceived = xQueueReceive(sensor4Data.sensor4_q , &sensorRead, portMAX_DELAY);
    //debugChar(sensorRead);
    //If not received, stop and turn on LED.
    if(sensorReceived == pdFALSE)
    {
        stopEverything();
    }
    return sensorRead;
}
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void SENSOR4_Initialize ( void )

  Remarks:
    See prototype in sensor4.h.
 */

void SENSOR4_Initialize ( void )
{
   //stopEverything();
    /* Place the App state machine in its initial state. */
    sensor4Data.state = SENSOR4_STATE_INIT;
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    //Create the queue
    sensor4Data.local_q = xQueueCreate(10, sizeof(unsigned int));
    //Ensure queue was created. If not, do not continue and turn on LED
    if(sensor4Data.local_q == 0)
    {
        stopEverything();
    }
    sensor4Data.sensor4_q = xQueueCreate(100, sizeof(unsigned char));
    if(sensor4Data.sensor4_q == 0)
    {
        stopEverything();
    }
    //stopEverything();
    //Create the timer
    sensor4Data.local_timer = xTimerCreate( "50msTimer",
                50 / portTICK_PERIOD_MS,
                pdTRUE,
                0,
                vTimerCallback );
    
    //Ensure timer was created. If not, do not continue and turn on LED
    if(sensor4Data.local_timer == 0)
    {
        stopEverything();
    }
    BaseType_t started = xTimerStart(sensor4Data.local_timer, 0);
    
    //Ensure the timer started successfully. If not, do not continue and turn
    // on LED
    if(started == pdFAIL)
    {
        stopEverything();
    }   
    
    /* Initialization is done, allow the state machine to continue */
    sensor4Data.state = SENSOR1_STATE_OUTPUT;
}


/******************************************************************************
  Function:
    void SENSOR4_Tasks ( void )

  Remarks:
    See prototype in sensor4.h.
 */

void SENSOR4_Tasks ( void )
{
    /* Check the application's current state. */
    switch ( sensor4Data.state )
    {
        /* Application's initial state. */
        case SENSOR4_STATE_INIT:
        {
            break;
        }

                /* TODO: implement your application state machine.*/
        case SENSOR2_STATE_OUTPUT:
        {
            //stopEverything();
            //Receive Information from the Queue
#ifdef MACRO_DEBUG
debugChar(0x0C);
#endif 
            //Number of elapsed ms.
            unsigned int ms;
            BaseType_t received = xQueueReceive(sensor4Data.local_q , &ms, portMAX_DELAY);
            //If not received, stop and turn on LED.
            if(received == pdFALSE)
            {
                stopEverything();
            }
            
            //Read a value from the xQueue every 10ms
            unsigned char sensorRead;
            BaseType_t sensorReceived;

            sensorReceived = xQueueReceive(sensor4Data.sensor4_q , &sensorRead, portMAX_DELAY);
            //debugChar(sensorRead);

#ifdef MACRO_DEBUG
debugChar(0x0D);
#endif
            //If not received, stop and turn on LED.
            if(sensorReceived == pdFALSE)
            {
                stopEverything();
            }
            if(ms % 100 == 0)
            {
                
            }
            
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
