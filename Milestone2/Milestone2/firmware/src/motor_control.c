/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    motor_control.c

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

#include "motor_control.h"
#include "motor_control_public.h"
#include "peripheral/oc/plib_oc.h"

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

MOTOR_CONTROL_DATA motor_controlData;

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
//Push onto the q from the ISR
BaseType_t pushEncoderFromISR(unsigned int dataValue)
{
    BaseType_t taskWoken = pdFALSE;
    BaseType_t dataRecv = xQueueSendToBackFromISR(motor_controlData.encoder_q, &dataValue, &taskWoken );
    if (dataRecv == pdFALSE) {
        stopEverything();
    }
    return taskWoken;
}

//Give a distance to move
void pushLengthCommand(unsigned int dataValue)
{
    BaseType_t dataRecv = xQueueSendToBack(motor_controlData.encoder_q, &dataValue, portMAX_DELAY);
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
    void MOTOR_CONTROL_Initialize ( void )

  Remarks:
    See prototype in motor_control.h.
 */

void MOTOR_CONTROL_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    motor_controlData.state = MOTOR_CONTROL_STATE_INIT;
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    //Local variables
    motor_controlData.lwheelint = 0;
    motor_controlData.rwheelint = 0;
    motor_controlData.lwheelpwm = 75;
    motor_controlData.rwheelpwm = 75;
    motor_controlData.stopped = 1;
    //Create the queue
    motor_controlData.encoder_q = xQueueCreate(10, sizeof(unsigned int));
    //Ensure queue was created. If not, do not continue and turn on LED
    if(motor_controlData.encoder_q == 0)
    {
        stopEverything();
    }
    
    motor_controlData.state = MOTOR_CONTROL_STATE_RUN1;
}


/******************************************************************************
  Function:
    void MOTOR_CONTROL_Tasks ( void )

  Remarks:
    See prototype in motor_control.h.
 */
unsigned int distance = 0;
unsigned int total = 0;
void MOTOR_CONTROL_Tasks ( void )
{
    /* Check the application's current state. */
    switch ( motor_controlData.state )
    {
        /* Application's initial state. */
        case MOTOR_CONTROL_STATE_INIT:
        {
            break;
        }

        /* TODO: implement your application state machine.*/
        case MOTOR_CONTROL_STATE_RUN1:
        {
            unsigned int data;
            BaseType_t received = xQueueReceive(motor_controlData.encoder_q,
                    &data, portMAX_DELAY);
            //If not received, stop and turn on LED.
            if(received == pdFALSE)
            {
                stopEverything();
            }
            //Calculation for straight motion
            if((data & 0xFF000000) == 0xFF000000)
            {
                distance = data & 0xFFFF;
                distance = (6 * distance * distance) +
                        (69003 * distance) - 16480;
                distance = distance / 10000;
                total = distance;
                total = total / 2;
            }
            //Calculation for a turn
            if((data & 0xF0F00000) == 0xF0F00000)
            {
                distance = data & 0xFFFF;
                distance = distance * 100;
                distance = distance / 173;
                total = distance;
                total = total / 2;
            }
            
            if((data & 0xFF00) == 0xF000)
            {
                motor_controlData.lwheelint += 1;
                if(distance != 0)
                {
                    distance -= 1;
                    if(distance == total)
                    {
                        unsigned char message4[10] = {0x81, 'C', 0x25, 0, 1, 0, 0, 0, 0, 0x88};
                        //sendMsgToWIFLY(message4, 10);
                    }
                }
            }
            else if((data & 0xFF00) == 0x0F00)
            {
                motor_controlData.rwheelint += 1;
            }
            
            if(distance == 0 && !motor_controlData.stopped)
            {
                motor_controlData.stopped = 1;
                unsigned char message4[10] = {0x81, 'M', 0x26, 0, 1, 0, 0, 0, 0, 0x88};
                sendMsgToWIFLY(message4, 10);
                stop();
            }
            if(distance != 0)
            {
                motor_controlData.stopped = 0;
            }
            
            if(motor_controlData.rwheelint > (motor_controlData.lwheelint + 1))
            {
                motor_controlData.rwheelpwm -= 1;
                PLIB_OC_PulseWidth16BitSet(OC_ID_1, motor_controlData.rwheelpwm);
                motor_controlData.lwheelint = 0;
                motor_controlData.rwheelint = 0;
            }
            else if(motor_controlData.lwheelint > (motor_controlData.rwheelint + 1))
            {
                motor_controlData.rwheelpwm += 1;
                PLIB_OC_PulseWidth16BitSet(OC_ID_1, motor_controlData.rwheelpwm);
                motor_controlData.lwheelint = 0;
                motor_controlData.rwheelint = 0;
            }
            
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
