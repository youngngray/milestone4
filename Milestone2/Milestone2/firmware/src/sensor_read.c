/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    sensor_read.c

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

#include "sensor_read.h"
#include "simulation.h"
#include "messaging_task.h"
#include "messaging_task_public.h"

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

SENSOR_READ_DATA sensor_readData;

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

/*******************************************************************************
  Function:
    void SENSOR_READ_Initialize ( void )

  Remarks:
    See prototype in sensor_read.h.
 */

void SENSOR_READ_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    sensor_readData.state = SENSOR_READ_STATE_INIT;
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    sensor_readData.token_found = 0;
    sensor_readData.tokens = 0;
    sensor_readData.history = 0;
    
    sensor_readData.state = SENSOR_READ_STATE_CAPTURE;
}


/******************************************************************************
  Function:
    void SENSOR_READ_Tasks ( void )

  Remarks:
    See prototype in sensor_read.h.
 */

void SENSOR_READ_Tasks ( void )
{
    /* Check the application's current state. */
    switch ( sensor_readData.state )
    {
        /* Application's initial state. */
        case SENSOR_READ_STATE_INIT:
        {
            break;
        }

        /* TODO: implement your application state machine.*/
        case SENSOR_READ_STATE_CAPTURE:
        {
            //Steps described by manufacturer to use QTR-8 Sensor Array
            //1. Turn on sensor
            PLIB_PORTS_PinWrite(PORTS_ID_0 , PORT_CHANNEL_B, PORTS_BIT_POS_11, 1);
            //2. Set I/O Line to an Output and Drive it High
            //               This code works because the necessary pins are
            //                      already configured as outputs in harmony.
            PLIB_PORTS_DirectionOutputSet( PORTS_ID_0, PORT_CHANNEL_G,  SYS_PORT_G_TRIS ^ 0xFFFF);
            PLIB_PORTS_DirectionOutputSet( PORTS_ID_0, PORT_CHANNEL_A,  SYS_PORT_A_TRIS ^ 0xFFFF);
            PLIB_PORTS_DirectionOutputSet( PORTS_ID_0, PORT_CHANNEL_F,  SYS_PORT_F_TRIS ^ 0xFFFF);
            PLIB_PORTS_DirectionOutputSet( PORTS_ID_0, PORT_CHANNEL_D,  SYS_PORT_D_TRIS ^ 0xFFFF);
            
            PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_13, 1);
            PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_B, PORTS_BIT_POS_12, 1);
            PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_8, 1);
            PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_7, 1);
            PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_10, 1);
            PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_F, PORTS_BIT_POS_0, 1);
            PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_F, PORTS_BIT_POS_1, 1);
            PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_6, 1);
            PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_8, 1);
            PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_11, 1);
            //3. Allow some time to pass for sensor output to rise
            vTaskDelay(1);
            //4. Make the I/O Line an input (HI-Z)
            PLIB_PORTS_DirectionInputSet(PORTS_ID_0, PORT_CHANNEL_G, 0x0180);
            PLIB_PORTS_DirectionInputSet(PORTS_ID_0, PORT_CHANNEL_A, 0x0400);
            PLIB_PORTS_DirectionInputSet(PORTS_ID_0, PORT_CHANNEL_F, 0x0003);
            PLIB_PORTS_DirectionInputSet(PORTS_ID_0, PORT_CHANNEL_D, 0x0940);
            
            //5. Measure voltage to go low.
            
            /* TWEAK THIS VALUE TO CHANGE SENSITIVITY OF SENSOR */
            /*I am simply waiting 15ms then measuring. If the value is low, we
             are on white, if the value is high, then we are on black.*/
            //Lower detects more
            vTaskDelay(18);
            
            unsigned int read[5] = {0};
            read[1] = PLIB_PORTS_Read(PORTS_ID_0, PORT_CHANNEL_G);
            read[2] = PLIB_PORTS_Read(PORTS_ID_0, PORT_CHANNEL_A);
            read[3] = PLIB_PORTS_Read(PORTS_ID_0, PORT_CHANNEL_F);
            read[4] = PLIB_PORTS_Read(PORTS_ID_0, PORT_CHANNEL_D);
            unsigned char count = 0;
            
            //6. Turn off sensor
            PLIB_PORTS_PinWrite(PORTS_ID_0 , PORT_CHANNEL_B, PORTS_BIT_POS_11, 0);
            
            //If any 2 of 8 is true, then count as success
            if(read[1] & 0x0080)
            {
                count++;
            }
            if(read[4] & 0x0800)
            {
                count++;
            }
            if(read[1] & 0x0100)
            {
                count++;
            }
            if(read[2] & 0x0400)
            {
                count++;
            }
            if(read[3] & 0x0001)
            {
                count++;
            }
            if(read[3] & 0x0002)
            {
                count++;
            }
            if(read[4] & 0x0040)
            {
                count++;
            }
            if(read[4] & 0x0100)
            {
                count++;
            }
            
            if(count >= 2)
            {
                sensor_readData.history = (sensor_readData.history << 1) + 1;
                if(sensor_readData.history == 0xFF)
                {
                    if(!sensor_readData.token_found)
                    {
                        unsigned char message[10] = {0x81, 'M', 0x01, 0, 0, 0, 0, 0, 0, 0x88};
                        sendMsgToWIFLY(message, 10);
                        PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_3, 1);
                        sensor_readData.tokens += 1;
                        sensor_readData.token_found = 1;
                        /*This was used in Milestone 4 to stop after the third 
                         * token was found. Removed for Final Demo.
                        if(sensor_readData.tokens == 3)
                        {
                            vTaskDelay(25);
                            unsigned char message[10] = {0x81, 'L', 0x20, 0, 1, 0, 0, 0, 0, 0x88};
                            sendMsgToWIFLY(message, 10);
                        }
                         * */
                    }
                }
            }
            else
            {
                sensor_readData.history = (sensor_readData.history << 1) + 0;
                if(sensor_readData.history == 0x00)
                {
                    PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_3, 0);
                    sensor_readData.token_found = 0;
                }
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
