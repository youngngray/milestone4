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

SENSOR1_DATA sensor1Data;
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
 
//Puts an unsigned int in milliseconds into the queue
//Returns 0 if successful and 1 if failed
int sensor1SendTimerValToMsgQ(unsigned int millisecondsElapsed)
{
    BaseType_t err_code;
    err_code = xQueueSendToBack( sensor1Data.local_q, &millisecondsElapsed,
                                   portMAX_DELAY );
    if(err_code == pdTRUE)
        return 0;
    else if(err_code == errQUEUE_FULL)
        return 1;
}

void sensor1SendSensorValToSensorQ(unsigned char sensorValue)
{
#ifdef MACRO_DEBUG
    debugChar(0x01);
#endif 
    //debugChar(sensorValue);
    xQueueSendFromISR( sensor1Data.sensor1_q, &sensorValue,
                                   NULL );
#ifdef MACRO_DEBUG
    debugChar(0x02);
#endif
}

unsigned char sensor1ReceiveVal()
{
    unsigned char sensorRead;
    BaseType_t sensorReceived;
    sensorReceived = xQueueReceive(sensor1Data.sensor1_q , &sensorRead, portMAX_DELAY);
    //debugChar(sensorRead);
    //If not received, stop and turn on LED.
    if(sensorReceived == pdFALSE)
    {
        stopEverything();
    }
    return sensorRead;
}

void receiveMsgFormat(unsigned char buffer[])
{
    int i;
    for(i = 0; i < 10; i++)
    {
        if(i == 0)
            msgFormat.header = buffer[i];
        else if(i==1)
            msgFormat.dst = buffer[i];
        else if(i==2)
            msgFormat.type = buffer[i];
        else if(i==3)
            msgFormat.msgNum1 = buffer[i];
        else if(i==4)
            msgFormat.msgNum2 = buffer[i];
        else if(i==5)
            msgFormat.data1 = buffer[i];
        else if(i==6)
            msgFormat.data2 = buffer[i];
        else if(i==7)
            msgFormat.data3 = buffer[i];
        else if(i==8)
            msgFormat.data4 = buffer[i];
        else if(i==9)
            msgFormat.footer = buffer[i];
        else{}
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

void SENSOR1_Initialize ( void )
{
    //stopEverything();
    /* Place the App state machine in its initial state. */
    sensor1Data.state = SENSOR1_STATE_INIT;
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    //Create the queue
    sensor1Data.local_q = xQueueCreate(10, sizeof(unsigned int));
    //Ensure queue was created. If not, do not continue and turn on LED
    if(sensor1Data.local_q == 0)
    {
        stopEverything();
    }
    sensor1Data.sensor1_q = xQueueCreate(100, sizeof(unsigned char));
    if(sensor1Data.sensor1_q == 0)
    {
        stopEverything();
    }
    //stopEverything()
    //Create the timer
    sensor1Data.local_timer = xTimerCreate( "50msTimer",
                50 / portTICK_PERIOD_MS,
                pdTRUE,
                0,
                vTimerCallback );
    
    //Ensure timer was created. If not, do not continue and turn on LED
    if(sensor1Data.local_timer == 0)
    {
        stopEverything();
    }
    BaseType_t started = xTimerStart(sensor1Data.local_timer, 0);
    
    //Ensure the timer started successfully. If not, do not continue and turn
    // on LED
    if(started == pdFAIL)
    {
        stopEverything();
    }   
    
    //Setup AD Driver
    SYS_INT_SourceEnable(INT_SOURCE_ADC_1);
    DRV_ADC_Initialize();
    DRV_ADC_Open();
    DRV_ADC_ChannelScanInputsAdd(ADC_INPUT_SCAN_AN0 | ADC_INPUT_SCAN_AN1|ADC_INPUT_SCAN_AN2);
    PLIB_ADC_MuxAInputScanEnable(ADC_ID_1);
    DRV_ADC_Start();
    
    /* Initialization is done, allow the state machine to continue */
    sensor1Data.state = SENSOR1_STATE_OUTPUT;
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */
void SENSOR1_Tasks ( void )
{
   /* Check the application's current state. */
    switch ( sensor1Data.state )
    {
        /* Application's initial state. */
        case SENSOR1_STATE_INIT:
        {
            break;
        }

        /* The running state. Get value from the queue and output character
         * if necessary */
        case SENSOR1_STATE_OUTPUT:
        {
            //stopEverything();
            //Receive Information from the Queue
#ifdef MACRO_DEBUG
debugChar(0x03);
#endif 
            //Number of elapsed ms.
            unsigned int ms;
            BaseType_t received = xQueueReceive(sensor1Data.local_q , &ms, portMAX_DELAY);
            //If not received, stop and turn on LED.
            if(received == pdFALSE)
            {
                stopEverything();
            }
            
            //Read a value from the xQueue every 10ms
            unsigned char sensorRead;
            BaseType_t sensorReceived;

            sensorReceived = xQueueReceive(sensor1Data.sensor1_q , &sensorRead, portMAX_DELAY);
            //debugChar(sensorRead);

#ifdef MACRO_DEBUG
debugChar(0x04);
#endif
            //If not received, stop and turn on LED.
            if(sensorReceived == pdFALSE)
            {
                stopEverything();
            }
            if(ms % 100 == 0)
            {
                PLIB_ADC_SampleAutoStartEnable(DRV_ADC_ID_1);
            }
            //Once the Task runs once, restart the ISR to read values
            PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_ADC_1);
            
            //SendUSARTMsgToMsgQ("My name is Andrew");
            unsigned char s1msg;
            unsigned char s2msg;
            unsigned char s3msg;
            unsigned char s4msg;

            s1msg = sensor1ReceiveVal();
            s2msg = sensor2ReceiveVal();
            s3msg = sensor3ReceiveVal();
            s4msg = sensor4ReceiveVal();
            unsigned char buffer[10] = {0x81,0x10,0x00,0x00,0x00,0x11,0x22,0x33,0x44,0x88};
            
            receiveMsgFormat(buffer);
//            debugChar(0x55);
//            debugChar(msgFormat.footer);
//            debugBuffer("jelloworld",10);
            //stopEverything();
            sendByteToWIFLY(0x81);
            sendByteToWIFLY(0x01);
            sendByteToWIFLY(0x02);
            sendByteToWIFLY(0x03);
            sendByteToWIFLY(0x04);
            sendByteToWIFLY(0x05);
            sendByteToWIFLY(0x06);
            sendByteToWIFLY(0x07);
            sendByteToWIFLY(0x08);
            sendByteToWIFLY(0x88);
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
