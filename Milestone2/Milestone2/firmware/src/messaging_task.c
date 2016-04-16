/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    messaging_task.c

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

MESSAGING_TASK_DATA msg_taskData;
MESSAGE_FORMAT msg_Format;
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
void sendMsgToWIFLY(unsigned char message[], int num)
{
    if (num == 10 && message[0] == 0x81){
        if (message[2] == 0x01) {
            msg_Format.token_pickup++;
            message[3] = msg_Format.token_pickup >> 8;
            message[4] = (unsigned char) msg_Format.token_pickup;
        }
        if (message[2] == 0x06) {
            msg_Format.debug_count++;
            message[3] = msg_Format.debug_count >> 8;
            message[4] = (unsigned char) msg_Format.debug_count;
        }
        if(message[2] == 0x25) {
            msg_Format.half_count++;
            message[3] = msg_Format.half_count >> 8;
            message[4] = (unsigned char) msg_Format.half_count;
        }
        if(message[2] == 0x26) {
            msg_Format.done_count++;
            message[3] = msg_Format.done_count >> 8;
            message[4] = (unsigned char) msg_Format.done_count;
        }
    }
    int i;
    for(i = 0; i < num; i++)
    {
        sendByteToWIFLY(message[i]);
    }
    //PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
}

void sendByteToWIFLY(unsigned char byte)
{
    debugChar(send_msg_wifly_byte);
    BaseType_t err = xQueueSendToFront(msg_taskData.sendMsg_q, &byte, portMAX_DELAY);
    if(err != pdTRUE)
    {
        stopEverything();
    }
    debugChar(done_msg_to_sendmsg_q);
    PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
}

BaseType_t ReceiveUSARTMsgFromMsgQ(unsigned char usartMsg)
{
    BaseType_t taskWoken = pdFALSE;
    BaseType_t err = xQueueSendToBackFromISR(msg_taskData.receiveMsg_q, &usartMsg, &taskWoken);
    if(err != pdPASS)
    {
        stopEverything();
    }
    //stopEverything();
    return taskWoken;
}
/* TODO:  Add any necessary local functions.
*/
int isQueueEmpty()
{
    if(pdFALSE == xQueueIsQueueEmptyFromISR(msg_taskData.sendMsg_q))
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

unsigned char messageQ(BaseType_t *taskWoken)
{
    unsigned char data = 0x0;
    BaseType_t errors;
    errors = xQueueReceiveFromISR(msg_taskData.sendMsg_q, &data, taskWoken);
    if(errors != pdTRUE)
    {
        stopEverything();
    }
    return data;
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void MESSAGING_TASK_Initialize ( void )

  Remarks:
    See prototype in messaging_task.h.
 */

void MESSAGING_TASK_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    msg_taskData.state = MESSAGING_TASK_STATE_INIT;
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    msg_taskData.sendMsg_q = xQueueCreate(50, sizeof(unsigned char));
    if(msg_taskData.sendMsg_q == 0)
    {
        stopEverything();
    }
    msg_taskData.receiveMsg_q = xQueueCreate(50, sizeof(unsigned char));
    if(msg_taskData.sendMsg_q == 0)
    {
        stopEverything();
    }
    msg_Format.count = 0;
    msg_Format.validHeader = 0;
    msg_Format.validFooter = 0;
    msg_Format.numInvalid = 0;
    msg_Format.command_count = 0;
    msg_Format._20_count = 0;
    msg_Format._21_count = 0;
    msg_Format._22_count = 0;
    msg_Format._23_count = 0;
    msg_Format._24_count = 0;
    msg_Format.debug_count = 0;
    msg_Format.token_pickup = 0;
    msg_Format.half_count = 0;
    //stopEverything();
    /* Initialization is done, allow the state machine to continue */
    msg_taskData.state = MESSAGING_TASK_STATE_RUN;
    
#ifdef MACRO_DEBUG
      debugChar(0x09);      
#endif
      //stopEverything();
}


/******************************************************************************
  Function:
    void MESSAGING_TASK_Tasks ( void )

  Remarks:
    See prototype in messaging_task.h.
 */

void MESSAGING_TASK_Tasks ( void )
{
    /* Check the application's current state. */
    switch ( msg_taskData.state )
    {
        /* Application's initial state. */
        case MESSAGING_TASK_STATE_INIT:
        {
            break;
        }

        /* TODO: implement your application state machine.*/
        case MESSAGING_TASK_STATE_RUN:
        {
            //stopEverything();
#ifdef MACRO_DEBUG
      debugChar(0x07);      
#endif
            unsigned char temp;
            xQueueReceive(msg_taskData.receiveMsg_q, &temp, portMAX_DELAY);
#ifdef MACRO_DEBUG
      debugBuffer(0x08);      
#endif
      
            if((temp == 0x81) && (msg_Format.count == 0))
            {
//                sendByteToWIFLY(temp);
#ifdef MACRO_DEBUG
                debugChar(0xD1);
#endif
                //sendByteToWIFLY(msg_Format.header);
                msg_Format.validHeader = 5;
                msg_Format.header = temp;
                msg_Format.count++;
            }
            else if ((temp != 0x81) && (msg_Format.count == 0))
            {
                //Invalid Message
//                sendByteToWIFLY(temp);
#ifdef MACRO_DEBUG
                debugChar(0xF3);
#endif
                msg_Format.numInvalid++;
                //sendMsgToWIFLY("NO\t");
                //sendByteToWIFLY(msg_Format.numInvalid);
                //sendByteToWIFLY('\n');            
                msg_Format.validHeader = 0;
                msg_Format.validFooter = 0;
                msg_Format.count = 0;
            }
            if((msg_Format.validHeader == 5) && (msg_Format.count != 0))
            {
#ifdef MACRO_DEBUG
                debugChar(0xEE);
#endif
//                sendByteToWIFLY(temp);
                if(msg_Format.count == 2){
#ifdef MACRO_DEBUG
                    debugChar(0xD2);
#endif
//                    sendByteToWIFLY(temp);
                    msg_Format.dst = temp;
                }
                else if(msg_Format.count == 3) {
#ifdef MACRO_DEBUG
                    debugChar(0xD3);
#endif
//                    sendByteToWIFLY(temp);
                    msg_Format.type = temp;
                }
                else if(msg_Format.count == 4) {
#ifdef MACRO_DEBUG
                    debugChar(0xD4);
#endif
//                    sendByteToWIFLY(temp);
                    msg_Format.msgNum1 = temp;
                }
                else if(msg_Format.count == 5) {
#ifdef MACRO_DEBUG
                    debugChar(0xD5);
#endif
//                    sendByteToWIFLY(temp);
                    msg_Format.msgNum2 = temp;
                }
                else if(msg_Format.count == 6) {
#ifdef MACRO_DEBUG
                    debugChar(0xD6);
#endif
//                    sendByteToWIFLY(temp);
                    msg_Format.data1 = temp;
                }
                else if(msg_Format.count == 7) {
#ifdef MACRO_DEBUG
                    debugChar(0xD7);
#endif
//                    sendByteToWIFLY(temp);
                    msg_Format.data2 = temp;
                }
                else if(msg_Format.count == 8) {
#ifdef MACRO_DEBUG
                    debugChar(0xD8);
#endif
//                    sendByteToWIFLY(temp);
                    msg_Format.data3 = temp;
                }
                else if(msg_Format.count == 9) {
#ifdef MACRO_DEBUG
                    debugChar(0xD9);
#endif
//                    sendByteToWIFLY(temp);
                    msg_Format.data4 = temp;
                    
                }
                msg_Format.count++;
            }
            if((msg_Format.validHeader == 5)&&(temp == 0x88)&&(msg_Format.count == 11))
            {
#ifdef MACRO_DEBUG
                debugChar(0xE0);
#endif

                msg_Format.footer = temp;
                msg_Format.validFooter = 5;
                msg_Format.count = 0;
            }
            else if((msg_Format.count == 11) && (msg_Format.validHeader == 5) && (temp != 0x88))
            {
#ifdef MACRO_DEBUG
                debugChar(0xF1);
#endif
                
                msg_Format.numInvalid++;              
                msg_Format.validHeader = 0;
                msg_Format.valid = 0;
                msg_Format.validFooter = 0;
                msg_Format.count = 0;
            }
            if((msg_Format.validHeader == 5) && (msg_Format.validFooter == 5))
            {
                msg_Format.valid = 1;
                msg_Format.count = 0;
            }
      
          if(msg_Format.valid == 1)
          {

              msg_Format.count = 0;
              msg_taskData.state = MESSAGING_TASK_STATE_READ;
          }

            break;
        }
        case MESSAGING_TASK_STATE_READ:
        {
#ifdef MACRO_DEBUG
            debugChar(msg_Format.header);
            debugChar(msg_Format.dst);
            debugChar(msg_Format.type);
            debugChar(msg_Format.msgNum1);
            debugChar(msg_Format.msgNum2);
            debugChar(msg_Format.data1);
            debugChar(msg_Format.data2);
            debugChar(msg_Format.data3);
            debugChar(msg_Format.data4);
            debugChar(msg_Format.footer);
#endif
            //Old type system... Ignore, not used
            if(msg_Format.type == 0x04)
            {
                msg_Format.command_count++;
                unsigned int num_received = (msg_Format.msgNum1 << 8) + msg_Format.msgNum2;
                if(msg_Format.command_count != num_received)
                {
                    //Handle error message
                    unsigned char message[10] = {0x81, 'M', 0x06, 0, 0, 'C', 'O', 'F', 'F', 0x88};
                    sendMsgToWIFLY(message, 10);
                    //Re-synch with sent message
                    msg_Format.command_count = num_received;
                }
                else
                {
                    pushDataQ(msg_Format.data1);
                }
            }
            
            //Type 0x20 is a Stop
            else if(msg_Format.type == 0x20)
            {
                msg_Format._20_count++;
                unsigned int num_received = (msg_Format.msgNum1 << 8) + msg_Format.msgNum2;
                if(msg_Format._20_count != num_received)
                {
                    //Handle error message
                    unsigned char message[10] = {0x81, 'M', 0x06, 0, 0, 'C', 'O', 'F', 'F', 0x88};
                    sendMsgToWIFLY(message, 10);
                    //Re-synch with sent message
                    msg_Format._20_count = num_received;
                }
                else
                {
                    pushLengthCommand(0xFF000000);
                    pushDataQ('S');
                }
            }
            
            //Type 0x21 is a forward
            else if(msg_Format.type == 0x21)
            {
                msg_Format._21_count++;
                unsigned int num_received = (msg_Format.msgNum1 << 8) + msg_Format.msgNum2;
                if(msg_Format._21_count != num_received)
                {
                    //Handle error message
                    unsigned char message[10] = {0x81, 'M', 0x06, 0, 0, 'C', 'O', 'F', 'F', 0x88};
                    sendMsgToWIFLY(message, 10);
                    //Re-synch with sent message
                    msg_Format._21_count = num_received;
                }
                else
                {
                    pushLengthCommand((0xFF000000 | (msg_Format.data1 << 8) | msg_Format.data2));
                    pushDataQ('F');
                }
            }
            
            //type 0x22 is a backward
            else if(msg_Format.type == 0x22)
            {
                msg_Format._22_count++;
                unsigned int num_received = (msg_Format.msgNum1 << 8) + msg_Format.msgNum2;
                if(msg_Format._22_count != num_received)
                {
                    //Handle error message
                    unsigned char message[10] = {0x81, 'M', 0x06, 0, 0, 'C', 'O', 'F', 'F', 0x88};
                    sendMsgToWIFLY(message, 10);
                    //Re-synch with sent message
                    msg_Format._22_count = num_received;
                }
                else
                {
                    pushLengthCommand((0xFF000000 | (msg_Format.data1 << 8) | msg_Format.data2));
                    pushDataQ('B');
                }
            }
            
            //type 0x23 is left
            else if(msg_Format.type == 0x23)
            {
                msg_Format._23_count++;
                unsigned int num_received = (msg_Format.msgNum1 << 8) + msg_Format.msgNum2;
                if(msg_Format._23_count != num_received)
                {
                    //Handle error message
                    unsigned char message[10] = {0x81, 'M', 0x06, 0, 0, 'C', 'O', 'F', 'F', 0x88};
                    sendMsgToWIFLY(message, 10);
                    //Re-synch with sent message
                    msg_Format._23_count = num_received;
                }
                else
                {
                    pushLengthCommand((0xF0F00000 | (msg_Format.data1 << 8) | msg_Format.data2));
                    pushDataQ('L');
                }
            }
            
            //type 0x24 is right
            else if(msg_Format.type == 0x24)
            {
                msg_Format._24_count++;
                unsigned int num_received = (msg_Format.msgNum1 << 8) + msg_Format.msgNum2;
                if(msg_Format._24_count != num_received)
                {
                    //Handle error message
                    unsigned char message[10] = {0x81, 'M', 0x06, 0, 0, 'C', 'O', 'F', 'F', 0x88};
                    sendMsgToWIFLY(message, 10);
                    //Re-synch with sent message
                    msg_Format._24_count = num_received;
                }
                else
                {
                    pushLengthCommand((0xF0F00000 | (msg_Format.data1 << 8) | msg_Format.data2));
                    pushDataQ('R');
                }
            }
            
            msg_Format.count = 0;
            msg_Format.validHeader = 0;
            msg_Format.validFooter = 0;
            msg_Format.valid = 0;
            debugChar(reset_msg);
            msg_taskData.state = MESSAGING_TASK_STATE_RUN;
            break;
        }
        /* The default state should never be executed. */
        default:
        {
            break;
        }
    }
}
 

/*******************************************************************************
 End of File
 */
