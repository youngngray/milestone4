/*******************************************************************************
 System Interrupts File

  File Name:
    system_int.c

  Summary:
    Raw ISR definitions.

  Description:
    This file contains a definitions of the raw ISRs required to support the
    interrupt sub-system.

  Summary:
    This file contains source code for the interrupt vector functions in the
    system.

  Description:
    This file contains source code for the interrupt vector functions in the
    system.  It implements the system and part specific vector "stub" functions
    from which the individual "Tasks" functions are called for any modules
    executing interrupt-driven in the MPLAB Harmony system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    interrupt-driven in the system.  These handles are passed into the individual
    module "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2014 released Microchip Technology Inc.  All rights reserved.

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

#include <xc.h>
#include <sys/attribs.h>
#include "app.h"
#include "app_public.h"
#include "debugging_task.h"
#include "messaging_task.h"
#include "system_definitions.h"
#include "motor_control_public.h"

//Local variables
unsigned int lwheelint = 0;
unsigned int rwheelint = 0;

//The interrupt for the timer counter storing left wheel encoder data
void IntHandlerDrvTmrInstance0(void)
{
    lwheelint += 1;
    BaseType_t taskWoken = pdFALSE;
    taskWoken = pushEncoderFromISR((0xF000 | lwheelint));
    if(lwheelint >= 200)
    {
        lwheelint = 0;
        rwheelint = 0;
    }
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_3);
    
    //portEND_SWITCHING_ISR(taskWoken);
}

//The interrupt for the timer counter storing right wheel encoder data
IntHandlerDrvTmrInstance1(void)
{
    rwheelint += 1;
    BaseType_t taskWoken = pdFALSE;
    taskWoken = pushEncoderFromISR((0x0F00 | rwheelint));
    if(rwheelint >= 200)
    {
        lwheelint = 0;
        rwheelint = 0;
    }
    //DRV_OC0_Stop();
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_4);
    //portEND_SWITCHING_ISR(taskWoken);
}

IntHandlerDrvTmrInstance2(void)
{
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_2);
}

void IntHandlerDrvUsartInstance0(void)
{
    BaseType_t taskWoken = pdFALSE;
    //Check for any USART data
    /* Clear pending interrupt */
    if(!isQueueEmpty())
    {
        
        while (!isQueueEmpty() && !PLIB_USART_TransmitterBufferIsFull(USART_ID_1)) {
            unsigned char data = 0x00;
            data = messageQ(&taskWoken);
            debugChar(before_usart_transmit);
            PLIB_USART_TransmitterByteSend(USART_ID_1, data);
            debugChar(after_usart_transmit);
        }
    }    
    else//(PLIB_USART_ReceiverDataIsAvailable(USART_ID_1));
    {
        PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
    }
    if(PLIB_USART_ReceiverDataIsAvailable(USART_ID_1))
    {
        //stopEverything();
        unsigned char byte = 0x00; 
        byte= PLIB_USART_ReceiverByteReceive(USART_ID_1);
//        sendByteToWIFLY(byte);
        taskWoken = ReceiveUSARTMsgFromMsgQ(byte);
//        PLIB_USART_TransmitterByteSend(USART_ID_1, byte);
        //unsigned char byte = 0x00; 
        //byte= PLIB_USART_ReceiverByteReceive(USART_ID_1);
    }
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_RECEIVE);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_ERROR);
    portEND_SWITCHING_ISR(taskWoken);
}

 
/*******************************************************************************
 End of File
*/

