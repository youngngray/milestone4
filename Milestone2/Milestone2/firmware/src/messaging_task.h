/* 
 * File:   messaging.h
 * Author: Andrew Wang
 *
 * Created on February 10, 2016, 5:05 PM
 */

#ifndef MESSAGING_H
#define	MESSAGING_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"

#include "timers.h"
#include "queue.h"
#include "debugging_task_public.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/

typedef enum
{
	/* Application's state machine's initial state. */
	MESSAGING_TASK_STATE_INIT = 0,

	/* TODO: Define states used by the application state machine. */
    MESSAGING_TASK_STATE_RUN = 1,

    MESSAGING_TASK_STATE_READ = 2,
} MESSAGING_TASK_STATES;

int isQueueEmpty();
unsigned char messageQ(BaseType_t *taskWoken);
/*******************************************************************************
  Function:
    void vTimerCallback( TimerHandle_t pxTimer )

  Summary:
 Callback routine for the local timer

  Description:
    This function is called whenever the timer rolls over. This happens
 once per 50ms. It then sends a time value to the local queue.

  Precondition:
 The timer should be setup to call this routine.

  Parameters:
    None.

  Returns:
    None.

  Example:
 None.

  Remarks:
    This routine must be called from the timer.
*/
void vTimerCallback( TimerHandle_t pxTimer );

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef struct
{
    /* The application's current state */
    MESSAGING_TASK_STATES state;

    /* TODO: Define any additional data used by the application. */
    QueueHandle_t sendMsg_q;
    QueueHandle_t receiveMsg_q;
} MESSAGING_TASK_DATA;

typedef struct
{
    int count;
    int validHeader;
    int validFooter;
    unsigned char header;
    unsigned char dst;
    unsigned char type;
    unsigned char msgNum1;
    unsigned char msgNum2;
    unsigned char data1;
    unsigned char data2;
    unsigned char data3;
    unsigned char data4;
    unsigned char footer;
    unsigned char valid;
    unsigned int command_count;
    unsigned int _20_count;
    unsigned int _21_count;
    unsigned int _22_count;
    unsigned int _23_count;
    unsigned int _24_count;
    int token_pickup;
    int debug_count;
    int half_count;
    int done_count;
    int numInvalid;
} MESSAGE_FORMAT;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/

	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void MESSAGING_TASK_Initialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the 
    application in its initial state and prepares it to run so that its 
    APP_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    MESSAGING_TASK_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void MESSAGING_TASK_Initialize ( void );


/*******************************************************************************
  Function:
    void MESSAGING_TASK_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    MESSAGING_TASK_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void MESSAGING_TASK_Tasks( void );

#endif	/* MESSAGING_H */

