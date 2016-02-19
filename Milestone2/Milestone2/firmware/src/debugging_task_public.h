/* ************************************************************************** */
/** Descriptive File Name
  @Company
    Company Name
  @File Name
    filename.h
  @Summary
    Brief description of the file.
  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#ifndef _DEBUGGING_TASK_PUBLIC_H    /* Guard against multiple inclusion */
#define _DEBUGGING_TASK_PUBLIC_H


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */

/* TODO:  Include other files here if needed. */


/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

    void debugChar(unsigned char toSend);
    void debugCharFromISR(unsigned char toSend);
    void debugBuffer(unsigned char buffer[], unsigned int num);
    void stopEverything( void );
    
    /*DEFINES FOR DEBUGGING************************************************/
    
#define START_OF_ANDREW 0x01
    /*Add your defines here*/
#define END_OF_ANDREW 0x3f
    
#define START_OF_AUSTIN 0x40
    /*Add your defines here*/
#define timer_q_before_recv 0x41
#define timer_q_after_recv 0x42
#define data_q_before_recv 0x43
#define data_q_after_recv 0x44
#define before_wifly_send 0x45
#define after_wifly_send 0x46
#define send_msg_wifly_byte 0x47
#define before_push_data_q 0x48
#define after_push_data_q 0x49
#define reset_msg 0x50
#define done_msg_to_sendmsg_q 0x51
#define start_app_state_output 0x52
#define before_usart_transmit 0x53
#define after_usart_transmit 0x54
#define FOUND_COUNT_WRONG 0x7e
#define END_OF_AUSTIN 0x7f
    
#define START_OF_MITCHELL 0x80
    /*Add your defines here*/
#define END_OF_MITCHELL 0xbf
    
#define START_OF_TOM 0xc0
    /*Add your defines here*/
#define END_OF_TOM 0xff
    
    /*END OF DEFINES*********************************************************/
    
    
    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _DEBUGGING_TASK_PUBLIC_H */

/* *****************************************************************************
 End of File
 */