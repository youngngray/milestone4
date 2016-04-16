/* ************************************************************************** */
/** Public Task Header

  @Company
 Team 5

  @File Name
    motor_control_public.h

  @Summary
 Public function related to motor control.

  @Description
 This declares the function of the motor control that are available to other files.
 */
/* ************************************************************************** */

#ifndef _MOTOR_CONTROL_PUBLIC_H    /* Guard against multiple inclusion */
#define _MOTOR_CONTROL_PUBLIC_H

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

    // *****************************************************************************
    // *****************************************************************************
    // Section: Interface Functions
    // *****************************************************************************
    // *****************************************************************************
    
    BaseType_t pushEncoderFromISR(unsigned int dataValue);
    void pushLengthCommand(unsigned int dataValue);
    
    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _MOTOR_CONTROL_PUBLIC_H */

/* *****************************************************************************
 End of File
 */
